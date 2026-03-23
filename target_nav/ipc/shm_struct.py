"""
Lock-free shared memory communication for the V2N robot.

This module replaces the old JSON-over-SHM approach with fixed-layout binary
structs mapped directly into shared memory via mmap.  The fundamental invariant
is **single-producer / single-consumer (SPSC)** — exactly ONE writer and ONE
reader per channel.  Under this constraint no locks are required.

Torn-read detection
-------------------
Every message is bracketed by a sequence number:

    [seq_start] [payload ...] [seq_end]

The writer increments a monotonic counter, writes seq_start, fills the payload,
then writes seq_end equal to seq_start.  The reader checks that seq_start ==
seq_end **and** that the value changed since the last successful read.  If the
two halves disagree the read was "torn" (writer was mid-update) and the reader
returns None — the caller simply tries again on the next tick.

Because the writer is the only process mutating the buffer and the reader never
writes, there is no ABA problem and no need for CAS / futex / mutex.

Shared-memory paths live under ``/dev/shm`` so they are backed by tmpfs and
never touch the SD card.

Channels
--------
==========  ============  ============================
Path        Direction     Content
==========  ============  ============================
v2n_nav     nav  -> GUI   Navigation state snapshot
v2n_laser   nav  -> GUI   Down-sampled LiDAR point cloud
v2n_det     cam  -> GUI   Object detections
v2n_cmd     GUI  -> nav   Command ring buffer
==========  ============  ============================
"""

import mmap
import os
import struct
import time
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# SHM file paths
# ---------------------------------------------------------------------------

SHM_NAV = '/dev/shm/v2n_nav'
SHM_LASER = '/dev/shm/v2n_laser'
SHM_DET = '/dev/shm/v2n_det'
SHM_CMD = '/dev/shm/v2n_cmd'

# ---------------------------------------------------------------------------
# Nav state enum
# ---------------------------------------------------------------------------

NAV_STATES = [
    'IDLE',            # 0
    'NAVIGATING',      # 1
    'SEARCHING',       # 2
    'BLIND_APPROACH',  # 3
    'SPIRAL_SEARCH',   # 4
    'ARRIVED',         # 5
    'ERROR',           # 6
]

# ---------------------------------------------------------------------------
# NavSnapshot dataclass
# ---------------------------------------------------------------------------

@dataclass
class NavSnapshot:
    """All navigation state fields the GUI needs to render a single frame.

    Populated by ``NavShmReader.read()`` and handed through a ``DoubleBuffer``
    to the GTK draw thread so the rendering code never touches shared memory
    directly.
    """
    timestamp: float = 0.0
    nav_state: int = 0
    nav_state_name: str = 'IDLE'
    lidar_at_target: float = 0.0
    time_since_target: float = 0.0
    target_robot_x: float = 0.0
    target_robot_y: float = 0.0
    nav_target_map_x: float = 0.0
    nav_target_map_y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    obstacle_ahead: bool = False
    obstacle_dist: float = 0.0
    search_time: float = 0.0
    detector_mode: str = ''

# =========================================================================
# Helpers
# =========================================================================

def _open_shm_write(path: str, size: int):
    """Create (or re-create) a SHM file, truncate, and mmap read-write."""
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o666)
    os.ftruncate(fd, size)
    mm = mmap.mmap(fd, size)
    os.close(fd)
    return mm


def _open_shm_read(path: str, size: int):
    """Open an existing SHM file read-only and mmap it.

    Returns ``None`` if the file does not exist yet (the writer has not
    started).
    """
    try:
        fd = os.open(path, os.O_RDONLY)
    except FileNotFoundError:
        return None
    try:
        mm = mmap.mmap(fd, size, access=mmap.ACCESS_READ)
    finally:
        os.close(fd)
    return mm


# =========================================================================
# Nav channel  (nav process -> GUI)
# =========================================================================

# struct format (little-endian, no padding holes because we order carefully):
#   I  seq_start
#   d  timestamp          (8)
#   B  nav_state          (1)
#   f  lidar_at_target    (4)
#   f  time_since_target  (4)
#   f  target_robot_x     (4)
#   f  target_robot_y     (4)
#   f  nav_target_map_x   (4)
#   f  nav_target_map_y   (4)
#   f  vx                 (4)
#   f  vy                 (4)
#   f  wz                 (4)
#   B  obstacle_ahead     (1)
#   f  obstacle_dist      (4)
#   f  search_time        (4)
#   B  detector_mode_len  (1)
#   32s detector_mode     (32)
#   I  seq_end
# Pack format (18 items, 95 bytes):
#   '<' = little-endian
#   I   = seq_start (uint32)         -- torn-read sentinel, written FIRST
#   d   = timestamp (float64, 8B)    -- time.time() epoch seconds
#   B   = nav_state (uint8)          -- index into NAV_STATES enum
#   fffff = lidar_at_target, time_since_target, target_robot_x,
#           target_robot_y, nav_target_map_x  (5 x float32)
#   ffff  = nav_target_map_y, vx, vy, wz     (4 x float32)
#   B   = obstacle_ahead (uint8)     -- 0 or 1
#   f   = obstacle_dist (float32)    -- meters, 999.0 if none
#   f   = search_time (float32)      -- seconds in search mode
#   B   = detector_mode_len (uint8)  -- length of mode string
#   32s = detector_mode (32 bytes)   -- UTF-8, null-padded
#   I   = seq_end (uint32)           -- must match seq_start for valid read
_NAV_FMT = '<I dB fffff ffff Bf fB 32s I'
_NAV_SIZE = struct.calcsize(_NAV_FMT)  # 95 bytes


class NavShmWriter:
    """Write navigation state into shared memory.

    Exactly ONE nav process instance should create this writer.  The
    ``write()`` method packs the entire struct atomically (from the reader's
    perspective) by bracketing the payload with a monotonic sequence number.
    """

    def __init__(self):
        self._mm = _open_shm_write(SHM_NAV, _NAV_SIZE)
        self._seq = 0

    def write(self, timestamp: float, nav_state: int, lidar_at_target: float,
              time_since_target: float, target_robot_x: float,
              target_robot_y: float, nav_target_map_x: float,
              nav_target_map_y: float, vx: float, vy: float, wz: float,
              obstacle_ahead: bool, obstacle_dist: float,
              search_time: float, detector_mode: str) -> None:
        """Pack all fields into the mmap buffer."""
        self._seq += 1
        seq = self._seq
        mode_bytes = detector_mode.encode('utf-8')[:32]
        mode_len = len(mode_bytes)
        mode_padded = mode_bytes.ljust(32, b'\x00')

        struct.pack_into(
            _NAV_FMT, self._mm, 0,
            seq,
            timestamp,
            nav_state,
            lidar_at_target,
            time_since_target,
            target_robot_x,
            target_robot_y,
            nav_target_map_x,
            nav_target_map_y,
            vx, vy, wz,
            1 if obstacle_ahead else 0,
            obstacle_dist,
            search_time,
            mode_len,
            mode_padded,
            seq,  # seq_end
        )

    def close(self) -> None:
        """Release the mmap (does NOT delete the SHM file)."""
        if self._mm is not None:
            self._mm.close()
            self._mm = None


class NavShmReader:
    """Read navigation state from shared memory.

    Exactly ONE GUI process instance should create this reader.  ``read()``
    returns a ``NavSnapshot`` on success or ``None`` when the SHM file does
    not exist or a torn read is detected.
    """

    def __init__(self):
        self._mm = None
        self._last_seq = 0

    def read(self) -> 'NavSnapshot | None':
        """Attempt a consistent read.  Returns *None* on failure."""
        if self._mm is None:
            self._mm = _open_shm_read(SHM_NAV, _NAV_SIZE)
            if self._mm is None:
                return None

        try:
            vals = struct.unpack_from(_NAV_FMT, self._mm, 0)
        except (struct.error, ValueError):
            return None

        seq_start = vals[0]
        seq_end = vals[-1]

        # Torn-read check
        if seq_start != seq_end or seq_start == 0:
            return None
        # No new data
        if seq_start == self._last_seq:
            return None
        self._last_seq = seq_start

        state_idx = vals[2]
        state_name = NAV_STATES[state_idx] if state_idx < len(NAV_STATES) else 'ERROR'

        mode_len = vals[15]
        mode_raw: bytes = vals[16]
        detector_mode = mode_raw[:mode_len].decode('utf-8', errors='replace')

        return NavSnapshot(
            timestamp=vals[1],
            nav_state=state_idx,
            nav_state_name=state_name,
            lidar_at_target=vals[3],
            time_since_target=vals[4],
            target_robot_x=vals[5],
            target_robot_y=vals[6],
            nav_target_map_x=vals[7],
            nav_target_map_y=vals[8],
            vx=vals[9],
            vy=vals[10],
            wz=vals[11],
            obstacle_ahead=bool(vals[12]),
            obstacle_dist=vals[13],
            search_time=vals[14],
            detector_mode=detector_mode,
        )

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None


# =========================================================================
# Laser channel  (nav process -> GUI)
# =========================================================================

_LASER_MAX_POINTS = 400
# Header: seq_start(I) n_points(I) pose_x(f) pose_y(f) pose_theta(f) scan_hz(f) n_raw(I)
_LASER_HDR_FMT = '<I I f f f f I'
_LASER_HDR_SIZE = struct.calcsize(_LASER_HDR_FMT)  # 28
# Each point: x(f) y(f)
_LASER_PT_FMT = '<ff'
_LASER_PT_SIZE = struct.calcsize(_LASER_PT_FMT)  # 8
# Footer: seq_end(I)
_LASER_FTR_FMT = '<I'
_LASER_FTR_SIZE = struct.calcsize(_LASER_FTR_FMT)  # 4
_LASER_TOTAL = _LASER_HDR_SIZE + _LASER_MAX_POINTS * _LASER_PT_SIZE + _LASER_FTR_SIZE  # 3232


class LaserShmWriter:
    """Write down-sampled LiDAR scan points into shared memory.

    The nav process calls ``write()`` once per scan with a list of (x, y)
    tuples in the robot's map frame plus the robot pose used to transform
    them.  Points beyond ``_LASER_MAX_POINTS`` are silently dropped.
    """

    def __init__(self):
        self._mm = _open_shm_write(SHM_LASER, _LASER_TOTAL)
        self._seq = 0

    def write(self, points: 'list[tuple[float, float]]',
              pose_x: float, pose_y: float, pose_theta: float,
              scan_hz: float = 0.0, n_raw: int = 0) -> None:
        """Pack header, point array, and footer into the mmap buffer."""
        self._seq += 1
        seq = self._seq
        n = min(len(points), _LASER_MAX_POINTS)

        # Header
        struct.pack_into(_LASER_HDR_FMT, self._mm, 0,
                         seq, n, pose_x, pose_y, pose_theta, scan_hz, n_raw)

        # Points
        off = _LASER_HDR_SIZE
        for i in range(n):
            struct.pack_into(_LASER_PT_FMT, self._mm, off, points[i][0], points[i][1])
            off += _LASER_PT_SIZE

        # Footer (always at fixed offset so reader knows where to find it)
        ftr_off = _LASER_HDR_SIZE + _LASER_MAX_POINTS * _LASER_PT_SIZE
        struct.pack_into(_LASER_FTR_FMT, self._mm, ftr_off, seq)

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None


class LaserShmReader:
    """Read LiDAR points from shared memory.

    Returns a tuple ``(points, pose_x, pose_y, pose_theta, scan_hz, n_raw)``
    where *points* is a ``list[tuple[float, float]]``.  Returns ``None`` on
    torn read or missing SHM file.
    """

    def __init__(self):
        self._mm = None
        self._last_seq = 0

    def read(self):
        """Attempt a consistent read of the laser scan.

        Returns
        -------
        tuple | None
            ``(points, pose_x, pose_y, pose_theta, scan_hz, n_raw)`` on
            success, ``None`` on failure or no new data.
        """
        if self._mm is None:
            self._mm = _open_shm_read(SHM_LASER, _LASER_TOTAL)
            if self._mm is None:
                return None

        try:
            hdr = struct.unpack_from(_LASER_HDR_FMT, self._mm, 0)
        except (struct.error, ValueError):
            return None

        seq_start = hdr[0]
        n_points = hdr[1]
        pose_x = hdr[2]
        pose_y = hdr[3]
        pose_theta = hdr[4]
        scan_hz = hdr[5]
        n_raw = hdr[6]

        # Read footer
        ftr_off = _LASER_HDR_SIZE + _LASER_MAX_POINTS * _LASER_PT_SIZE
        try:
            (seq_end,) = struct.unpack_from(_LASER_FTR_FMT, self._mm, ftr_off)
        except (struct.error, ValueError):
            return None

        if seq_start != seq_end or seq_start == 0:
            return None
        if seq_start == self._last_seq:
            return None
        self._last_seq = seq_start

        # Clamp to safe range
        n_points = min(n_points, _LASER_MAX_POINTS)

        points = []
        off = _LASER_HDR_SIZE
        for i in range(n_points):
            x, y = struct.unpack_from(_LASER_PT_FMT, self._mm, off)
            points.append((x, y))
            off += _LASER_PT_SIZE

        return (points, pose_x, pose_y, pose_theta, scan_hz, n_raw)

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None


# =========================================================================
# Detection channel  (camera process -> GUI)
# =========================================================================

_DET_MAX = 5
# Header: seq_start(I) timestamp(d) n_dets(B)
_DET_HDR_FMT = '<I d B'
_DET_HDR_SIZE = struct.calcsize(_DET_HDR_FMT)  # 13
# Per detection: distance(f) angle(f) confidence(f)
#                bbox_x1(H) bbox_y1(H) bbox_x2(H) bbox_y2(H)
#                bbox_clipped(B) class_name_len(B) class_name(16s)
_DET_ITEM_FMT = '<fff HHHH BB 16s'
_DET_ITEM_SIZE = struct.calcsize(_DET_ITEM_FMT)  # 30
# Info string: info_len(B) info(64s)
_DET_INFO_FMT = '<B 64s'
_DET_INFO_SIZE = struct.calcsize(_DET_INFO_FMT)  # 65
# Footer: seq_end(I)
_DET_FTR_FMT = '<I'
_DET_FTR_SIZE = struct.calcsize(_DET_FTR_FMT)  # 4
_DET_TOTAL = _DET_HDR_SIZE + _DET_MAX * _DET_ITEM_SIZE + _DET_INFO_SIZE + _DET_FTR_SIZE  # 232


@dataclass
class Detection:
    """A single object detection as unpacked from shared memory."""
    distance: float = 0.0
    angle: float = 0.0
    confidence: float = 0.0
    bbox_x1: int = 0
    bbox_y1: int = 0
    bbox_x2: int = 0
    bbox_y2: int = 0
    bbox_clipped: bool = False
    class_name: str = ''


class DetShmWriter:
    """Write object detections into shared memory.

    The camera process calls ``write()`` once per frame with a list of
    detection dicts and an optional info string for debug overlay.
    """

    def __init__(self):
        self._mm = _open_shm_write(SHM_DET, _DET_TOTAL)
        self._seq = 0

    def write(self, timestamp: float,
              detections: 'list[dict]',
              info: str = '') -> None:
        """Pack detections into the mmap buffer.

        Parameters
        ----------
        timestamp : float
            ``time.time()`` epoch timestamp of the detection frame.
        detections : list[dict]
            Each dict must have keys: ``distance``, ``angle``, ``confidence``,
            ``bbox`` (x1, y1, x2, y2), ``bbox_clipped``, ``class_name``.
        info : str
            Short status string (max 64 bytes UTF-8) for debug overlay.
        """
        self._seq += 1
        seq = self._seq
        n = min(len(detections), _DET_MAX)

        # Header
        struct.pack_into(_DET_HDR_FMT, self._mm, 0, seq, timestamp, n)

        # Detections
        off = _DET_HDR_SIZE
        for i in range(n):
            d = detections[i]
            bbox = d.get('bbox', (0, 0, 0, 0))
            cname = d.get('class_name', '')
            cname_bytes = cname.encode('utf-8')[:16]
            cname_len = len(cname_bytes)
            cname_padded = cname_bytes.ljust(16, b'\x00')

            struct.pack_into(
                _DET_ITEM_FMT, self._mm, off,
                float(d.get('distance', 0.0)),
                float(d.get('angle', 0.0)),
                float(d.get('confidence', 0.0)),
                int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]),
                1 if d.get('bbox_clipped', False) else 0,
                cname_len,
                cname_padded,
            )
            off += _DET_ITEM_SIZE

        # Zero out unused detection slots so stale data is not visible
        for i in range(n, _DET_MAX):
            struct.pack_into(_DET_ITEM_FMT, self._mm, off,
                             0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0,
                             b'\x00' * 16)
            off += _DET_ITEM_SIZE

        # Info string
        info_bytes = info.encode('utf-8')[:64]
        info_len = len(info_bytes)
        info_padded = info_bytes.ljust(64, b'\x00')
        struct.pack_into(_DET_INFO_FMT, self._mm, off, info_len, info_padded)
        off += _DET_INFO_SIZE

        # Footer
        struct.pack_into(_DET_FTR_FMT, self._mm, off, seq)

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None


class DetShmReader:
    """Read object detections from shared memory.

    Returns a tuple ``(timestamp, detections, info)`` where *detections* is a
    list of ``Detection`` dataclass instances.  Returns ``None`` on torn read
    or missing SHM file.
    """

    def __init__(self):
        self._mm = None
        self._last_seq = 0

    def read(self):
        """Attempt a consistent read of the detection buffer.

        Returns
        -------
        tuple | None
            ``(timestamp, detections, info)`` on success, ``None`` otherwise.
        """
        if self._mm is None:
            self._mm = _open_shm_read(SHM_DET, _DET_TOTAL)
            if self._mm is None:
                return None

        try:
            hdr = struct.unpack_from(_DET_HDR_FMT, self._mm, 0)
        except (struct.error, ValueError):
            return None

        seq_start = hdr[0]
        timestamp = hdr[1]
        n_dets = hdr[2]

        # Read footer
        ftr_off = _DET_HDR_SIZE + _DET_MAX * _DET_ITEM_SIZE + _DET_INFO_SIZE
        try:
            (seq_end,) = struct.unpack_from(_DET_FTR_FMT, self._mm, ftr_off)
        except (struct.error, ValueError):
            return None

        if seq_start != seq_end or seq_start == 0:
            return None
        if seq_start == self._last_seq:
            return None
        self._last_seq = seq_start

        n_dets = min(n_dets, _DET_MAX)
        detections = []
        off = _DET_HDR_SIZE
        for i in range(n_dets):
            vals = struct.unpack_from(_DET_ITEM_FMT, self._mm, off)
            cname_len = vals[8]
            cname_raw: bytes = vals[9]
            cname = cname_raw[:cname_len].decode('utf-8', errors='replace')

            detections.append(Detection(
                distance=vals[0],
                angle=vals[1],
                confidence=vals[2],
                bbox_x1=vals[3],
                bbox_y1=vals[4],
                bbox_x2=vals[5],
                bbox_y2=vals[6],
                bbox_clipped=bool(vals[7]),
                class_name=cname,
            ))
            off += _DET_ITEM_SIZE

        # Info string
        info_off = _DET_HDR_SIZE + _DET_MAX * _DET_ITEM_SIZE
        info_vals = struct.unpack_from(_DET_INFO_FMT, self._mm, info_off)
        info_len = info_vals[0]
        info_raw: bytes = info_vals[1]
        info = info_raw[:info_len].decode('utf-8', errors='replace')

        return (timestamp, detections, info)

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None


# =========================================================================
# Command ring buffer  (GUI -> nav process)
# =========================================================================

# SPSC (single-producer single-consumer) ring buffer.
#
# Layout:
#   head(I)  — written ONLY by producer (GUI) after filling a slot
#   tail(I)  — written ONLY by consumer (nav) after processing a slot
#   slots[8] — each 64 bytes: cmd_type(B) + payload(63B)
#
# The producer writes to slots[head % 8], then increments head.
# The consumer reads from slots[tail % 8], then increments tail.
# Buffer is full when (head - tail) == 8.
# Buffer is empty when head == tail.
#
# Because head and tail are each written by exactly one side, no locks or
# atomics are needed on architectures with word-sized atomic stores (all
# modern ARM and x86 CPUs).

_CMD_SLOTS = 8
_CMD_SLOT_SIZE = 64
_CMD_HDR_FMT = '<I I'  # head, tail
_CMD_HDR_SIZE = struct.calcsize(_CMD_HDR_FMT)  # 8
_CMD_TOTAL = _CMD_HDR_SIZE + _CMD_SLOTS * _CMD_SLOT_SIZE  # 520
_CMD_SLOT_FMT = '<B 63s'

# Command type constants
CMD_NONE = 0
CMD_GO = 1
CMD_STOP = 2
CMD_SET_PARAM = 3
CMD_TEST_MOTOR = 4


class CmdRingBuffer:
    """Lock-free SPSC ring buffer for sending commands from GUI to nav.

    The GUI process is the sole *producer* (calls ``push``).
    The nav process is the sole *consumer* (calls ``pop``).

    Both sides open the same SHM file.  The GUI side should call the
    class method ``create()`` to initialise the file; the nav side calls
    ``open()`` to attach to an existing buffer.
    """

    def __init__(self, mm: mmap.mmap, is_writer: bool):
        self._mm = mm
        self._is_writer = is_writer

    # -- construction helpers -----------------------------------------------

    @classmethod
    def create(cls) -> 'CmdRingBuffer':
        """Create the SHM file and return a producer instance."""
        mm = _open_shm_write(SHM_CMD, _CMD_TOTAL)
        # Zero head and tail
        struct.pack_into(_CMD_HDR_FMT, mm, 0, 0, 0)
        return cls(mm, is_writer=True)

    @classmethod
    def open(cls) -> 'CmdRingBuffer | None':
        """Attach to an existing SHM file as a consumer.

        Returns ``None`` if the file does not exist yet.
        """
        mm = _open_shm_read(SHM_CMD, _CMD_TOTAL)
        if mm is None:
            return None
        # Re-open read-write so the consumer can update ``tail``
        try:
            fd = os.open(SHM_CMD, os.O_RDWR)
        except FileNotFoundError:
            return None
        try:
            mm_rw = mmap.mmap(fd, _CMD_TOTAL)
        finally:
            os.close(fd)
        return cls(mm_rw, is_writer=False)

    # -- producer API -------------------------------------------------------

    def push(self, cmd_type: int, payload: bytes = b'') -> bool:
        """Enqueue a command.  Returns False if the buffer is full."""
        head, tail = struct.unpack_from(_CMD_HDR_FMT, self._mm, 0)
        if (head - tail) >= _CMD_SLOTS:
            return False  # full

        slot_off = _CMD_HDR_SIZE + (head % _CMD_SLOTS) * _CMD_SLOT_SIZE
        padded = payload[:63].ljust(63, b'\x00')
        struct.pack_into(_CMD_SLOT_FMT, self._mm, slot_off, cmd_type, padded)

        # Increment head (single atomic 4-byte write)
        struct.pack_into('<I', self._mm, 0, head + 1)
        return True

    # -- consumer API -------------------------------------------------------

    def pop(self) -> 'tuple[int, bytes] | None':
        """Dequeue a command.  Returns ``None`` if the buffer is empty."""
        head, tail = struct.unpack_from(_CMD_HDR_FMT, self._mm, 0)
        if head == tail:
            return None  # empty

        slot_off = _CMD_HDR_SIZE + (tail % _CMD_SLOTS) * _CMD_SLOT_SIZE
        cmd_type, payload_raw = struct.unpack_from(_CMD_SLOT_FMT, self._mm, slot_off)

        # Increment tail (single atomic 4-byte write)
        struct.pack_into('<I', self._mm, 4, tail + 1)
        return (cmd_type, payload_raw)

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None


# =========================================================================
# DoubleBuffer  (intra-process, thread-safe without locks)
# =========================================================================

class DoubleBuffer:
    """Lock-free double buffer for thread-safe producer/consumer.

    The SHM poll thread writes to one buffer while GTK reads from the other.
    ``swap()`` is called once per GTK tick between draws to flip which side
    each thread sees.

    Why this is safe without locks
    ------------------------------
    * The producer only ever writes to ``_back``.
    * The consumer only ever reads from ``_front``.
    * ``swap()`` is called from the consumer's thread (the GTK main loop)
      between frames, so it never races with a read.
    * A swap while the producer is mid-write is harmless: the consumer gets
      the *previous* complete snapshot, and the producer finishes writing
      into what is now the front buffer — but the consumer won't read it
      until the *next* swap.

    Usage::

        db = DoubleBuffer(NavSnapshot)
        # producer thread:
        snap = NavSnapshot(...)
        db.write(snap)
        # consumer thread (GTK tick):
        db.swap()
        snap = db.read()
    """

    def __init__(self, factory=None):
        """
        Parameters
        ----------
        factory : callable, optional
            A zero-argument callable that returns a fresh default instance.
            If ``None``, both buffers start as ``None``.
        """
        if factory is not None:
            self._front = factory()
            self._back = factory()
        else:
            self._front = None
            self._back = None
        self._has_new = False

    def write(self, value):
        """Called by the producer thread to submit a new value."""
        self._back = value
        self._has_new = True

    def swap(self):
        """Called by the consumer thread to flip buffers.

        Only actually swaps if the producer has written since the last swap.
        """
        if self._has_new:
            self._front, self._back = self._back, self._front
            self._has_new = False

    def read(self):
        """Called by the consumer thread to get the latest value.

        Always returns the front buffer — never ``None`` after the first
        successful write + swap cycle.
        """
        return self._front


# =========================================================================
# Cleanup helper
# =========================================================================

def cleanup_all_shm() -> None:
    """Remove all four SHM files.

    Safe to call even if some or all files do not exist.  Intended for use
    at application shutdown or during development resets.
    """
    for path in (SHM_NAV, SHM_LASER, SHM_DET, SHM_CMD):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
