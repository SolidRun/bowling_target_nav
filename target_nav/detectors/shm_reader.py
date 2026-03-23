"""Shared-memory IPC for DRP-AI detection results and calibration.

This module provides zero-copy shared-memory communication between the
C++ DRP-AI binary and the Python camera pipeline:

  - **DetectionShmReader**: Reads detection structs from
    ``/dev/shm/v2n_detections``, written by the C++ binary once per
    inference pass. Each struct contains bbox, confidence, distance,
    angle, and a clipped flag. Reading is ~500x faster than parsing
    equivalent JSON from stdout. The reader tracks a sequence number
    to skip already-consumed data.

  - **CalibrationShmWriter**: Writes calibration parameters (reference
    box height, reference distance, camera FOV) to
    ``/dev/shm/v2n_calibration`` for the C++ binary to pick up within
    ~2 seconds. This enables live parameter tuning from the Settings
    GUI without restarting the binary.

Architecture:
  - Runs inside the camera worker thread (camera_worker.py).
  - DetectionShmReader is the preferred detection source in stream mode;
    the JSON stdout pipe is only used as a fallback.
  - CalibrationShmWriter is created lazily on first calibration change
    and cleaned up when the camera thread exits.

Shared memory layout (detection):
  Header (48 bytes): [uint32 n_dets][uint32 seq][float infer_ms][padding]
  Per-entry (48 bytes): [float x1][float y1][float x2][float y2]
                         [float conf][float dist][float angle]
                         [int32 class_id][uint8 clipped][padding]

Shared memory layout (calibration):
  16 bytes: [float ref_box_height][float ref_distance][float camera_fov]
            [uint32 seq]

Thread safety: DetectionShmReader.read() is called only from the camera
thread. CalibrationShmWriter.write() is also called only from the camera
thread (via _sync_calibration). No cross-thread access.
"""

import mmap
import os
import struct
from typing import List, Optional

from target_nav.config import TARGET_CLASS_NAME
from target_nav.detectors.detection import Det
from target_nav.utils.log import get_logger

logger = get_logger('detectors.shm_reader')

# --- Shared memory paths and layout sizes ---
from target_nav.config import SHM_DETECTIONS_PATH, SHM_CALIBRATION_PATH
DET_SHM_PATH = SHM_DETECTIONS_PATH
CAL_SHM_PATH = SHM_CALIBRATION_PATH
DET_HEADER_SIZE = 48   # Header: n_dets(u32) + seq(u32) + infer_ms(f32) + padding
DET_ENTRY_SIZE = 48    # Per-detection: 7 floats + class_id(i32) + clipped(u8) + padding
MAX_DETS = 20          # Max detections per frame (matches C++ MAX_DETECTIONS)
CAL_SHM_SIZE = 16      # 3 floats + 1 uint32 sequence counter


class DetectionShmReader:
    """Read binary detection structs from /dev/shm/v2n_detections.

    The C++ DRP-AI binary writes a fixed-size header followed by up to
    MAX_DETS detection entries after each inference pass. This reader
    attaches lazily (on first read or explicit attach()), tracks a
    sequence number to avoid re-reading the same data, and auto-detaches
    on read errors so the next call retries attachment.

    Lifecycle:
      1. Camera thread calls ``attach()`` (or lets ``read()`` do it).
      2. Camera thread calls ``read()`` each frame -- returns new data
         or None if the sequence hasn't advanced.
      3. Camera thread calls ``close()`` on shutdown.
    """

    def __init__(self, labels: Optional[List[str]] = None):
        """Initialize the shared-memory reader.

        Args:
            labels: Class name list indexed by class_id. When the C++ binary
                writes a class_id, this list maps it to a human-readable name.
                Defaults to ``[TARGET_CLASS_NAME]`` for single-class detection.
        """
        self._labels = labels if labels is not None else [TARGET_CLASS_NAME]
        self._fd = -1        # File descriptor for the shm file
        self._mmap = None    # mmap object for zero-copy reads
        self._last_seq = 0   # Last consumed sequence number
        self._attached = False

    def attach(self) -> bool:
        """Open and mmap the detection shared memory region.

        Safe to call repeatedly -- returns True immediately if already
        attached. On failure, cleans up partial state so the next call
        retries cleanly.

        Returns:
            True if the shm region is now mapped and ready for read().
        """
        if self._attached:
            return True
        try:
            if not os.path.exists(DET_SHM_PATH):
                return False
            self._fd = os.open(DET_SHM_PATH, os.O_RDONLY)
            size = os.fstat(self._fd).st_size
            if size < DET_HEADER_SIZE:
                os.close(self._fd)
                self._fd = -1
                return False
            self._mmap = mmap.mmap(self._fd, size, mmap.MAP_SHARED, mmap.PROT_READ)
            self._attached = True
            # Skip any stale data from a previous session by reading
            # the current sequence number. Only new detections (seq > this)
            # will be returned by read().
            try:
                self._mmap.seek(0)
                header = self._mmap.read(DET_HEADER_SIZE)
                _, seq, _, _ = struct.unpack('<IIfI', header[:16])
                self._last_seq = seq
                logger.info("Attached to detection shm (skipping stale seq=%d)", seq)
            except Exception:
                self._last_seq = 0
                logger.info("Attached to detection shm")
            return True
        except Exception as e:
            logger.warning("Cannot attach to detection shm: %s", e)
            if self._fd >= 0:
                os.close(self._fd)
                self._fd = -1
            return False

    def read(self) -> Optional[tuple]:
        """Read latest detections from shared memory.

        Attempts to attach if not already connected. Compares the
        sequence number in the header against the last consumed value;
        returns None if they match (no new data). On any read error,
        auto-detaches so the next call retries attachment.

        Returns:
            Tuple of (list[Det], inference_ms: float, seq: int) when
            new data is available, or None if no new data / not attached.
        """
        if not self._attached:
            if not self.attach():
                return None

        try:
            self._mmap.seek(0)
            header = self._mmap.read(DET_HEADER_SIZE)
            if len(header) < DET_HEADER_SIZE:
                return None

            # Parse the fixed-size header at offset 0
            n_dets = struct.unpack_from('<I', header, 0)[0]   # number of detections
            seq = struct.unpack_from('<I', header, 4)[0]      # monotonic sequence counter
            infer_ms = struct.unpack_from('<f', header, 8)[0] # DRP-AI inference time (ms)

            if seq == self._last_seq:
                return None  # No new data since last read
            self._last_seq = seq

            n = min(n_dets, MAX_DETS)
            dets = []
            for i in range(n):
                entry = self._mmap.read(DET_ENTRY_SIZE)
                if len(entry) < DET_ENTRY_SIZE:
                    break
                # Unpack 7 floats: bbox corners, confidence, distance, angle
                x1, y1, x2, y2, conf, dist, angle = struct.unpack_from('<fffffff', entry, 0)
                class_id = struct.unpack_from('<i', entry, 28)[0]  # signed int at offset 28
                clipped = entry[32] != 0  # single byte flag at offset 32

                # Sanity-check values from shared memory (guard against
                # corrupted reads if the writer is mid-update)
                if conf < 0.0 or conf > 1.0:
                    continue
                if dist < 0.0 or dist > 100.0:
                    dist = 0.0

                # Map class_id to a label; fall back to first label or "unknown"
                if 0 <= class_id < len(self._labels):
                    label = self._labels[class_id]
                elif self._labels:
                    label = self._labels[0]
                else:
                    label = "unknown"

                dets.append(Det(
                    class_name=label,
                    confidence=conf,
                    bbox=(int(x1), int(y1), int(x2), int(y2)),
                    distance=dist,
                    angle=angle,
                    bbox_clipped=clipped,
                ))

            # Re-read sequence number to detect torn data (C++ mid-write)
            self._mmap.seek(4)  # offset of seq field in header
            seq_check = struct.unpack_from('<I', self._mmap.read(4), 0)[0]
            if seq_check != seq:
                return None  # Writer was mid-update, discard

            return dets, infer_ms, seq

        except Exception as e:
            logger.warning("Detection shm read error: %s", e)
            self._attached = False
            try:
                if self._mmap:
                    self._mmap.close()
            except Exception:
                pass
            try:
                if self._fd >= 0:
                    os.close(self._fd)
            except Exception:
                pass
            self._mmap = None
            self._fd = -1
            return None

    def close(self):
        """Detach from shared memory."""
        if self._mmap:
            try:
                self._mmap.close()
            except Exception:
                pass
        if self._fd >= 0:
            try:
                os.close(self._fd)
            except Exception:
                pass
        self._mmap = None
        self._fd = -1
        self._attached = False


class CalibrationShmWriter:
    """Write calibration parameters to /dev/shm/v2n_calibration for the C++ binary.

    The C++ DRP-AI binary polls this region every ~2 seconds to pick up
    live calibration changes from the Python Settings GUI. Layout:
      [float ref_box_height][float ref_distance][float camera_fov][uint32 seq]

    Created lazily on first write. The sequence counter lets the C++ side
    detect stale data (only apply when seq advances).
    """

    def __init__(self):
        """Initialize writer state. The shm region is created lazily on first write()."""
        self._fd = -1       # File descriptor for the shm file
        self._mmap = None   # mmap object for writes
        self._seq = 0       # Monotonic sequence counter
        self._created = False

    def create(self) -> bool:
        """Create or open the calibration shared memory file and mmap it.

        Returns:
            True if the region is now mapped and ready for write().
        """
        try:
            path = CAL_SHM_PATH
            self._fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
            os.ftruncate(self._fd, CAL_SHM_SIZE)
            self._mmap = mmap.mmap(self._fd, CAL_SHM_SIZE)
            self._created = True
            logger.info("Calibration shm created at %s", path)
            return True
        except Exception as e:
            logger.warning("Cannot create calibration shm: %s", e)
            if self._fd >= 0:
                try:
                    os.close(self._fd)
                except Exception:
                    pass
                self._fd = -1
            return False

    def write(self, ref_box_height: float, ref_distance: float, camera_fov: float) -> bool:
        """Write calibration parameters to shared memory.

        Increments the sequence counter so the C++ binary detects the
        change. Creates the shm region lazily if it does not exist yet.

        Args:
            ref_box_height: Reference bbox height in pixels at ref_distance.
            ref_distance: Distance (meters) at which ref_box_height was measured.
            camera_fov: Horizontal field of view in degrees.

        Returns:
            True if the write succeeded.
        """
        if not self._created:
            if not self.create():
                return False
        try:
            self._seq += 1
            data = struct.pack('<fffI', ref_box_height, ref_distance, camera_fov, self._seq)
            self._mmap.seek(0)
            self._mmap.write(data)
            self._mmap.flush()
            return True
        except Exception as e:
            logger.warning("Calibration shm write error: %s", e)
            return False

    def close(self):
        """Unmap and close the calibration shared memory file descriptor."""
        if self._mmap:
            try:
                self._mmap.close()
            except Exception:
                pass
        if self._fd >= 0:
            try:
                os.close(self._fd)
            except Exception:
                pass
        self._mmap = None
        self._fd = -1
        self._created = False
