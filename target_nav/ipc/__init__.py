"""Lock-free IPC infrastructure for cross-process communication.

Provides struct-based shared memory (SHM) channels that replace the old
JSON-over-SHM Bridge process. Each channel has exactly ONE writer and
ONE reader (SPSC) — no locks needed.

Modules:
    shm_struct  -- NavShmWriter/Reader, LaserShmWriter/Reader,
                   DetShmWriter/Reader, CmdRingBuffer, DoubleBuffer.
"""
