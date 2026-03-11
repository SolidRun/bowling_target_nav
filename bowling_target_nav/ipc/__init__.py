"""IPC package — multiprocessing shared memory channels and hub."""

from .hub import IPCHub
from .map_info import MapInfo

__all__ = ['IPCHub', 'MapInfo']
