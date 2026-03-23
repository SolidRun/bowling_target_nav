"""State package -- thread-safe shared state singleton.

Creates a module-level ``state`` singleton of :class:`SharedState` that is
imported by GUI code. ROS2 nodes maintain their own local state and
communicate with the GUI via ROS2 topics.
"""

from .shared_state import SharedState

state = SharedState()

__all__ = ['state', 'SharedState']
