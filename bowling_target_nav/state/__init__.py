"""State package - thread-safe shared state singleton."""

from .shared_state import SharedState

state = SharedState()

__all__ = ['state', 'SharedState']
