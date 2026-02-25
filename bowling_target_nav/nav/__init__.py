"""Navigation package - algorithms for target tracking and obstacle avoidance."""

from .navigator import Navigator
from .target_selector import find_best_target

__all__ = ['Navigator', 'find_best_target']
