"""Navigation algorithms -- holonomic drive, obstacle avoidance, and target selection.

Modules:
    navigator.py       -- Navigator class: velocity commands, blind approach,
                          search rotation, spiral search, obstacle avoidance.
    target_selector.py -- find_best_target(): picks highest-priority detection
                          from a list of Det objects (used by TargetTracker).

Called from:
    app/nav_controller.py -- owns a Navigator, calls its methods each tick.
    app/target_tracker.py -- calls find_best_target() before Kalman filtering.
"""

from .navigator import Navigator
from .target_selector import find_best_target

__all__ = ['Navigator', 'find_best_target']
