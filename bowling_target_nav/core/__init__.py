"""
Core Module
===========

Contains core functionality shared across the robot system:
- Configuration management
- State machine
- Event system
- Factory patterns
"""

from .config import Config, get_config
from .state_machine import RobotState, RobotStateMachine
from .events import EventBus, Event

__all__ = [
    'Config',
    'get_config',
    'RobotState',
    'RobotStateMachine',
    'EventBus',
    'Event',
]
