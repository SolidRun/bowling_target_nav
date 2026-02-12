"""
Robot State Machine
===================

Provides a clean state machine implementation for robot behavior control.
Uses the State pattern for extensibility and testability.
"""

import logging
import time
from abc import ABC, abstractmethod
from enum import Enum, auto
from typing import Dict, Optional, Callable, List, Any
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


class RobotState(Enum):
    """Robot operational states."""
    IDLE = auto()           # Robot is idle, waiting for commands
    DETECTING = auto()      # Actively looking for targets
    NAVIGATING = auto()     # Moving towards detected target
    SEARCHING = auto()      # Target lost, rotating to find it
    AVOIDING = auto()       # Avoiding an obstacle
    STOPPED = auto()        # Emergency stop activated
    ERROR = auto()          # Error state


@dataclass
class StateContext:
    """Context data shared between states."""
    # Target information
    target_detected: bool = False
    target_x: float = 0.0
    target_y: float = 0.0
    target_distance: float = float('inf')
    target_confidence: float = 0.0
    last_target_time: float = 0.0

    # Obstacle information
    obstacle_detected: bool = False
    obstacle_distance: float = float('inf')
    obstacle_angle: float = 0.0

    # Navigation
    current_linear_vel: float = 0.0
    current_angular_vel: float = 0.0

    # Timing
    state_enter_time: float = 0.0
    last_update_time: float = 0.0

    # Error tracking
    error_message: str = ""
    error_count: int = 0

    def reset_target(self):
        """Reset target information."""
        self.target_detected = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_distance = float('inf')
        self.target_confidence = 0.0

    def time_in_state(self) -> float:
        """Get time spent in current state."""
        return time.time() - self.state_enter_time

    def time_since_target(self) -> float:
        """Get time since last target detection."""
        if self.last_target_time == 0:
            return float('inf')
        return time.time() - self.last_target_time


@dataclass
class StateTransition:
    """Represents a state transition."""
    from_state: RobotState
    to_state: RobotState
    condition: str
    timestamp: float = field(default_factory=time.time)


class StateHandler(ABC):
    """Abstract base class for state handlers."""

    def __init__(self, state: RobotState):
        self.state = state

    @abstractmethod
    def enter(self, context: StateContext) -> None:
        """Called when entering this state."""
        pass

    @abstractmethod
    def update(self, context: StateContext) -> Optional[RobotState]:
        """
        Called every update cycle.
        Returns the next state or None to stay in current state.
        """
        pass

    @abstractmethod
    def exit(self, context: StateContext) -> None:
        """Called when exiting this state."""
        pass

    def get_velocity(self, context: StateContext) -> tuple:
        """Get velocity command for this state (linear, angular)."""
        return (0.0, 0.0)


class IdleStateHandler(StateHandler):
    """Handler for IDLE state."""

    def __init__(self):
        super().__init__(RobotState.IDLE)

    def enter(self, context: StateContext) -> None:
        logger.info("Entering IDLE state")
        context.current_linear_vel = 0.0
        context.current_angular_vel = 0.0

    def update(self, context: StateContext) -> Optional[RobotState]:
        # Transition to DETECTING when ready
        return RobotState.DETECTING

    def exit(self, context: StateContext) -> None:
        logger.info("Exiting IDLE state")


class DetectingStateHandler(StateHandler):
    """Handler for DETECTING state."""

    def __init__(self, lost_timeout: float = 10.0):
        super().__init__(RobotState.DETECTING)
        self.lost_timeout = lost_timeout

    def enter(self, context: StateContext) -> None:
        logger.info("Entering DETECTING state")

    def update(self, context: StateContext) -> Optional[RobotState]:
        if context.target_detected:
            context.last_target_time = time.time()
            return RobotState.NAVIGATING

        # Check if we should start searching
        if context.time_since_target() > self.lost_timeout:
            return RobotState.SEARCHING

        return None

    def exit(self, context: StateContext) -> None:
        logger.info("Exiting DETECTING state")

    def get_velocity(self, context: StateContext) -> tuple:
        return (0.0, 0.0)


class NavigatingStateHandler(StateHandler):
    """Handler for NAVIGATING state."""

    def __init__(
        self,
        linear_speed: float = 0.15,
        angular_speed: float = 0.3,
        target_reached_distance: float = 0.3,
        obstacle_threshold: float = 0.4,
        lost_timeout: float = 3.0
    ):
        super().__init__(RobotState.NAVIGATING)
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.target_reached_distance = target_reached_distance
        self.obstacle_threshold = obstacle_threshold
        self.lost_timeout = lost_timeout

    def enter(self, context: StateContext) -> None:
        logger.info("Entering NAVIGATING state")

    def update(self, context: StateContext) -> Optional[RobotState]:
        # Check for obstacles
        if context.obstacle_detected and context.obstacle_distance < self.obstacle_threshold:
            return RobotState.AVOIDING

        # Check if target is still visible
        if not context.target_detected:
            if context.time_since_target() > self.lost_timeout:
                return RobotState.DETECTING
            # Brief loss, keep going
            return None

        # Update last target time
        context.last_target_time = time.time()

        # Check if reached target
        if context.target_distance < self.target_reached_distance:
            logger.info(f"Target reached at distance {context.target_distance:.2f}m")
            return RobotState.IDLE

        return None

    def exit(self, context: StateContext) -> None:
        logger.info("Exiting NAVIGATING state")
        context.current_linear_vel = 0.0
        context.current_angular_vel = 0.0

    def get_velocity(self, context: StateContext) -> tuple:
        if not context.target_detected:
            return (0.0, 0.0)

        # Calculate angular velocity to turn towards target
        # target_x is normalized: -1 (left) to 1 (right)
        angular = -context.target_x * self.angular_speed

        # Linear velocity (reduce when turning)
        linear = self.linear_speed * (1.0 - abs(context.target_x) * 0.5)

        return (linear, angular)


class SearchingStateHandler(StateHandler):
    """Handler for SEARCHING state - rotates to find lost target."""

    def __init__(
        self,
        angular_speed: float = 0.25,
        search_timeout: float = 30.0
    ):
        super().__init__(RobotState.SEARCHING)
        self.angular_speed = angular_speed
        self.search_timeout = search_timeout

    def enter(self, context: StateContext) -> None:
        logger.info("Entering SEARCHING state - rotating to find target")

    def update(self, context: StateContext) -> Optional[RobotState]:
        # Found target
        if context.target_detected:
            context.last_target_time = time.time()
            return RobotState.NAVIGATING

        # Search timeout - go back to idle
        if context.time_in_state() > self.search_timeout:
            logger.warning("Search timeout - returning to IDLE")
            return RobotState.IDLE

        return None

    def exit(self, context: StateContext) -> None:
        logger.info("Exiting SEARCHING state")
        context.current_angular_vel = 0.0

    def get_velocity(self, context: StateContext) -> tuple:
        return (0.0, self.angular_speed)


class AvoidingStateHandler(StateHandler):
    """Handler for AVOIDING state - obstacle avoidance."""

    def __init__(
        self,
        linear_speed: float = 0.1,
        angular_speed: float = 0.4,
        safe_distance: float = 0.6,
        timeout: float = 10.0
    ):
        super().__init__(RobotState.AVOIDING)
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.safe_distance = safe_distance
        self.timeout = timeout

    def enter(self, context: StateContext) -> None:
        logger.info(f"Entering AVOIDING state - obstacle at {context.obstacle_distance:.2f}m")

    def update(self, context: StateContext) -> Optional[RobotState]:
        # Obstacle cleared
        if not context.obstacle_detected or context.obstacle_distance > self.safe_distance:
            return RobotState.DETECTING

        # Timeout
        if context.time_in_state() > self.timeout:
            logger.warning("Avoidance timeout")
            return RobotState.STOPPED

        return None

    def exit(self, context: StateContext) -> None:
        logger.info("Exiting AVOIDING state")

    def get_velocity(self, context: StateContext) -> tuple:
        # Turn away from obstacle
        if context.obstacle_angle >= 0:
            angular = -self.angular_speed  # Turn right
        else:
            angular = self.angular_speed   # Turn left

        # Move slowly backward or stay still
        linear = -self.linear_speed * 0.5 if context.obstacle_distance < 0.3 else 0.0

        return (linear, angular)


class StoppedStateHandler(StateHandler):
    """Handler for STOPPED state - emergency stop."""

    def __init__(self):
        super().__init__(RobotState.STOPPED)

    def enter(self, context: StateContext) -> None:
        logger.warning("EMERGENCY STOP activated")
        context.current_linear_vel = 0.0
        context.current_angular_vel = 0.0

    def update(self, context: StateContext) -> Optional[RobotState]:
        # Stay stopped until manually reset
        return None

    def exit(self, context: StateContext) -> None:
        logger.info("Exiting STOPPED state")

    def get_velocity(self, context: StateContext) -> tuple:
        return (0.0, 0.0)


class ErrorStateHandler(StateHandler):
    """Handler for ERROR state."""

    def __init__(self):
        super().__init__(RobotState.ERROR)

    def enter(self, context: StateContext) -> None:
        logger.error(f"Entering ERROR state: {context.error_message}")
        context.current_linear_vel = 0.0
        context.current_angular_vel = 0.0

    def update(self, context: StateContext) -> Optional[RobotState]:
        return None

    def exit(self, context: StateContext) -> None:
        logger.info("Exiting ERROR state")
        context.error_message = ""

    def get_velocity(self, context: StateContext) -> tuple:
        return (0.0, 0.0)


class RobotStateMachine:
    """
    Robot state machine controller.

    Manages state transitions and provides velocity commands based on current state.

    Usage:
        sm = RobotStateMachine()
        sm.start()

        # In control loop:
        sm.update_context(target_detected=True, target_x=0.1, ...)
        sm.update()
        linear, angular = sm.get_velocity()
    """

    def __init__(self, config: Optional[Any] = None):
        self.context = StateContext()
        self._current_state = RobotState.IDLE
        self._handlers: Dict[RobotState, StateHandler] = {}
        self._transition_history: List[StateTransition] = []
        self._callbacks: Dict[str, List[Callable]] = {
            'on_state_change': [],
            'on_target_detected': [],
            'on_target_lost': [],
            'on_obstacle_detected': [],
        }

        # Initialize with config or defaults
        if config:
            self._init_from_config(config)
        else:
            self._init_default_handlers()

    def _init_default_handlers(self) -> None:
        """Initialize default state handlers."""
        self._handlers = {
            RobotState.IDLE: IdleStateHandler(),
            RobotState.DETECTING: DetectingStateHandler(),
            RobotState.NAVIGATING: NavigatingStateHandler(),
            RobotState.SEARCHING: SearchingStateHandler(),
            RobotState.AVOIDING: AvoidingStateHandler(),
            RobotState.STOPPED: StoppedStateHandler(),
            RobotState.ERROR: ErrorStateHandler(),
        }

    def _init_from_config(self, config) -> None:
        """Initialize handlers from configuration."""
        nav = config.navigation

        self._handlers = {
            RobotState.IDLE: IdleStateHandler(),
            RobotState.DETECTING: DetectingStateHandler(
                lost_timeout=nav.lost_timeout
            ),
            RobotState.NAVIGATING: NavigatingStateHandler(
                linear_speed=nav.default_linear_speed,
                angular_speed=nav.default_angular_speed,
                target_reached_distance=nav.target_reached_distance,
                obstacle_threshold=nav.obstacle_distance_threshold,
            ),
            RobotState.SEARCHING: SearchingStateHandler(
                angular_speed=nav.search_angular_speed,
                search_timeout=nav.search_timeout,
            ),
            RobotState.AVOIDING: AvoidingStateHandler(
                safe_distance=nav.obstacle_slowdown_distance,
            ),
            RobotState.STOPPED: StoppedStateHandler(),
            RobotState.ERROR: ErrorStateHandler(),
        }

    def register_handler(self, state: RobotState, handler: StateHandler) -> None:
        """Register a custom state handler."""
        self._handlers[state] = handler

    def on(self, event: str, callback: Callable) -> None:
        """Register event callback."""
        if event in self._callbacks:
            self._callbacks[event].append(callback)

    def _emit(self, event: str, *args, **kwargs) -> None:
        """Emit event to callbacks."""
        for callback in self._callbacks.get(event, []):
            try:
                callback(*args, **kwargs)
            except Exception as e:
                logger.error(f"Callback error: {e}")

    @property
    def current_state(self) -> RobotState:
        """Get current state."""
        return self._current_state

    @property
    def state_name(self) -> str:
        """Get current state name."""
        return self._current_state.name

    def start(self) -> None:
        """Start the state machine."""
        logger.info("Starting state machine")
        self._transition_to(RobotState.IDLE)

    def stop(self) -> None:
        """Stop the robot (emergency stop)."""
        self._transition_to(RobotState.STOPPED)

    def reset(self) -> None:
        """Reset to idle state."""
        self._transition_to(RobotState.IDLE)

    def set_error(self, message: str) -> None:
        """Set error state with message."""
        self.context.error_message = message
        self.context.error_count += 1
        self._transition_to(RobotState.ERROR)

    def update_context(self, **kwargs) -> None:
        """Update context with new sensor data."""
        old_target = self.context.target_detected

        for key, value in kwargs.items():
            if hasattr(self.context, key):
                setattr(self.context, key, value)

        # Emit events
        if not old_target and self.context.target_detected:
            self._emit('on_target_detected', self.context)
        elif old_target and not self.context.target_detected:
            self._emit('on_target_lost', self.context)

        if self.context.obstacle_detected:
            self._emit('on_obstacle_detected', self.context)

        self.context.last_update_time = time.time()

    def update(self) -> None:
        """Update state machine - call this every control loop iteration."""
        handler = self._handlers.get(self._current_state)
        if handler is None:
            logger.error(f"No handler for state {self._current_state}")
            return

        next_state = handler.update(self.context)
        if next_state is not None and next_state != self._current_state:
            self._transition_to(next_state)

    def _transition_to(self, new_state: RobotState) -> None:
        """Transition to a new state."""
        if new_state == self._current_state:
            return

        old_state = self._current_state

        # Exit current state
        old_handler = self._handlers.get(old_state)
        if old_handler:
            old_handler.exit(self.context)

        # Record transition
        transition = StateTransition(
            from_state=old_state,
            to_state=new_state,
            condition=f"{old_state.name} -> {new_state.name}"
        )
        self._transition_history.append(transition)

        # Keep history bounded
        if len(self._transition_history) > 100:
            self._transition_history = self._transition_history[-50:]

        # Update state
        self._current_state = new_state
        self.context.state_enter_time = time.time()

        # Enter new state
        new_handler = self._handlers.get(new_state)
        if new_handler:
            new_handler.enter(self.context)

        # Emit event
        self._emit('on_state_change', old_state, new_state)
        logger.info(f"State transition: {old_state.name} -> {new_state.name}")

    def get_velocity(self) -> tuple:
        """Get current velocity command (linear, angular)."""
        handler = self._handlers.get(self._current_state)
        if handler:
            return handler.get_velocity(self.context)
        return (0.0, 0.0)

    def get_transition_history(self) -> List[StateTransition]:
        """Get state transition history."""
        return self._transition_history.copy()
