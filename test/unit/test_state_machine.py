"""
State Machine Tests
===================

Unit tests for robot state machine.
"""

import os
import pytest
import time

import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from bowling_target_nav.core.state_machine import (
    RobotState, RobotStateMachine, StateContext,
    IdleStateHandler, DetectingStateHandler, NavigatingStateHandler,
    SearchingStateHandler, AvoidingStateHandler, StoppedStateHandler
)


class TestRobotState:
    """Test RobotState enum."""

    def test_state_values(self):
        """Test all states exist."""
        assert RobotState.IDLE
        assert RobotState.DETECTING
        assert RobotState.NAVIGATING
        assert RobotState.SEARCHING
        assert RobotState.AVOIDING
        assert RobotState.STOPPED
        assert RobotState.ERROR


class TestStateContext:
    """Test StateContext data class."""

    def test_default_values(self):
        """Test default context values."""
        ctx = StateContext()

        assert ctx.target_detected == False
        assert ctx.target_distance == float('inf')
        assert ctx.obstacle_detected == False
        assert ctx.current_linear_vel == 0.0

    def test_reset_target(self):
        """Test resetting target information."""
        ctx = StateContext()
        ctx.target_detected = True
        ctx.target_x = 0.5
        ctx.target_distance = 1.0

        ctx.reset_target()

        assert ctx.target_detected == False
        assert ctx.target_x == 0.0
        assert ctx.target_distance == float('inf')

    def test_time_in_state(self):
        """Test time tracking in state."""
        ctx = StateContext()
        ctx.state_enter_time = time.time() - 5.0

        elapsed = ctx.time_in_state()
        assert 4.9 < elapsed < 5.1

    def test_time_since_target(self):
        """Test time since last target detection."""
        ctx = StateContext()
        ctx.last_target_time = time.time() - 3.0

        elapsed = ctx.time_since_target()
        assert 2.9 < elapsed < 3.1

    def test_time_since_target_never_seen(self):
        """Test time since target when never seen."""
        ctx = StateContext()
        assert ctx.time_since_target() == float('inf')


class TestRobotStateMachine:
    """Test RobotStateMachine."""

    def test_initial_state(self):
        """Test state machine starts in IDLE."""
        sm = RobotStateMachine()
        sm.start()

        assert sm.current_state == RobotState.IDLE

    def test_state_name_property(self):
        """Test state name property."""
        sm = RobotStateMachine()
        sm.start()

        assert sm.state_name == "IDLE"

    def test_transition_to_detecting(self):
        """Test transition from IDLE to DETECTING."""
        sm = RobotStateMachine()
        sm.start()
        sm.update()

        assert sm.current_state == RobotState.DETECTING

    def test_target_detected_transition(self):
        """Test transition to NAVIGATING when target detected."""
        sm = RobotStateMachine()
        sm.start()
        sm.update()  # IDLE -> DETECTING

        sm.update_context(
            target_detected=True,
            target_x=0.0,
            target_distance=1.5,
            target_confidence=0.9
        )
        sm.update()

        assert sm.current_state == RobotState.NAVIGATING

    def test_obstacle_avoidance_transition(self):
        """Test transition to AVOIDING when obstacle detected."""
        sm = RobotStateMachine()
        sm.start()
        sm.update()  # IDLE -> DETECTING

        # Detect target
        sm.update_context(target_detected=True, target_distance=2.0)
        sm.update()  # -> NAVIGATING

        # Detect obstacle
        sm.update_context(obstacle_detected=True, obstacle_distance=0.3)
        sm.update()

        assert sm.current_state == RobotState.AVOIDING

    def test_emergency_stop(self):
        """Test emergency stop."""
        sm = RobotStateMachine()
        sm.start()
        sm.update()

        sm.stop()

        assert sm.current_state == RobotState.STOPPED

    def test_reset_to_idle(self):
        """Test reset returns to IDLE."""
        sm = RobotStateMachine()
        sm.start()
        sm.update()
        sm.stop()

        sm.reset()

        assert sm.current_state == RobotState.IDLE

    def test_error_state(self):
        """Test error state transition."""
        sm = RobotStateMachine()
        sm.start()

        sm.set_error("Test error")

        assert sm.current_state == RobotState.ERROR
        assert sm.context.error_message == "Test error"
        assert sm.context.error_count == 1

    def test_velocity_output(self):
        """Test velocity output from states."""
        sm = RobotStateMachine()
        sm.start()

        # IDLE state should output zero velocity
        linear, angular = sm.get_velocity()
        assert linear == 0.0
        assert angular == 0.0

    def test_navigation_velocity(self):
        """Test velocity during navigation."""
        sm = RobotStateMachine()
        sm.start()
        sm.update()  # -> DETECTING

        sm.update_context(
            target_detected=True,
            target_x=0.2,  # Target slightly to the right
            target_distance=1.5
        )
        sm.update()  # -> NAVIGATING

        linear, angular = sm.get_velocity()

        # Should have forward motion and turn towards target
        assert linear > 0
        assert angular < 0  # Turn right (negative)

    def test_state_change_callback(self):
        """Test state change callback."""
        sm = RobotStateMachine()
        transitions = []

        def on_change(old_state, new_state):
            transitions.append((old_state, new_state))

        sm.on('on_state_change', on_change)
        sm.start()
        sm.update()  # IDLE -> DETECTING

        assert len(transitions) >= 1
        assert transitions[-1] == (RobotState.IDLE, RobotState.DETECTING)

    def test_transition_history(self):
        """Test transition history tracking."""
        sm = RobotStateMachine()
        sm.start()
        sm.update()

        history = sm.get_transition_history()
        assert len(history) >= 1


class TestSearchingState:
    """Test searching behavior."""

    def test_search_rotation(self):
        """Test search state rotates robot."""
        handler = SearchingStateHandler(angular_speed=0.25)
        ctx = StateContext()
        ctx.state_enter_time = time.time()

        handler.enter(ctx)
        linear, angular = handler.get_velocity(ctx)

        assert linear == 0.0
        assert angular == 0.25

    def test_search_finds_target(self):
        """Test search exits when target found."""
        handler = SearchingStateHandler()
        ctx = StateContext()
        ctx.state_enter_time = time.time()

        handler.enter(ctx)

        # No target
        next_state = handler.update(ctx)
        assert next_state is None

        # Target found
        ctx.target_detected = True
        next_state = handler.update(ctx)
        assert next_state == RobotState.NAVIGATING

    def test_search_timeout(self):
        """Test search times out."""
        handler = SearchingStateHandler(search_timeout=1.0)
        ctx = StateContext()
        ctx.state_enter_time = time.time() - 2.0  # Started 2 seconds ago

        next_state = handler.update(ctx)
        assert next_state == RobotState.IDLE


class TestAvoidingState:
    """Test obstacle avoidance behavior."""

    def test_avoiding_turns_away(self):
        """Test avoiding state turns away from obstacle."""
        handler = AvoidingStateHandler()
        ctx = StateContext()
        ctx.state_enter_time = time.time()
        ctx.obstacle_angle = 30.0  # Obstacle to the left

        handler.enter(ctx)
        linear, angular = handler.get_velocity(ctx)

        # Should turn right (negative angular)
        assert angular < 0

    def test_avoiding_clears(self):
        """Test avoiding exits when obstacle cleared."""
        handler = AvoidingStateHandler(safe_distance=0.6)
        ctx = StateContext()
        ctx.state_enter_time = time.time()
        ctx.obstacle_detected = True
        ctx.obstacle_distance = 0.3

        handler.enter(ctx)
        next_state = handler.update(ctx)
        assert next_state is None  # Still avoiding

        # Obstacle cleared
        ctx.obstacle_detected = False
        ctx.obstacle_distance = 1.0
        next_state = handler.update(ctx)
        assert next_state == RobotState.DETECTING


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
