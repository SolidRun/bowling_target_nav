"""
Event System
============

Provides a publish-subscribe event system for loose coupling between components.
"""

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional
from enum import Enum, auto

logger = logging.getLogger(__name__)


class EventType(Enum):
    """Standard event types."""
    # Detection events
    TARGET_DETECTED = auto()
    TARGET_LOST = auto()
    DETECTION_UPDATE = auto()

    # Navigation events
    NAVIGATION_STARTED = auto()
    NAVIGATION_COMPLETED = auto()
    OBSTACLE_DETECTED = auto()
    OBSTACLE_CLEARED = auto()

    # State events
    STATE_CHANGED = auto()
    ERROR_OCCURRED = auto()

    # Hardware events
    ARDUINO_CONNECTED = auto()
    ARDUINO_DISCONNECTED = auto()
    LIDAR_CONNECTED = auto()
    LIDAR_DISCONNECTED = auto()
    CAMERA_CONNECTED = auto()
    CAMERA_DISCONNECTED = auto()

    # System events
    SYSTEM_STARTED = auto()
    SYSTEM_STOPPED = auto()
    CONFIG_CHANGED = auto()


@dataclass
class Event:
    """Event data container."""
    event_type: EventType
    data: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)
    source: str = ""

    def __repr__(self) -> str:
        return f"Event({self.event_type.name}, source={self.source})"


class EventBus:
    """
    Centralized event bus for component communication.

    Uses publish-subscribe pattern for loose coupling.

    Usage:
        bus = EventBus()

        # Subscribe to events
        def on_target_detected(event):
            print(f"Target at {event.data['distance']}m")

        bus.subscribe(EventType.TARGET_DETECTED, on_target_detected)

        # Publish events
        bus.publish(Event(
            event_type=EventType.TARGET_DETECTED,
            data={'distance': 1.5, 'confidence': 0.95},
            source='detector'
        ))
    """

    _instance: Optional['EventBus'] = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self._subscribers: Dict[EventType, List[Callable[[Event], None]]] = {}
        self._global_subscribers: List[Callable[[Event], None]] = []
        self._event_history: List[Event] = []
        self._lock = threading.Lock()
        self._max_history = 100
        self._initialized = True

    @classmethod
    def reset(cls):
        """Reset singleton instance (for testing)."""
        cls._instance = None

    def subscribe(
        self,
        event_type: EventType,
        callback: Callable[[Event], None]
    ) -> None:
        """Subscribe to a specific event type."""
        with self._lock:
            if event_type not in self._subscribers:
                self._subscribers[event_type] = []
            if callback not in self._subscribers[event_type]:
                self._subscribers[event_type].append(callback)
                logger.debug(f"Subscribed to {event_type.name}")

    def subscribe_all(self, callback: Callable[[Event], None]) -> None:
        """Subscribe to all events."""
        with self._lock:
            if callback not in self._global_subscribers:
                self._global_subscribers.append(callback)
                logger.debug("Subscribed to all events")

    def unsubscribe(
        self,
        event_type: EventType,
        callback: Callable[[Event], None]
    ) -> None:
        """Unsubscribe from a specific event type."""
        with self._lock:
            if event_type in self._subscribers:
                if callback in self._subscribers[event_type]:
                    self._subscribers[event_type].remove(callback)

    def unsubscribe_all(self, callback: Callable[[Event], None]) -> None:
        """Unsubscribe from all events."""
        with self._lock:
            if callback in self._global_subscribers:
                self._global_subscribers.remove(callback)

    def publish(self, event: Event) -> None:
        """Publish an event to all subscribers."""
        with self._lock:
            # Store in history
            self._event_history.append(event)
            if len(self._event_history) > self._max_history:
                self._event_history = self._event_history[-self._max_history:]

            # Get subscribers
            specific = list(self._subscribers.get(event.event_type, []))
            global_subs = list(self._global_subscribers)

        # Call specific subscribers
        for callback in specific:
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Event callback error: {e}")

        # Call global subscribers
        for callback in global_subs:
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Global event callback error: {e}")

    def publish_async(self, event: Event) -> None:
        """Publish event asynchronously in a separate thread."""
        thread = threading.Thread(target=self.publish, args=(event,))
        thread.daemon = True
        thread.start()

    def get_history(
        self,
        event_type: Optional[EventType] = None,
        limit: int = 50
    ) -> List[Event]:
        """Get event history, optionally filtered by type."""
        with self._lock:
            if event_type is None:
                return self._event_history[-limit:]
            return [e for e in self._event_history if e.event_type == event_type][-limit:]

    def clear_history(self) -> None:
        """Clear event history."""
        with self._lock:
            self._event_history.clear()


# Global event bus accessor
def get_event_bus() -> EventBus:
    """Get the global event bus instance."""
    return EventBus()
