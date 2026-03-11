"""Lightweight map metadata — replaces ROS nav_msgs.MapMetaData for GUI process.

The GUI process cannot import ROS messages (rclpy is not initialized there),
so this plain-Python class carries the same information.  It also provides
a nested ``origin.position.x/y`` API so existing code that was written
against the ROS MapMetaData interface continues to work unchanged.
"""


class MapInfo:
    """Plain Python replacement for ROS MapMetaData.

    Attributes match what map_panel.py needs:
        resolution, origin_x, origin_y, width, height

    Also exposes an ``origin`` attribute with a nested ``position`` object
    so existing code using ``info.origin.position.x`` works without changes.
    """

    __slots__ = ('resolution', 'width', 'height', 'origin_x', 'origin_y',
                 'origin')

    def __init__(self, resolution=0.05, origin_x=0.0, origin_y=0.0,
                 width=0, height=0):
        """Create a MapInfo instance.

        Args:
            resolution: Meters per pixel.
            origin_x: X coordinate of the map origin in world frame (meters).
            origin_y: Y coordinate of the map origin in world frame (meters).
            width: Map width in pixels.
            height: Map height in pixels.
        """
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height
        self.origin = _Origin(origin_x, origin_y)


class _Origin:
    """Nested object mimicking ``info.origin.position.x/y``."""
    __slots__ = ('position',)

    def __init__(self, x, y):
        self.position = _Position(x, y)


class _Position:
    """Holds x/y coordinates, mimicking ``geometry_msgs.msg.Point``."""
    __slots__ = ('x', 'y')

    def __init__(self, x, y):
        self.x = x
        self.y = y
