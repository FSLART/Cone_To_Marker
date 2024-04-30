from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3

class Topics():
    # Subscriber info
    SUBSCRIBER_NODE_NAME = 'cone_msg_finder'
    SUBSCRIBED_TOPIC = '/cone_array_topic'
    
    # Publisher info
    PUBLISHER_NODE_NAME = 'marker_convert'
    PUBLISHED_TOPIC = '/cone_markers'

class Colors():
    RED = ColorRGBA(
        r=1.0,
        g=0.0,
        b=0.0,
        a=1.0
        )
    GREEN = ColorRGBA(
        r=0.0,
        g=1.0,
        b=0.0,
        a=1.0
        )
    BLUE = ColorRGBA(
        r=0.0,
        g=0.0,
        b=1.0,
        a=1.0
        )
    CYAN = ColorRGBA(
        r=0.0,
        g=1.0,
        b=1.0,
        a=1.0
        )
    YELLOW = ColorRGBA(
        r=1.0,
        g=1.0,
        b=0.0,
        a=1.0
        )
    MAGENTA = ColorRGBA(
        r=1.0,
        g=0.0,
        b=1.0,
        a=1.0
        )
    ORANGE = ColorRGBA(
        r=1.0,
        g=0.6470588235294118,
        b=0.0,
        a=1.0
        )
    GRAY = ColorRGBA(
        r=0.5019607843137255,
        g=0.5019607843137255,
        b=0.5019607843137255,
        a=1.0
        )
    BLACK = ColorRGBA(
        r=0.0,
        g=0.0,
        b=0.0,
        a=1.0
        )
    WHITE = ColorRGBA(
        r=1.0,
        g=1.0,
        b=1.0,
        a=1.0
        )

class Size():
    SMALL = Vector3(
        x=0.22,
        y=0.22,
        z=0.35
        )
    LARGE = Vector3(
        x=0.28,
        y=0.28,
        z=0.50
        )

class ClassType():
    UNKNOWN = 0
    YELLOW = 1
    BLUE = 2
    ORANGE = 3
    LARGE_ORANGE = 4

    
CLASS_COLORS = {
    ClassType.UNKNOWN: Colors.BLACK,
    ClassType.YELLOW: Colors.YELLOW,
    ClassType.BLUE: Colors.BLUE,
    ClassType.ORANGE: Colors.ORANGE,
    ClassType.LARGE_ORANGE: Colors.ORANGE
}

CLASS_SCALE = {
    ClassType.UNKNOWN: Size.SMALL,
    ClassType.YELLOW: Size.SMALL,
    ClassType.BLUE: Size.SMALL,
    ClassType.ORANGE: Size.SMALL,
    ClassType.LARGE_ORANGE: Size.LARGE
}