from rclpy.node import Node
from typing import Union
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Quaternion, Point, Pose

def createHeader(node: Node, frameId: str = "world") -> Header: 
    """Create a message Header

    Args:
        node (Node): The node subscribed to the custom messages to convert
        frameId (str, optional): The frameId type ["world", "base_link", "map"]. Defaults to "world".

    Returns:
        Header: Header for the message
    """
    
    toHeader = Header()
    toHeader.frame_id = frameId
    toHeader.stamp = node.get_clock().now().to_msg()
    return toHeader


def convertPosition(fromPosition: Union[list[float], tuple[float], dict[str, float], any] = None) -> Point:
    """Convert non-Vector3 types to Vector3 type. Add any other convertion here.

    Args:
        fromPosition (list[float] | dict[str, float] | any): The data type to convert to Vector3

    Raises:
        IndexError: Position must contain 3 elements: 'x', 'y' and 'z' respective values.
        IndexError: Position must contain, at least, the 3 keys 'x', 'y', and 'z' and respective values.

    Returns:
        toPosition: The submessage to populate. Defaults to a zeroed Vector3.
    """
    
    toPosition = Point()
    # List or tuple convert
    if isinstance(fromPosition, (list, tuple)):
        if len(fromPosition) != 3:
            raise IndexError("Position must contain 3 elements: 'x', 'y', and 'z' respective values.")
        toPosition.x, toPosition.y, toPosition.z = float(fromPosition[0]), float(fromPosition[1]), float(fromPosition[2])
        return toPosition
    
    # Dict convert
    if isinstance(fromPosition, dict):
        if any(key not in fromPosition for key in ['x', 'y', 'z']):
            raise IndexError("Position must contain, at least, the 3 keys 'x', 'y', and 'z' and respective values.")
        toPosition.x, toPosition.y, toPosition.z = float(fromPosition['x']), float(fromPosition['y']), float(fromPosition['z'])
        return toPosition
    
    # Default behavior
    toPosition.x = toPosition.y = toPosition.z = 0.0
    return toPosition


def convertOrientation(fromOrientation: Union[list[float], tuple[float], dict[str, float], any] = None) -> Quaternion:
    """Convert non-Quaternion types to Quaternion type. Add any other convertion here.
    
    Args:
        fromOrientation (list[float] | dict[str, float] | any): The data type to convert to Quaternion

    Raises:
        IndexError: Orientation must contain 4 elements: 'x', 'y', 'z' and 'w' respective values.
        IndexError: Orientation must contain, at least, the 4 keys 'x', 'y', 'z' and 'w' and respective values.

    Returns:
        toOrientation: The submessage to populate. Defaults to a zeroed Quaternion.
    """
    
    toOrientation = Quaternion()
    # List or tuple convert
    if isinstance(fromOrientation, (list, tuple)):
        if len(fromOrientation) != 4:
            raise IndexError("Orientation must contain 4 elements: 'x', 'y', 'z' and 'w' respective values.")
        toOrientation.x, toOrientation.y, toOrientation.z, toOrientation.w = float(fromOrientation[0]), float(fromOrientation[1]), float(fromOrientation[2]), float(fromOrientation[3])
        return toOrientation
    
    # Dict convert
    if isinstance(fromOrientation, dict):
        if any(key not in fromOrientation for key in ['x', 'y', 'z', 'w']):
            raise IndexError("Orientation must contain, at least, the 4 keys 'x', 'y', 'z' and 'w' and respective values.")
        toOrientation.x, toOrientation.y, toOrientation.z, toOrientation.w = float(fromOrientation['x']), float(fromOrientation['y']), float(fromOrientation['z']), float(fromOrientation['w'])
        return toOrientation
    
    # Default behavior
    toOrientation.x = toOrientation.y = toOrientation.z = 0.0
    toOrientation.w = 1.0
    return toOrientation


def createPose(fromPosition: Union[list[float], tuple[float], dict[str, float], any] = None,
               fromOrientation: Union[list[float], tuple[float], dict[str, float], any] = None) -> Pose:
    """Creates a Pose object from the postision and orientation converters.

    Args:
        fromPosition (Union[list[float], tuple[float], dict[str, float], any], optional): Position of Pose object. Defaults to None.
        fromOrientation (Union[list[float], tuple[float], dict[str, float], any], optional): Orientation of Pose object. Defaults to None.

    Returns:
        Pose: The submessage to populate. Defaults to a zeroed Position and Quaternion.
    """
    return Pose(position=convertPosition(fromPosition), orientation=convertOrientation(fromOrientation))


def convertScale(fromScale: Union[list[float], tuple[float], dict[str, float], any] = None) -> Vector3:
    """Convert non-Vector3 types to Vector3 type. Add any other convertion type here.

    Args:
        fromScale (list[float] | tuple[float] | dict[str, float] | any): The data type to convert to Vector3

    Raises:
        IndexError: Scale must contain 3 elements: 'x', 'y' and 'z' respective values.
        IndexError: Scale must contain, at least, the 3 keys 'x', 'y', and 'z' and respective values.
        
    Returns:
        toScale: The submessage to populate. Defaults to a ones Vector3.
    """
    toScale = Vector3()
    # List or tuple convert
    if isinstance(fromScale, (list, tuple)):
        if len(fromScale) != 3:
            raise ValueError("Scale must contain 3 elements: 'x', 'y' and 'z' respective values.")
        toScale.x, toScale.y, toScale.z = float(fromScale[0]), float(fromScale[1]), float(fromScale[2])
        return toScale
    
    # Dict convert
    if isinstance(fromScale, dict):
        if any(key not in fromScale for key in ['x', 'y', 'z']):
            raise IndexError("Scale must contain, at least, the 3 keys 'x', 'y', and 'z' and respective values.")
        toScale.x, toScale.y, toScale.z = float(fromScale['x']), float(fromScale['y']), float(fromScale['z'])
        return toScale
    
    # Default behavior
    toScale.x = toScale.y = toScale.z = 1.0
    return toScale


def getColor(fromColor: Union[list[float], tuple[float], dict[str, float], any] = None) -> ColorRGBA:
    """Get a ColorRGBA object from 4 (rgba) values.

    Args:
        fromColor (list[float] | tuple[float] | dict[str, float] | any): The 4 values representing the color in rgba format. Needs normalisation.

    Raises:
        IndexError: color must contain 4 elements: 'r', 'g', 'b' and 'a' respective values.
        IndexError: color must contain, at least, the keys 'r', 'g', 'b' and 'a' and respective values.
        ValueError: color must have values between 0.0 and 1.0 (Normalized values).

    Returns:
        toColor: The submessage to populate. Defaults to WHITE color code.
    """
    
    toColor = ColorRGBA()
    # List or tuple convert
    if isinstance(fromColor, (list, tuple)):
        if len(fromColor) != 4:
            raise IndexError("color must contain 4 elements: 'r', 'g', 'b' and 'a' respective values.")
        for value in fromColor:
            if not (0.0 <= value <= 1.0):
                raise ValueError("color must have values between 0.0 and 1.0 (Normalized values).")
        toColor.r, toColor.g, toColor.b, toColor.a = float(fromColor[0]), float(fromColor[1]), float(fromColor[2]), float(fromColor[3])
        return toColor
    
    # Dict convert
    if isinstance(fromColor, dict):
        if any(key not in fromColor for key in ['r', 'g', 'b', 'a']):
            raise IndexError("color must contain, at least, the keys 'r', 'g', 'b' and 'a' and respective values.")
        for key, value in fromColor.items():
            if key in ['r', 'g', 'b', 'a'] and not (0.0 <= value <= 1.0):
                raise ValueError("color must have values between 0.0 and 1.0 (Normalized values).")
        toColor.r, toColor.g, toColor.b, toColor.a = float(fromColor['r']), float(fromColor['g']), float(fromColor['b']), float(fromColor['a'])
        return toColor
    
    # Default behavior
    toColor.r = toColor.g = toColor.b = toColor.a = 1.0
    return toColor
