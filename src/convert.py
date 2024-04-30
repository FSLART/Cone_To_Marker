from visualization_msgs.msg import Marker
from .utils import *
from .const import *

class Converter:
    def __init__(self) -> None:
        self.id = 0
            
    def convertConeToMarker(self, cone, header) -> Marker:
        self.id+=1
        return Marker(
            header=header,
            type=Marker.CYLINDER,
            id=self.id,
            color=CLASS_COLORS.get(cone.class_type.data),
            scale=CLASS_SCALE.get(cone.class_type.data),
            pose=createPose((
                cone.position.x,
                cone.position.y,
                cone.position.z
                ))
        )