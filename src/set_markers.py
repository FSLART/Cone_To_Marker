#!/usr/bin/env python3

import rclpy
from rclpy.publisher import Publisher
from visualization_msgs.msg import MarkerArray, Marker


class SetMarkers:
    
    def __init__(self, publisher: Publisher, node) -> None:
        from .convert import Converter
        self.publisher = publisher
        self.node = node
        self.converter = Converter()
        
    def set(self, cones: list):
        from .utils import createHeader
        
        markerArray = MarkerArray()
        
        header = createHeader(self.node)
        for cone in cones:
            markerArray.markers.append(self.converter.convertConeToMarker(cone, header))
        
        self.converter.id=0
        self.publisher.publish(markerArray)


    @staticmethod
    def debug(args=None):
        # Debug only imports
        from .utils import createHeader, convertScale, createPose
        from .const import Colors, Topics
        
        rclpy.init(args=args)

        node = rclpy.create_node(Topics.PUBLISHER_NODE_NAME)

        marker_pub = node.create_publisher(Marker, Topics.PUBLISHED_TOPIC, 2)        
        
        if rclpy.ok():
            try:
                marker_pub.publish(
                    Marker(
                        header=createHeader(node),
                        type=Marker.CYLINDER,
                        id=0,
                        color=Colors.CYAN,
                        scale=convertScale(),
                        pose=createPose((1,1,0))
                    )
                )
                node.get_logger().info('Publishing ' + Topics.PUBLISHER_NODE_NAME)
                rclpy.spin_once(node)
                node.get_clock().sleep(1.0)
            except KeyboardInterrupt:
                pass
            
        node.destroy_node()
        rclpy.shutdown()
