#!/usr/bin/env python3

import rclpy


class GetCones:
    
    def __init__(self, setMarkers) -> None:
        self.markers = setMarkers

    
    def get(self, msg):
        self.markers.set(msg.cones)


    @staticmethod
    def coneMsgPrint(msg):
        print("Received marker:")
        print("Frame ID:", msg.header.frame_id)
        i=0
        for cone in msg.cones:
            print("Cone n"+str(i))
            print("position: [\n\tx:" + str(cone.position.x) + ",\n\ty:" + str(cone.position.y) + ",\n\tz:" + str(cone.position.z) + "\n]")
            print("cone type: " + str(cone.class_type.data))
            i+=1


    @staticmethod
    def debug(self, args=None):
        # Debug imports
        from lart_msgs.msg import ConeArray # type: ignore
        from .const import Topics
        
        rclpy.init(args=args)

        node = rclpy.create_node(Topics.SUBSCRIBER_NODE_NAME)

        coneArray_sub = node.create_subscription(
            ConeArray,
            Topics.SUBSCRIBED_TOPIC,
            self.coneMsgPrint,
            10
        )

        node.get_logger().info('Subscriber node ready.')

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

        node.destroy_node()
        rclpy.shutdown()
