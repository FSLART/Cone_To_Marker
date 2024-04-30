import rclpy
from visualization_msgs.msg import MarkerArray
from lart_msgs.msg import ConeArray # type: ignore
from .const import Topics
from .get_cones import GetCones
from .set_markers import SetMarkers

def main(args=None):
    
    rclpy.init(args=args)
    
    getter_node = rclpy.create_node(Topics.SUBSCRIBER_NODE_NAME)
    sender_node = rclpy.create_node(Topics.PUBLISHER_NODE_NAME)
    
    publisher = sender_node.create_publisher(MarkerArray, Topics.PUBLISHED_TOPIC, 2)
    subscriber = getter_node.create_subscription(
        ConeArray,
        Topics.SUBSCRIBED_TOPIC,
        GetCones(SetMarkers(publisher, sender_node)).get,
        8
    )
    
    
    getter_node.get_logger().info(" Subscriber node[" + Topics.SUBSCRIBER_NODE_NAME + "] ready.")
    sender_node.get_logger().info(" Publisher node[" + Topics.PUBLISHER_NODE_NAME + "] ready.")
    
    try:
        while rclpy.ok():
            rclpy.spin(getter_node)
            rclpy.spin(sender_node)
    except KeyboardInterrupt:
        pass

    getter_node.destroy_node()
    sender_node.destroy_node()
    rclpy.shutdown()
