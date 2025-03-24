#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
# import tf_transformations

class OdomToTfNode(Node):
    """
    ROS2 Node that subscribes to Odometry messages and publishes corresponding tf transforms.
    """
    
    def __init__(self):
        super().__init__('odom_to_tf_node')
        
        # Declare parameters
        self.declare_parameter('odom_topic', 'mavros/local_position/odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'map')
        self.declare_parameter('publish_rate_hz', 50.0)
        
        # Get parameters
        self.odom_topic = self.get_parameter('odom_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # Create a transformer broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        qos = rclpy.qos.qos_profile_sensor_data
        # Subscribe to the odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos)
        
        self.get_logger().info(f"Subscribed to: {self.odom_topic}")
        self.get_logger().info(f"Publishing transform from {self.odom_frame} to {self.base_frame}")
        
    def odom_callback(self, msg):
        """
        Callback function for the odometry subscription.
        Converts odometry message to a transform and broadcasts it.
        
        Args:
            msg (Odometry): The received odometry message
        """
        # Create transform message
        transform = TransformStamped()
        
        # Set header information
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        
        # Set translation from odometry message
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation from odometry message
        transform.transform.rotation.x = msg.pose.pose.orientation.x
        transform.transform.rotation.y = msg.pose.pose.orientation.y
        transform.transform.rotation.z = msg.pose.pose.orientation.z
        transform.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
        
def main(args=None):
    rclpy.init(args=args)
    
    node = OdomToTfNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()