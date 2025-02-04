import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseRelayNode(Node):
    def __init__(self):
        super().__init__('pose_relay_node')
        
        # Subscriber to /glim_ros/pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/glim_ros/pose',
            self.pose_callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning

        # Publisher to /mavros/vision_pose/pose
        self.publisher = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10  # QoS profile depth
        )

    def pose_callback(self, msg):
        self.get_logger().info(f'Received pose from /glim_ros/pose, republishing to /mavros/vision_pose/pose')
        
        # Republish the received pose message
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
