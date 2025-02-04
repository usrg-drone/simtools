import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

class LidarWallTimeRepublisher(Node):
    def __init__(self):
        super().__init__('lidar_wall_time_republisher')
        
        # Subscribe to the original lidar points topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',  # Change this if needed
            self.lidar_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Publisher for republished lidar points with wall time
        self.publisher = self.create_publisher(PointCloud2, '/lidar/points_wall_time', 10)
        
    def lidar_callback(self, msg: PointCloud2):
        # Update the timestamp to the current wall time
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the modified message
        self.publisher.publish(msg)
        self.get_logger().info('Republished lidar points with wall time')


def main(args=None):
    rclpy.init(args=args)
    node = LidarWallTimeRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
