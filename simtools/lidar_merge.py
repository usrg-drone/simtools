#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import time
from std_msgs.msg import Header
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('pointcloud_merger')
        
        # Declare ROS2 parameters with descriptions
        self.declare_parameter(
            'input_topic', 
            '/pointcloud',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Input pointcloud topic to subscribe to'
            )
        )
        
        self.declare_parameter(
            'duration', 
            0.1,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Duration to collect points (seconds)'
            )
        )
        
        self.declare_parameter(
            'output_topic', 
            '/merged_pointcloud',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Output topic for the merged pointcloud'
            )
        )
        
        self.declare_parameter(
            'output_frame', 
            '',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Frame ID for the output point cloud'
            )
        )
        
        # Get parameter values
        self.topic_name = self.get_parameter('input_topic').get_parameter_value().string_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self.input_frame = ""

        # Register parameter callback to handle parameter changes at runtime
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic_name,
            self.listener_callback,
            10)
            
        # Publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10)
            
        # Storage for point clouds
        self.all_points = []
        self.fields = None
        self.field_names = None
        
        # Timer for tracking duration
        self.start_time = None
        self.is_collecting = False
        
        self.get_logger().info(f"Initialized PointCloud concatenator with parameters:")
        self.get_logger().info(f"  - input_topic: {self.topic_name}")
        self.get_logger().info(f"  - duration: {self.duration} seconds")
        self.get_logger().info(f"  - output_topic: {self.output_topic}")
        self.get_logger().info(f"  - output_frame: {self.output_frame}")
    
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'input_topic' and param.type_ == ParameterType.PARAMETER_STRING:
                # Only update subscriber if the topic changed
                if param.value.string_value != self.topic_name:
                    self.topic_name = param.value.string_value
                    # Recreate subscriber with new topic
                    self.destroy_subscription(self.subscription)
                    self.subscription = self.create_subscription(
                        PointCloud2,
                        self.topic_name,
                        self.listener_callback,
                        10)
                    self.get_logger().info(f"Subscription updated to topic: {self.topic_name}")
            
            elif param.name == 'output_topic' and param.type_ == ParameterType.PARAMETER_STRING:
                # Only update publisher if the topic changed
                if param.value.string_value != self.output_topic:
                    self.output_topic = param.value.string_value
                    # Recreate publisher with new topic
                    self.destroy_publisher(self.publisher)
                    self.publisher = self.create_publisher(
                        PointCloud2,
                        self.output_topic,
                        10)
                    self.get_logger().info(f"Publisher updated to topic: {self.output_topic}")
            
            elif param.name == 'duration' and param.type_ == ParameterType.PARAMETER_DOUBLE:
                self.duration = param.value.double_value
                self.get_logger().info(f"Duration updated to: {self.duration} seconds")
            
            elif param.name == 'output_frame' and param.type_ == ParameterType.PARAMETER_STRING:
                self.output_frame = param.value.string_value
                self.get_logger().info(f"Output frame updated to: {self.output_frame}")
        
        return True
        
    def listener_callback(self, msg):
        self.input_frame = msg.header.frame_id
        if not self.is_collecting:
            self.is_collecting = True
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            self.fields = msg.fields
            self.field_names = [field.name for field in msg.fields]
            self.get_logger().info(f"Started collecting points. Fields: {self.field_names}")
        
        # Check if we've reached the desired duration
        if (msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9) - self.start_time > self.duration:
            if not self.all_points:
                self.get_logger().warning("Collection time elapsed, but no points were collected.")
                return
                
            self.get_logger().info("Collection time elapsed. Publishing merged point cloud...")
            self.publish_merged_cloud()
            return
        
        # Extract points from the message and append them to our list
        points = list(pc2.read_points(msg, field_names=self.field_names, skip_nans=True))
        self.all_points.extend(points)
        # self.get_logger().info(f"Received message with {len(points)} points. Total points so far: {len(self.all_points)}")
    
    def publish_merged_cloud(self):
        if not self.all_points:
            self.get_logger().error("No points to publish!")
            return
            
        # Create a header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.output_frame if len(self.output_frame) > 0 else self.input_frame
        
        # Create a PointCloud2 message
        cloud_msg = pc2.create_cloud(header, self.fields, self.all_points)
        
        # Publish the merged cloud
        self.publisher.publish(cloud_msg)
        self.get_logger().info(f"Published merged point cloud with {len(self.all_points)} points.")
        
        # Reset collection
        self.all_points = []
        self.is_collecting = False

def main(args=None):
    rclpy.init(args=args)
    
    node = PointCloudMerger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()