#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from livox_ros_driver2.msg import CustomMsg


class LivoxCustomToPointCloud2(Node):
    def __init__(self):
        super().__init__('livox_custom_to_pointcloud2')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/pointcloud2')
        self.declare_parameter('frame_id', 'livox_frame')
        self.declare_parameter('queue_size', 100)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value

        qos = QoSProfile(depth=self.queue_size)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, qos)
        self.subscriber = self.create_subscription(CustomMsg, self.input_topic, self.callback, qos)

        self.get_logger().info('livox custom -> pointcloud2 bridge started')
        self.get_logger().info(f'subscribing: {self.input_topic}')
        self.get_logger().info(f'publishing: {self.output_topic}')
        self.get_logger().info(f'frame_id: {self.frame_id}')

    def callback(self, msg: CustomMsg):
        header = msg.header
        header.frame_id = self.frame_id or msg.header.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        points = [
            (point.x, point.y, point.z, float(point.reflectivity))
            for point in msg.points
        ]

        cloud_msg = point_cloud2.create_cloud(header, fields, points)
        self.publisher.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxCustomToPointCloud2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
