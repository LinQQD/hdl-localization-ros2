#!/usr/bin/env python3
# Copyright 2026
# Convert Livox MID360 CustomMsg (/livox/lidar) to PointCloud2 (/lidar/pointcloud2)
# for hdl_localization (expects sensor_msgs/PointCloud2 with x,y,z,intensity).

import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class LivoxToPointCloud2(Node):
    """Subscribe Livox CustomMsg and publish standard PointCloud2 (PointXYZI layout)."""

    def __init__(self) -> None:
        super().__init__('livox_to_pointcloud2')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/lidar/pointcloud2')
        self.declare_parameter('output_frame_id', '')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._output_frame_id = self.get_parameter('output_frame_id').value

        self._fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self._pub = self.create_publisher(PointCloud2, output_topic, 10)
        self._sub = self.create_subscription(CustomMsg, input_topic, self._callback, 10)

        self.get_logger().info(
            f'Converting {input_topic} (livox_ros_driver2/CustomMsg) '
            f'-> {output_topic} (sensor_msgs/PointCloud2)')

    def _callback(self, msg: CustomMsg) -> None:
        if msg.point_num == 0 or not msg.points:
            return

        header = msg.header
        if self._output_frame_id:
            header.frame_id = self._output_frame_id

        count = min(int(msg.point_num), len(msg.points))
        points = [
            (
                float(msg.points[i].x),
                float(msg.points[i].y),
                float(msg.points[i].z),
                float(msg.points[i].reflectivity),
            )
            for i in range(count)
        ]

        cloud = point_cloud2.create_cloud(header, self._fields, points)
        self._pub.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxToPointCloud2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
