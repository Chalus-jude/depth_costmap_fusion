#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
import tf2_ros


class TFCloudTest(Node):

    def __init__(self):
        super().__init__('tf_cloud_test')

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=Duration(seconds=30.0)
        )

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # Subscribe to the generated point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/filtered_points',
            self.cloud_callback,
            10
        )

        self.count = 0

        self.get_logger().info(
            'TF cloud test started.'
        )

        self.get_logger().info(
            'Checking: camera_depth_optical_frame -> odom'
        )

        self.get_logger().info(
            'Waiting for /filtered_points...'
        )

    def cloud_callback(self, msg):

        self.count += 1

        # --------------------------------------------------
        # Cloud timestamp
        # --------------------------------------------------

        stamp = msg.header.stamp

        cloud_time = (
            stamp.sec +
            stamp.nanosec * 1e-9
        )

        cloud_frame = msg.header.frame_id

        self.get_logger().info(
            f'\nCloud #{self.count}'
        )

        self.get_logger().info(
            f'  Time  : {cloud_time:.9f}'
        )

        self.get_logger().info(
            f'  Frame : {cloud_frame}'
        )

        self.get_logger().info(
            f'  Size  : {msg.width} x {msg.height}'
        )

        # --------------------------------------------------
        # Check whether TF exists at EXACT cloud timestamp
        # --------------------------------------------------

        try:

            transform = self.tf_buffer.lookup_transform(
                'odom',
                cloud_frame,
                stamp,
                timeout=Duration(seconds=0.2)
            )

            # --------------------------------------------------
            # TF timestamp
            # --------------------------------------------------

            tf_stamp = transform.header.stamp

            tf_time = (
                tf_stamp.sec +
                tf_stamp.nanosec * 1e-9
            )

            # Difference between cloud timestamp
            # and the TF timestamp returned
            age = cloud_time - tf_time

            self.get_logger().info(
                '  TF RESULT: OK'
            )

            self.get_logger().info(
                f'  TF time : {tf_time:.9f}'
            )

            self.get_logger().info(
                f'  Cloud-TF: {age:+.6f} sec'
            )

            # --------------------------------------------------
            # Print transform
            # --------------------------------------------------

            t = transform.transform.translation
            q = transform.transform.rotation

            self.get_logger().info(
                f'  Translation: '
                f'x={t.x:.4f}, '
                f'y={t.y:.4f}, '
                f'z={t.z:.4f}'
            )

            self.get_logger().info(
                f'  Rotation: '
                f'x={q.x:.4f}, '
                f'y={q.y:.4f}, '
                f'z={q.z:.4f}, '
                f'w={q.w:.4f}'
            )

        except Exception as e:

            self.get_logger().error(
                '  TF RESULT: FAILED'
            )

            self.get_logger().error(
                f'  Cloud timestamp: '
                f'{cloud_time:.9f}'
            )

            self.get_logger().error(
                f'  Error: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = TFCloudTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
