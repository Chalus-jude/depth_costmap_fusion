#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import PointCloud2


class TFCheck(Node):

    def __init__(self):
        super().__init__('tf_check')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.sub = self.create_subscription(
            PointCloud2,
            '/filtered_points',
            self.callback,
            10
        )

        self.count = 0

    def callback(self, msg):

        self.count += 1

        stamp = msg.header.stamp
        cloud_time = stamp.sec + stamp.nanosec * 1e-9

        self.get_logger().info(
            f'========== CLOUD #{self.count} =========='
        )

        self.get_logger().info(
            f'Cloud time : {cloud_time:.9f}'
        )

        self.get_logger().info(
            f'Cloud frame: {msg.header.frame_id}'
        )

        # Check whether TF exists at the EXACT cloud timestamp
        try:

            tf = self.buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                Time.from_msg(stamp)
            )

            t = tf.transform.translation
            q = tf.transform.rotation

            tf_time = (
                tf.header.stamp.sec +
                tf.header.stamp.nanosec * 1e-9
            )

            self.get_logger().info(
                'RESULT: TF OK'
            )

            self.get_logger().info(
                f'TF time   : {tf_time:.9f}'
            )

            self.get_logger().info(
                f'TF age    : {cloud_time - tf_time:+.6f} s'
            )

            self.get_logger().info(
                f'Transform : '
                f'x={t.x:.3f}, '
                f'y={t.y:.3f}, '
                f'z={t.z:.3f}'
            )

            self.get_logger().info(
                f'Rotation  : '
                f'x={q.x:.3f}, '
                f'y={q.y:.3f}, '
                f'z={q.z:.3f}, '
                f'w={q.w:.3f}'
            )

        except Exception as e:

            self.get_logger().error(
                'RESULT: TF FAILED'
            )

            self.get_logger().error(
                str(e)
            )


def main(args=None):

    rclpy.init(args=args)

    node = TFCheck()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
