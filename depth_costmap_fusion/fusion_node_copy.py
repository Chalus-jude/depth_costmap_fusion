#!/usr/bin/env python3


import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge


class DepthFusionNode(Node):
    def __init__(self):
        super().__init__('depth_fusion_node')

        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('confidence_topic', '')  # empty = disabled
        self.declare_parameter('output_topic', '/filtered_points')
        self.declare_parameter('enable_confidence_filter', False)
        self.declare_parameter('confidence_threshold', 50.0)
        self.declare_parameter('min_range_m', 0.2)
        self.declare_parameter('max_range_m', 2.0)          # was 6 reducing to reduce pcd size
        self.declare_parameter('downsample_stride',2)  # 1 = no downsample                                          - currently downsampling it by 2 to minimize the pcd
        self.declare_parameter('organized_output', False)  # False = old flat/compact/no-NaN format, for A/B testing

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.confidence_topic = self.get_parameter('confidence_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.enable_confidence_filter = self.get_parameter('enable_confidence_filter').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.min_range_m = self.get_parameter('min_range_m').value
        self.max_range_m = self.get_parameter('max_range_m').value
        self.downsample_stride = max(1, int(self.get_parameter('downsample_stride').value))
        self.organized_output = self.get_parameter('organized_output').value

        self.bridge = CvBridge()

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 5)

        self.depth_sub = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, self.camera_info_topic, qos_profile=qos_profile_sensor_data)

        subs = [self.depth_sub, self.info_sub]

        if self.enable_confidence_filter and self.confidence_topic:
            self.conf_sub = message_filters.Subscriber(
                self, Image, self.confidence_topic, qos_profile=qos_profile_sensor_data)
            subs.append(self.conf_sub)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            subs, queue_size=10, slop=0.05)
        self.sync.registerCallback(self._callback)

        self.get_logger().info(
            f'depth_fusion_node up. depth={self.depth_topic} '
            f'info={self.camera_info_topic} -> {self.output_topic} '
            f'(depth-only, no RGB, organized cloud output)'
        )

# havent yet fused the image with the pcd for confidence masking
    def apply_confidence_mask(self, depth_m: np.ndarray, confidence_img) -> np.ndarray:

        if confidence_img is None:
            return depth_m
        return depth_m

# range filtering with respective to the cameras fov
    def apply_range_clip(self, points_xyz: np.ndarray) -> np.ndarray:

        range_ = np.linalg.norm(points_xyz, axis=2)
        out_of_range = (range_ < self.min_range_m) | (range_ > self.max_range_m)
        points_xyz[out_of_range] = np.nan
        return points_xyz

    def _callback(self, depth_msg: Image, info_msg: CameraInfo, *extra):
        self.get_logger().info(
            f'callback fired: depth_stamp={depth_msg.header.stamp.sec}.'
            f'{depth_msg.header.stamp.nanosec}, encoding={depth_msg.encoding}'
        )
        confidence_msg = extra[0] if extra else None

        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth_m = self._to_meters(depth, depth_msg.encoding)
        depth_m = self.apply_confidence_mask(depth_m, confidence_msg)


        fx = info_msg.p[0]
        fy = info_msg.p[5]
        cx = info_msg.p[2]
        cy = info_msg.p[6]
        if abs(info_msg.p[3]) > 1e-6:
            self.get_logger().warn(
                'CameraInfo.P has a nonzero Tx term -- this depth topic may '
                'not be registered to the frame.',
                once=True,
            )

        points_xyz = self._deproject(depth_m, fx, fy, cx, cy, stride=self.downsample_stride)
        points_xyz = self.apply_range_clip(points_xyz)

        if self.organized_output:
            cloud_msg = self._make_organized_pointcloud2(points_xyz, depth_msg.header)
        else:
            flat = points_xyz.reshape(-1, 3)
            flat = flat[~np.isnan(flat).any(axis=1)]
            if flat.shape[0] == 0:
                self.get_logger().info('callback: all points NaN after range clip, skipping publish')
                return
            cloud_msg = self._make_flat_pointcloud2(flat, depth_msg.header)
        self.get_logger().info(f'callback: publishing cloud, width={cloud_msg.width}')
        self.pub.publish(cloud_msg)

    @staticmethod
    def _to_meters(depth: np.ndarray, encoding: str) -> np.ndarray:
        if encoding == '16UC1':
            return depth.astype(np.float32) / 1000.0  # mm -> m
        return depth.astype(np.float32)  # already 32FC1 meters

# applies cameras intrinsic properties to the depth extracted value from the depth image
    @staticmethod
    def _deproject(depth_m, fx, fy, cx, cy, stride=1):

        h, w = depth_m.shape
        ys, xs = np.mgrid[0:h:stride, 0:w:stride] # no downsampling stride (incremental size) - inversely prop to accuracy -testing to reduce pcd size
        z = depth_m[ys, xs].astype(np.float32)   # extracting depth

        valid = np.isfinite(z) & (z > 0.0) # rejecting Nan and Infinite

#converting coordinates to float
        xs_f = xs.astype(np.float32)
        ys_f = ys.astype(np.float32)

# converting equation from pixel to points
        x = (xs_f - cx) * z / fx
        y = (ys_f - cy) * z / fy

        points = np.stack([x, y, z], axis=-1)  # (H', W', 3)
        points[~valid] = np.nan
        return points

#converting xyz points into sensor_msgs/msg/PointCloud2
    @staticmethod
    def _make_flat_pointcloud2(points_xyz: np.ndarray, header: Header) -> PointCloud2:

        n = points_xyz.shape[0]
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1), #4bytes
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1), #4bytes
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1), #4bytes
        ]
        point_step = 12

        msg = PointCloud2()
        msg.header = header   # frame_id and timestamp
        msg.height = 1
        msg.width = n
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * n  #converting entire cloud into 1 row
        msg.is_dense = True
        msg.data = points_xyz.astype(np.float32).tobytes() # converts np array into bytes for ros pipeline
        return msg

# preserving original image geometry
    @staticmethod
    def _make_organized_pointcloud2(points_xyz: np.ndarray, header: Header) -> PointCloud2:
        h, w, _ = points_xyz.shape
        fields = [                                                                  # represented as points i.e organized pcd into flat points
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12

        msg = PointCloud2()
        msg.header = header
        msg.height = h
        msg.width = w
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * w
        msg.is_dense = False  # organized cloud with NaNs is never dense
        msg.data = points_xyz.astype(np.float32).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DepthFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
