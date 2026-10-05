#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using std_msgs::msg::Header;

class DepthFusionNode : public rclcpp::Node
{
public:
  DepthFusionNode()
  : Node("depth_fusion_node")
  {
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/camera/camera/depth/image_rect_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera/depth/camera_info");
    confidence_topic_ = declare_parameter<std::string>("confidence_topic", "");  // empty = disabled
    output_topic_ = declare_parameter<std::string>("output_topic", "/filtered_points");
    enable_confidence_filter_ = declare_parameter<bool>("enable_confidence_filter", false);
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 50.0);
    min_range_m_ = declare_parameter<double>("min_range_m", 0.2);
    max_range_m_ = declare_parameter<double>("max_range_m", 2.0);        // was 6, reduced to shrink pcd size
    downsample_stride_ = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("downsample_stride", 1)));               // 1 = no downsample - currently 2 to minimize the pcd
    organized_output_ = declare_parameter<bool>("organized_output", false);  // false = old flat/compact/no-NaN format, for A/B testing

    pub_ = create_publisher<PointCloud2>(output_topic_, 5);

    const rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;

    depth_sub_.subscribe(this, depth_topic_, qos);
    info_sub_.subscribe(this, camera_info_topic_, qos);

    if (enable_confidence_filter_ && !confidence_topic_.empty()) {
      conf_sub_.subscribe(this, confidence_topic_, qos);
      sync3_ = std::make_shared<message_filters::Synchronizer<ApproxTime3>>(
        ApproxTime3(10), depth_sub_, info_sub_, conf_sub_);
      sync3_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
      sync3_->registerCallback(
        std::bind(
          &DepthFusionNode::callback3, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3));
    } else {
      sync2_ = std::make_shared<message_filters::Synchronizer<ApproxTime2>>(
        ApproxTime2(10), depth_sub_, info_sub_);
      sync2_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
      sync2_->registerCallback(
        std::bind(
          &DepthFusionNode::callback2, this, std::placeholders::_1,
          std::placeholders::_2));
    }

    RCLCPP_INFO(
      get_logger(),
      "depth_fusion_node up. depth=%s info=%s -> %s (depth-only, no RGB, organized cloud output)",
      depth_topic_.c_str(), camera_info_topic_.c_str(), output_topic_.c_str());
  }

private:
  using ApproxTime2 = message_filters::sync_policies::ApproximateTime<Image, CameraInfo>;
  using ApproxTime3 = message_filters::sync_policies::ApproximateTime<Image, CameraInfo, Image>;

  // haven't yet fused the image with the pcd for confidence masking
  cv::Mat apply_confidence_mask(const cv::Mat & depth_m, const Image::ConstSharedPtr & confidence_img)
  {
    if (!confidence_img) {
      return depth_m;
    }
    return depth_m;
  }

  // range filtering with respect to the camera's fov
  void apply_range_clip(cv::Mat_<cv::Vec3f> & points_xyz)
  {
    for (int r = 0; r < points_xyz.rows; ++r) {
      for (int c = 0; c < points_xyz.cols; ++c) {
        cv::Vec3f & p = points_xyz(r, c);
        const float range = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        if (range < static_cast<float>(min_range_m_) || range > static_cast<float>(max_range_m_)) {
          p[0] = p[1] = p[2] = std::numeric_limits<float>::quiet_NaN();
        }
      }
    }
  }

  void callback2(const Image::ConstSharedPtr & depth_msg, const CameraInfo::ConstSharedPtr & info_msg)
  {
    process(depth_msg, info_msg, nullptr);
  }

  void callback3(
    const Image::ConstSharedPtr & depth_msg, const CameraInfo::ConstSharedPtr & info_msg,
    const Image::ConstSharedPtr & confidence_msg)
  {
    process(depth_msg, info_msg, confidence_msg);
  }

  void process(
    const Image::ConstSharedPtr & depth_msg, const CameraInfo::ConstSharedPtr & info_msg,
    const Image::ConstSharedPtr & confidence_msg)
  {
    RCLCPP_INFO(
      get_logger(), "callback fired: depth_stamp=%d.%u, encoding=%s",
      depth_msg->header.stamp.sec, depth_msg->header.stamp.nanosec, depth_msg->encoding.c_str());

    cv::Mat depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;
    cv::Mat depth_m = to_meters(depth, depth_msg->encoding);
    depth_m = apply_confidence_mask(depth_m, confidence_msg); //confidence not yet published since the rgb fusion is not yet implemented for confidence masking from the image

    const double fx = info_msg->p[0];
    const double fy = info_msg->p[5];
    const double cx = info_msg->p[2];
    const double cy = info_msg->p[6];
    if (std::abs(info_msg->p[3]) > 1e-6) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "CameraInfo.P has a nonzero Tx term -- this depth topic may not be registered to the frame.");
    }

    cv::Mat_<cv::Vec3f> points_xyz = deproject(depth_m, fx, fy, cx, cy, downsample_stride_);
    apply_range_clip(points_xyz);

    PointCloud2 cloud_msg;
    if (organized_output_) {
      cloud_msg = make_organized_pointcloud2(points_xyz, depth_msg->header);
    } else {
      std::vector<float> flat;
      flat.reserve(static_cast<size_t>(points_xyz.rows) * points_xyz.cols * 3);
      for (int r = 0; r < points_xyz.rows; ++r) {
        for (int c = 0; c < points_xyz.cols; ++c) {
          const cv::Vec3f & p = points_xyz(r, c);
          if (std::isnan(p[0]) || std::isnan(p[1]) || std::isnan(p[2])) {
            continue;
          }
          flat.push_back(p[0]);
          flat.push_back(p[1]);
          flat.push_back(p[2]);
        }
      }
      if (flat.empty()) {
        RCLCPP_INFO(get_logger(), "callback: all points NaN after range clip, skipping publish");
        return;
      }
      cloud_msg = make_flat_pointcloud2(flat, depth_msg->header);
    }

    RCLCPP_INFO(get_logger(), "callback: publishing cloud, width=%u", cloud_msg.width);
    pub_->publish(cloud_msg);
  }

  static cv::Mat to_meters(const cv::Mat & depth, const std::string & encoding)
  {
    cv::Mat out;
    if (encoding == "16UC1") {
      depth.convertTo(out, CV_32F, 1.0 / 1000.0);  // mm -> m
    } else {
      depth.convertTo(out, CV_32F);  // already 32FC1 meters
    }
    return out;
  }

  // applies the camera's intrinsic properties to the depth value extracted from the depth image
  static cv::Mat_<cv::Vec3f> deproject(
    const cv::Mat & depth_m, double fx, double fy, double cx, double cy, int stride)
  {
    const int h = depth_m.rows;
    const int w = depth_m.cols;
    const int out_h = (h + stride - 1) / stride;
    const int out_w = (w + stride - 1) / stride;

    cv::Mat_<cv::Vec3f> points(out_h, out_w);

    for (int oy = 0, y = 0; oy < out_h; ++oy, y += stride) {
      for (int ox = 0, x = 0; ox < out_w; ++ox, x += stride) {
        const float z = depth_m.at<float>(y, x);
        const bool valid = std::isfinite(z) && z > 0.0f;  // rejecting NaN and infinite

        if (!valid) {
          points(oy, ox) = cv::Vec3f(
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN());
          continue;
        }

        // pixel -> point conversion
        const float px = (static_cast<float>(x) - static_cast<float>(cx)) * z / static_cast<float>(fx);
        const float py = (static_cast<float>(y) - static_cast<float>(cy)) * z / static_cast<float>(fy);
        points(oy, ox) = cv::Vec3f(px, py, z);
      }
    }
    return points;
  }

  // converting xyz points into sensor_msgs/msg/PointCloud2
  static PointCloud2 make_flat_pointcloud2(const std::vector<float> & xyz, const Header & header)
  {
    const uint32_t n = static_cast<uint32_t>(xyz.size() / 3);

    PointCloud2 msg;
    msg.header = header;  // frame_id and timestamp
    msg.height = 1;
    msg.width = n;
    msg.fields = make_xyz_fields();
    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = msg.point_step * n;  // entire cloud in 1 row
    msg.is_dense = true;
    msg.data.resize(xyz.size() * sizeof(float));
    std::memcpy(msg.data.data(), xyz.data(), msg.data.size());
    return msg;
  }

  // preserving original image geometry
  static PointCloud2 make_organized_pointcloud2(const cv::Mat_<cv::Vec3f> & points_xyz, const Header & header)
  {
    const int h = points_xyz.rows;
    const int w = points_xyz.cols;

    PointCloud2 msg;
    msg.header = header;
    msg.height = h;
    msg.width = w;
    msg.fields = make_xyz_fields();
    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = msg.point_step * w;
    msg.is_dense = false;  // organized cloud with NaNs is never dense

    msg.data.resize(static_cast<size_t>(h) * w * 3 * sizeof(float));
    // cv::Mat_<Vec3f> rows may not be contiguous with the target layout if the Mat has
    // padding, so copy row by row to be safe.
    size_t offset = 0;
    for (int r = 0; r < h; ++r) {
      const size_t row_bytes = static_cast<size_t>(w) * 3 * sizeof(float);
      std::memcpy(msg.data.data() + offset, points_xyz.ptr<float>(r), row_bytes);
      offset += row_bytes;
    }
    return msg;
  }

  static std::vector<PointField> make_xyz_fields()
  {
    std::vector<PointField> fields(3);
    fields[0].name = "x"; fields[0].offset = 0; fields[0].datatype = PointField::FLOAT32; fields[0].count = 1;
    fields[1].name = "y"; fields[1].offset = 4; fields[1].datatype = PointField::FLOAT32; fields[1].count = 1;
    fields[2].name = "z"; fields[2].offset = 8; fields[2].datatype = PointField::FLOAT32; fields[2].count = 1;
    return fields;
  }

  // parameters
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string confidence_topic_;
  std::string output_topic_;
  bool enable_confidence_filter_;
  double confidence_threshold_;
  double min_range_m_;
  double max_range_m_;
  int downsample_stride_;
  bool organized_output_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;

  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<CameraInfo> info_sub_;
  message_filters::Subscriber<Image> conf_sub_;

  std::shared_ptr<message_filters::Synchronizer<ApproxTime2>> sync2_;
  std::shared_ptr<message_filters::Synchronizer<ApproxTime3>> sync3_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
