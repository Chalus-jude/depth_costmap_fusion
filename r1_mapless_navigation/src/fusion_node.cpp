#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;
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
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");

    pub_ = create_publisher<PointCloud2>(output_topic_, rclcpp::SensorDataQoS());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // IMU runs on its own reentrant callback group so a MultiThreadedExecutor can
    // keep processing IMU samples while a depth frame is mid-callback, instead of
    // the depth callback blocking IMU updates on a single-threaded spin.
    imu_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions imu_opts;
    imu_opts.callback_group = imu_cb_group_;
    imu_sub_ = create_subscription<Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DepthFusionNode::imu_callback, this, std::placeholders::_1), imu_opts);

    const rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;

    depth_sub_.subscribe(this, depth_topic_, qos);
    info_sub_.subscribe(this, camera_info_topic_, qos);

// optional confidence filter using rgb fusion - disabled considering computational power
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
      "depth_fusion_node up. depth=%s info=%s imu=%s -> %s (depth-only, no RGB, organized cloud output, IMU tilt-corrected, output frame=%s)",
      depth_topic_.c_str(), camera_info_topic_.c_str(), imu_topic_.c_str(),
      output_topic_.c_str(), base_frame_.c_str());
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

// imu integration with quaternion from base_link to odom
  void imu_callback(const Imu::ConstSharedPtr & msg)
  {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    q.normalize();
    const rclcpp::Time stamp(msg->header.stamp);

    std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
    imu_buffer_.emplace_back(stamp, q);

    while (!imu_buffer_.empty() && (stamp - imu_buffer_.front().first).seconds() > 2.0) {
      imu_buffer_.pop_front();  // ~2s of history is far more than any realistic cloud latency
    }
  }


  std::optional<tf2::Quaternion> interpolate_imu(const rclcpp::Time & target_stamp)
  {
    std::lock_guard<std::mutex> lock(imu_buffer_mutex_);

    if (imu_buffer_.size() < 2) {
      return std::nullopt;
    }
    if (target_stamp < imu_buffer_.front().first || target_stamp > imu_buffer_.back().first) {
      return std::nullopt;
    }

    auto it = std::lower_bound(
      imu_buffer_.begin(), imu_buffer_.end(), target_stamp,
      [](const auto & sample, const rclcpp::Time & t) { return sample.first < t; });

    if (it == imu_buffer_.begin()) {
      return it->second;
    }
    auto it1 = it;
    auto it0 = std::prev(it);

    const double t0 = it0->first.seconds();
    const double t1 = it1->first.seconds();
    if (t1 <= t0) {
      return it0->second;
    }
    const double alpha = (target_stamp.seconds() - t0) / (t1 - t0);
    return it0->second.slerp(it1->second, alpha);
  }

  static tf2::Quaternion tilt_only(const tf2::Quaternion & q)
  {
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tf2::Quaternion out;
    out.setRPY(roll, pitch, 0.0);  // discarded yaw -- since already fused in ekf_filter_node for navigation in nav2
    return out;
  }

  bool get_corrected_transform(
    const std::string & camera_frame, const tf2::Quaternion & tilt_q, tf2::Transform & out)
  {
    if (!static_cam_to_base_.has_value()) {
      try {
        const auto tf_msg = tf_buffer_->lookupTransform(base_frame_, camera_frame, tf2::TimePointZero);
        tf2::Transform t;
        tf2::fromMsg(tf_msg.transform, t);
        static_cam_to_base_ = t;  // rigid mount, only ever need this once
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "%s->%s lookup failed: %s",
          camera_frame.c_str(), base_frame_.c_str(), ex.what());
        return false;
      }
    }

    const tf2::Transform tilt_tf(tilt_q, tf2::Vector3(0, 0, 0));
    out = tilt_tf * static_cam_to_base_.value();
    return true;
  }

  static void apply_transform(cv::Mat_<cv::Vec3f> & points_xyz, const tf2::Transform & tf)
  {
    const tf2::Matrix3x3 & R = tf.getBasis();
    const tf2::Vector3 & t = tf.getOrigin();

    for (int r = 0; r < points_xyz.rows; ++r) {
      for (int c = 0; c < points_xyz.cols; ++c) {
        cv::Vec3f & p = points_xyz(r, c);
        if (std::isnan(p[0])) {
          continue;  // preserve NaN, don't transform garbage
        }
        const tf2::Vector3 out = R * tf2::Vector3(p[0], p[1], p[2]) + t;
        p[0] = static_cast<float>(out.x());
        p[1] = static_cast<float>(out.y());
        p[2] = static_cast<float>(out.z());
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
    RCLCPP_DEBUG(
      get_logger(), "callback fired: depth_stamp=%d.%u, encoding=%s",
      depth_msg->header.stamp.sec, depth_msg->header.stamp.nanosec, depth_msg->encoding.c_str());

    // NOTE: confidence masking still operates on the raw cv_bridge image (pre-deproject).
    // Since confidence fusion isn't implemented yet (apply_confidence_mask is a no-op),
    // this has no cost today. If confidence masking is implemented later, mask here,
    // before deproject(), not after -- same reasoning as skipping full-res to_meters below.
    cv::Mat depth_raw = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;
    depth_raw = apply_confidence_mask(depth_raw, confidence_msg);

    const double fx = info_msg->p[0];
    const double fy = info_msg->p[5];
    const double cx = info_msg->p[2];
    const double cy = info_msg->p[6];
    if (std::abs(info_msg->p[3]) > 1e-6) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "CameraInfo.P has a nonzero Tx term -- this depth topic may not be registered to the frame.");
    }

    // deproject() now converts raw depth -> meters and applies the range clip inline,
    // per sampled pixel only -- avoids a full-resolution convertTo() and a separate
    // full-grid range-clip pass over points that downsample_stride would otherwise discard.
    cv::Mat_<cv::Vec3f> points_xyz = deproject(
      depth_raw, depth_msg->encoding, fx, fy, cx, cy, downsample_stride_,
      min_range_m_, max_range_m_);

    const rclcpp::Time cloud_stamp(depth_msg->header.stamp);
    const auto imu_q_opt = interpolate_imu(cloud_stamp);
    if (!imu_q_opt.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no bracketing IMU samples for cloud stamp %d.%u -- dropping frame",
        depth_msg->header.stamp.sec, depth_msg->header.stamp.nanosec);
      return;
    }

    tf2::Transform corrected_tf;
    if (!get_corrected_transform(depth_msg->header.frame_id, tilt_only(imu_q_opt.value()), corrected_tf)) {
      return;  // TF not up yet -- drop rather than publish uncorrected geometry
    }

    apply_transform(points_xyz, corrected_tf);

    Header out_header = depth_msg->header;
    out_header.frame_id = base_frame_;  // cloud is now expressed in base_link, not camera_depth_optical_frame

    PointCloud2 cloud_msg;
    if (organized_output_) {
      cloud_msg = make_organized_pointcloud2(points_xyz, out_header);
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
      cloud_msg = make_flat_pointcloud2(flat, out_header);
    }

    RCLCPP_DEBUG(get_logger(), "callback: publishing cloud, width=%u", cloud_msg.width);
    pub_->publish(cloud_msg);
  }

  // Converts raw depth pixels (16UC1 mm, or 32FC1 m) to 3D points, but only for the
  // sampled (strided) pixels -- avoids converting/visiting every pixel in the full
  // image when downsample_stride > 1. Range clipping is folded in here too, so NaN'd
  // or out-of-range points never need a second full pass.
  static cv::Mat_<cv::Vec3f> deproject(
    const cv::Mat & depth_raw, const std::string & encoding,
    double fx, double fy, double cx, double cy, int stride,
    double min_range_m, double max_range_m)
  {
    const bool is_16u = (encoding == "16UC1");
    const int h = depth_raw.rows;
    const int w = depth_raw.cols;
    const int out_h = (h + stride - 1) / stride;
    const int out_w = (w + stride - 1) / stride;

    const float min_sq = static_cast<float>(min_range_m * min_range_m);
    const float max_sq = static_cast<float>(max_range_m * max_range_m);

    cv::Mat_<cv::Vec3f> points(out_h, out_w);

    for (int oy = 0, y = 0; oy < out_h; ++oy, y += stride) {
      for (int ox = 0, x = 0; ox < out_w; ++ox, x += stride) {
        const float z = is_16u
          ? static_cast<float>(depth_raw.at<uint16_t>(y, x)) * 0.001f  // mm -> m
          : depth_raw.at<float>(y, x);                                 // already meters

        const bool valid = std::isfinite(z) && z > 0.0f;
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

        // range clip, folded in here (squared distance, no sqrt) instead of a
        // separate full-grid pass afterward
        const float range_sq = px * px + py * py + z * z;
        if (range_sq < min_sq || range_sq > max_sq) {
          points(oy, ox) = cv::Vec3f(
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN());
          continue;
        }

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
  std::string base_frame_;
  std::string imu_topic_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;

  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<CameraInfo> info_sub_;
  message_filters::Subscriber<Image> conf_sub_;

  std::shared_ptr<message_filters::Synchronizer<ApproxTime2>> sync2_;
  std::shared_ptr<message_filters::Synchronizer<ApproxTime3>> sync3_;

  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  std::mutex imu_buffer_mutex_;
  std::deque<std::pair<rclcpp::Time, tf2::Quaternion>> imu_buffer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::optional<tf2::Transform> static_cam_to_base_;  // rigid mount, cached after first lookup
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthFusionNode>();

  // MultiThreadedExecutor so the IMU callback group (reentrant) can run
  // concurrently with the depth/sync callback group (default, mutually
  // exclusive), instead of a single-threaded spin serializing everything
  // and starving IMU updates while a depth frame is being processed.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
