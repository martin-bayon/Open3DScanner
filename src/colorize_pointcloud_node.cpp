#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <nlohmann/json.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class ColorizePointCloudNode : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    PointCloud2,
    Image,
    CameraInfo>;

  ColorizePointCloudNode()
  : Node("colorize_pointcloud_node")
  {
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/ouster/points");
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
    output_topic_ = declare_parameter<std::string>("output_topic", "/colorized_points");

    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "os_lidar");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");

    calib_json_path_ = declare_parameter<std::string>("calib_json_path", "");
    calib_json_key_ = declare_parameter<std::string>("calib_json_key", "T_lidar_camera");
    calib_is_lidar_to_camera_ = declare_parameter<bool>("calib_is_lidar_to_camera", true);

    publish_uncolored_points_ = declare_parameter<bool>("publish_uncolored_points", false);
    use_distortion_ = declare_parameter<bool>("use_distortion", false);
    min_range_ = declare_parameter<double>("min_range", 0.0);
    max_range_ = declare_parameter<double>("max_range", 0.0);

    loadCalibrationFromJson();

    pub_colored_cloud_ = create_publisher<PointCloud2>(output_topic_, rclcpp::SensorDataQoS());

    sub_cloud_.subscribe(this, pointcloud_topic_, rmw_qos_profile_sensor_data);
    sub_image_.subscribe(this, image_topic_, rmw_qos_profile_sensor_data);
    sub_camera_info_.subscribe(this, camera_info_topic_, rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10),
      sub_cloud_,
      sub_image_,
      sub_camera_info_);

    sync_->registerCallback(
      std::bind(
        &ColorizePointCloudNode::callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    RCLCPP_INFO(get_logger(), "Colorizer node started.");
    RCLCPP_INFO(get_logger(), "PointCloud topic: %s", pointcloud_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Image topic     : %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "CameraInfo topic: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output topic    : %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "publish_uncolored_points: %s", publish_uncolored_points_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "use_distortion: %s", use_distortion_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "min_range: %.3f", min_range_);
    RCLCPP_INFO(get_logger(), "max_range: %.3f", max_range_);
  }

private:
  void loadCalibrationFromJson()
  {
    if (calib_json_path_.empty()) {
      throw std::runtime_error("Parameter calib_json_path is empty.");
    }

    std::ifstream file(calib_json_path_);
    if (!file.is_open()) {
      throw std::runtime_error("Could not open calibration file: " + calib_json_path_);
    }

    nlohmann::json json_data;
    file >> json_data;

    if (!json_data.contains(calib_json_key_)) {
      throw std::runtime_error(
        "Calibration JSON does not contain key: " + calib_json_key_);
    }

    const auto data = json_data[calib_json_key_];

    if (!data.is_array() || data.size() != 7) {
      throw std::runtime_error(
        "Calibration entry must be [x, y, z, qx, qy, qz, qw].");
    }

    const double x = data[0].get<double>();
    const double y = data[1].get<double>();
    const double z = data[2].get<double>();
    const double qx = data[3].get<double>();
    const double qy = data[4].get<double>();
    const double qz = data[5].get<double>();
    const double qw = data[6].get<double>();

    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize();

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = q.toRotationMatrix();
    T.translation() = Eigen::Vector3d(x, y, z);

    if (calib_is_lidar_to_camera_) {
      T_camera_lidar_ = T;
      RCLCPP_INFO(get_logger(), "Using calibration as LiDAR -> camera.");
    } else {
      T_camera_lidar_ = T.inverse();
      RCLCPP_INFO(get_logger(), "Using inverse calibration as LiDAR -> camera.");
    }

    RCLCPP_INFO_STREAM(get_logger(), "T_camera_lidar:\n" << T_camera_lidar_.matrix());
  }

  static float packRgbToFloat(const uint8_t r, const uint8_t g, const uint8_t b)
  {
    const uint32_t rgb_uint32 =
      (static_cast<uint32_t>(r) << 16) |
      (static_cast<uint32_t>(g) << 8) |
      static_cast<uint32_t>(b);

    float rgb_float;
    std::memcpy(&rgb_float, &rgb_uint32, sizeof(float));
    return rgb_float;
  }

  static bool getFieldOffset(
    const PointCloud2 & cloud,
    const std::string & field_name,
    size_t & offset)
  {
    for (const auto & field : cloud.fields) {
      if (field.name == field_name) {
        offset = field.offset;
        return true;
      }
    }
    return false;
  }

  static float readFloatField(
    const uint8_t * point_data,
    const size_t offset,
    const bool is_bigendian)
  {
    float value;
    std::memcpy(&value, point_data + offset, sizeof(float));

    if (is_bigendian) {
      uint8_t * bytes = reinterpret_cast<uint8_t *>(&value);
      std::reverse(bytes, bytes + sizeof(float));
    }

    return value;
  }

  bool convertImageToBgr(
    const Image::ConstSharedPtr image_msg,
    cv::Mat & image_bgr)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvShare(image_msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
      return false;
    }

    const cv::Mat image = cv_ptr->image;

    if (image.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty image.");
      return false;
    }

    try {
      if (image_msg->encoding == "bgr8") {
        image_bgr = image;
      } else if (image_msg->encoding == "rgb8") {
        cv::cvtColor(image, image_bgr, cv::COLOR_RGB2BGR);
      } else if (image_msg->encoding == "mono8") {
        cv::cvtColor(image, image_bgr, cv::COLOR_GRAY2BGR);
      } else if (image_msg->encoding == "rgba8") {
        cv::cvtColor(image, image_bgr, cv::COLOR_RGBA2BGR);
      } else if (image_msg->encoding == "bgra8") {
        cv::cvtColor(image, image_bgr, cv::COLOR_BGRA2BGR);
      } else {
        image_bgr = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Could not convert image encoding '%s' to bgr8: %s",
        image_msg->encoding.c_str(),
        e.what());
      return false;
    }

    return true;
  }

  bool projectPointFast(
    const double x_c,
    const double y_c,
    const double z_c,
    const double fx,
    const double fy,
    const double cx,
    const double cy,
    int & u,
    int & v) const
  {
    if (z_c <= 0.0) {
      return false;
    }

    const double inv_z = 1.0 / z_c;

    const double u_d = fx * x_c * inv_z + cx;
    const double v_d = fy * y_c * inv_z + cy;

    if (!std::isfinite(u_d) || !std::isfinite(v_d)) {
      return false;
    }

    u = static_cast<int>(std::round(u_d));
    v = static_cast<int>(std::round(v_d));

    return true;
  }

  bool projectPointWithDistortion(
    const double x_c,
    const double y_c,
    const double z_c,
    const cv::Mat & K,
    const cv::Mat & D,
    int & u,
    int & v) const
  {
    if (z_c <= 0.0) {
      return false;
    }

    std::vector<cv::Point3d> object_points;
    object_points.emplace_back(x_c, y_c, z_c);

    std::vector<cv::Point2d> image_points;

    static const cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    static const cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

    cv::projectPoints(
      object_points,
      rvec,
      tvec,
      K,
      D,
      image_points);

    if (image_points.empty()) {
      return false;
    }

    u = static_cast<int>(std::round(image_points[0].x));
    v = static_cast<int>(std::round(image_points[0].y));

    return std::isfinite(image_points[0].x) && std::isfinite(image_points[0].y);
  }

  void writeOutputPoint(
    sensor_msgs::PointCloud2Iterator<float> & out_x,
    sensor_msgs::PointCloud2Iterator<float> & out_y,
    sensor_msgs::PointCloud2Iterator<float> & out_z,
    sensor_msgs::PointCloud2Iterator<float> & out_rgb,
    const float x,
    const float y,
    const float z,
    const uint8_t r,
    const uint8_t g,
    const uint8_t b,
    size_t & output_count)
  {
    *out_x = x;
    *out_y = y;
    *out_z = z;
    *out_rgb = packRgbToFloat(r, g, b);

    ++out_x;
    ++out_y;
    ++out_z;
    ++out_rgb;
    ++output_count;
  }

  void callback(
    const PointCloud2::ConstSharedPtr cloud_msg,
    const Image::ConstSharedPtr image_msg,
    const CameraInfo::ConstSharedPtr camera_info_msg)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Synchronized callback running");

    cv::Mat image_bgr;
    if (!convertImageToBgr(image_msg, image_bgr)) {
      return;
    }

    const int width = image_bgr.cols;
    const int height = image_bgr.rows;

    if (width <= 0 || height <= 0) {
      RCLCPP_WARN(get_logger(), "Invalid image size: %dx%d", width, height);
      return;
    }

    const double fx = camera_info_msg->k[0];
    const double fy = camera_info_msg->k[4];
    const double cx = camera_info_msg->k[2];
    const double cy = camera_info_msg->k[5];

    if (fx <= 0.0 || fy <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid camera intrinsics: fx=%.3f fy=%.3f",
        fx,
        fy);
      return;
    }

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D;

    if (use_distortion_) {
      for (int i = 0; i < 9; ++i) {
        K.at<double>(i / 3, i % 3) = camera_info_msg->k[i];
      }

      if (!camera_info_msg->d.empty()) {
        D = cv::Mat(camera_info_msg->d, true).reshape(1, 1);
        D.convertTo(D, CV_64F);
      } else {
        D = cv::Mat::zeros(1, 5, CV_64F);
      }
    }

    size_t x_offset;
    size_t y_offset;
    size_t z_offset;

    if (!getFieldOffset(*cloud_msg, "x", x_offset) ||
        !getFieldOffset(*cloud_msg, "y", y_offset) ||
        !getFieldOffset(*cloud_msg, "z", z_offset))
    {
      RCLCPP_WARN(get_logger(), "Input cloud does not contain x, y, z fields.");
      return;
    }

    const size_t input_point_count =
      static_cast<size_t>(cloud_msg->width) * static_cast<size_t>(cloud_msg->height);

    PointCloud2 output_cloud;
    output_cloud.header = cloud_msg->header;
    output_cloud.header.frame_id =
      cloud_msg->header.frame_id.empty() ? lidar_frame_ : cloud_msg->header.frame_id;
    output_cloud.height = 1;
    output_cloud.is_bigendian = false;
    output_cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(output_cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);

    modifier.resize(input_point_count);

    sensor_msgs::PointCloud2Iterator<float> out_x(output_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> out_rgb(output_cloud, "rgb");

    size_t output_count = 0;
    size_t z_positive_count = 0;
    size_t inside_image_count = 0;
    size_t range_skipped_count = 0;

    const bool use_min_range = min_range_ > 0.0;
    const bool use_max_range = max_range_ > 0.0;
    const double min_range2 = min_range_ * min_range_;
    const double max_range2 = max_range_ * max_range_;

    for (size_t i = 0; i < input_point_count; ++i) {
      const uint8_t * point_data = &cloud_msg->data[i * cloud_msg->point_step];

      const float x_l = readFloatField(point_data, x_offset, cloud_msg->is_bigendian);
      const float y_l = readFloatField(point_data, y_offset, cloud_msg->is_bigendian);
      const float z_l = readFloatField(point_data, z_offset, cloud_msg->is_bigendian);

      if (!std::isfinite(x_l) || !std::isfinite(y_l) || !std::isfinite(z_l)) {
        continue;
      }

      if (use_min_range || use_max_range) {
        const double range2 =
          static_cast<double>(x_l) * static_cast<double>(x_l) +
          static_cast<double>(y_l) * static_cast<double>(y_l) +
          static_cast<double>(z_l) * static_cast<double>(z_l);

        if (use_min_range && range2 < min_range2) {
          ++range_skipped_count;

          if (publish_uncolored_points_) {
            writeOutputPoint(out_x, out_y, out_z, out_rgb, x_l, y_l, z_l, 0, 0, 0, output_count);
          }

          continue;
        }

        if (use_max_range && range2 > max_range2) {
          ++range_skipped_count;

          if (publish_uncolored_points_) {
            writeOutputPoint(out_x, out_y, out_z, out_rgb, x_l, y_l, z_l, 0, 0, 0, output_count);
          }

          continue;
        }
      }

      const Eigen::Vector3d p_lidar(
        static_cast<double>(x_l),
        static_cast<double>(y_l),
        static_cast<double>(z_l));

      const Eigen::Vector3d p_camera = T_camera_lidar_ * p_lidar;

      const double x_c = p_camera.x();
      const double y_c = p_camera.y();
      const double z_c = p_camera.z();

      bool has_color = false;
      uint8_t r = 0;
      uint8_t g = 0;
      uint8_t b = 0;

      if (z_c > 0.0) {
        ++z_positive_count;

        int u = -1;
        int v = -1;

        bool projected = false;

        if (use_distortion_) {
          projected = projectPointWithDistortion(x_c, y_c, z_c, K, D, u, v);
        } else {
          projected = projectPointFast(x_c, y_c, z_c, fx, fy, cx, cy, u, v);
        }

        if (projected && u >= 0 && u < width && v >= 0 && v < height) {
          const cv::Vec3b pixel = image_bgr.at<cv::Vec3b>(v, u);

          b = pixel[0];
          g = pixel[1];
          r = pixel[2];

          has_color = true;
          ++inside_image_count;
        }
      }

      if (!has_color && !publish_uncolored_points_) {
        continue;
      }

      writeOutputPoint(out_x, out_y, out_z, out_rgb, x_l, y_l, z_l, r, g, b, output_count);
    }

    if (output_count != input_point_count) {
      modifier.resize(output_count);
    }

    output_cloud.width = static_cast<uint32_t>(output_count);
    output_cloud.height = 1;
    output_cloud.row_step = output_cloud.point_step * output_cloud.width;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Input: %zu | z_cam>0: %zu | inside image: %zu | range skipped: %zu | output: %zu",
      input_point_count,
      z_positive_count,
      inside_image_count,
      range_skipped_count,
      output_count);

    pub_colored_cloud_->publish(output_cloud);
  }

private:
  std::string pointcloud_topic_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;

  std::string lidar_frame_;
  std::string camera_frame_;

  std::string calib_json_path_;
  std::string calib_json_key_;

  bool calib_is_lidar_to_camera_;
  bool publish_uncolored_points_;
  bool use_distortion_;

  double min_range_;
  double max_range_;

  Eigen::Isometry3d T_camera_lidar_ = Eigen::Isometry3d::Identity();

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_colored_cloud_;

  message_filters::Subscriber<PointCloud2> sub_cloud_;
  message_filters::Subscriber<Image> sub_image_;
  message_filters::Subscriber<CameraInfo> sub_camera_info_;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ColorizePointCloudNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
