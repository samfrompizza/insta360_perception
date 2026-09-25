#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "insta360_geometry/equidistant_model.hpp"
#include "insta360_geometry/fisheye_camera.hpp"

namespace
{

constexpr double kQuaternionTolerance = 1e-6;

bool quaternionsClose(
  const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  const double dot = std::abs(a.dot(b));
  return (1.0 - dot) < kQuaternionTolerance;
}

}  // namespace

class QuaternionVisualizer : public rclcpp::Node
{
public:
  QuaternionVisualizer()
  : Node("quaternion_visualizer")
  {
    const int frame_width = declare_parameter("frame_width", 1920);
    const int roi_size = declare_parameter("roi_size", 32);
    const bool full_frame = declare_parameter("full_frame", false);

    insta360_geometry::LensParams front{
      declare_parameter("front_fx", 480.0),
      declare_parameter("front_fy", 480.0),
      declare_parameter("front_cx", 480.0),
      declare_parameter("front_cy", 480.0)};
    insta360_geometry::LensParams rear{
      declare_parameter("rear_fx", 480.0),
      declare_parameter("rear_fy", 480.0),
      declare_parameter("rear_cx", 480.0),
      declare_parameter("rear_cy", 480.0)};

    camera_ = std::make_unique<insta360_geometry::FisheyeCamera>(
      front, rear, frame_width,
      std::make_unique<insta360_geometry::EquidistantModel>());

    roi_size_ = roi_size;
    full_frame_ = full_frame;

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/dual_fisheye/image", rclcpp::SensorDataQoS(),
      std::bind(&QuaternionVisualizer::onImage, this, std::placeholders::_1));

    command_sub_ = create_subscription<std_msgs::msg::String>(
      "~/marker_command", 10,
      std::bind(&QuaternionVisualizer::onCommand, this, std::placeholders::_1));

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/visualization", 10);
  }

private:
  void onCommand(const std_msgs::msg::String::SharedPtr msg)
  {
    std::istringstream stream(msg->data);
    std::string action;
    stream >> action;

    if (action == "clear") {
      markers_.clear();
      return;
    }

    double w = 0.0, x = 0.0, y = 0.0, z = 0.0;
    if (!(stream >> w >> x >> y >> z)) {
      RCLCPP_WARN(get_logger(), "Malformed command: '%s'", msg->data.c_str());
      return;
    }
    const double norm_sq = w * w + x * x + y * y + z * z;
    if (norm_sq == 0.0 || !std::isfinite(norm_sq)) {
      RCLCPP_WARN(
        get_logger(), "Zero or non-finite quaternion: '%s'", msg->data.c_str());
      return;
    }
    Eigen::Quaterniond q(w, x, y, z);
    q.normalize();

    if (action == "draw") {
      markers_.push_back(q);
    } else if (action == "delete") {
      markers_.erase(
        std::remove_if(
          markers_.begin(), markers_.end(),
          [&q](const Eigen::Quaterniond & m) { return quaternionsClose(m, q); }),
        markers_.end());
    } else {
      RCLCPP_WARN(get_logger(), "Unknown action: '%s'", action.c_str());
    }
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (markers_.empty()) {
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (full_frame_) {
      for (const auto & q : markers_) {
        auto pixel = camera_->quaternionToPixel(q);
        if (!pixel.has_value()) {
          continue;
        }
        drawMarker(cv_ptr->image, *pixel);
      }
      auto out = cv_bridge::CvImage(
        msg->header, sensor_msgs::image_encodings::BGR8, cv_ptr->image).toImageMsg();
      image_pub_->publish(*out);
      return;
    }

    for (const auto & q : markers_) {
      auto pixel = camera_->quaternionToPixel(q);
      if (!pixel.has_value()) {
        continue;
      }
      publishRoi(cv_ptr->image, *pixel, msg->header);
    }
  }

  // Draws the 5x5 red marker rectangle onto a full frame, clamped to its bounds.
  void drawMarker(cv::Mat & frame, const cv::Point2d & pixel)
  {
    const int u = static_cast<int>(std::lround(pixel.x));
    const int v = static_cast<int>(std::lround(pixel.y));
    const int x = std::clamp(u - 2, 0, std::max(0, frame.cols - 5));
    const int y = std::clamp(v - 2, 0, std::max(0, frame.rows - 5));
    const int w = std::min(5, frame.cols - x);
    const int h = std::min(5, frame.rows - y);
    cv::rectangle(frame, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 1);
  }

  void publishRoi(
    const cv::Mat & frame, const cv::Point2d & pixel,
    const std_msgs::msg::Header & header)
  {
    const int half = roi_size_ / 2;
    const int u = static_cast<int>(std::lround(pixel.x));
    const int v = static_cast<int>(std::lround(pixel.y));

    const int x0 = std::clamp(u - half, 0, std::max(0, frame.cols - roi_size_));
    const int y0 = std::clamp(v - half, 0, std::max(0, frame.rows - roi_size_));
    const int w = std::min(roi_size_, frame.cols - x0);
    const int h = std::min(roi_size_, frame.rows - y0);
    if (w <= 0 || h <= 0) {
      return;
    }

    cv::Mat roi = frame(cv::Rect(x0, y0, w, h)).clone();
    const int local_u = u - x0;
    const int local_v = v - y0;
    cv::rectangle(
      roi,
      cv::Rect(
        std::max(0, local_u - 2), std::max(0, local_v - 2),
        std::min(5, w), std::min(5, h)),
      cv::Scalar(0, 0, 255), 1);

    auto out = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, roi).toImageMsg();
    image_pub_->publish(*out);
  }

  std::unique_ptr<insta360_geometry::FisheyeCamera> camera_;
  int roi_size_{32};
  bool full_frame_{false};
  std::vector<Eigen::Quaterniond> markers_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuaternionVisualizer>());
  rclcpp::shutdown();
  return 0;
}
