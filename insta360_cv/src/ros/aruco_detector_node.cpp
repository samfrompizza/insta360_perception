#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <insta360_geometry/equidistant_model.hpp>
#include <insta360_geometry/fisheye_camera.hpp>

#include "insta360_cv/aruco_detector.hpp"
#include "insta360_cv/detection.hpp"
#include "insta360_cv/msg/detection.hpp"
#include "insta360_cv/msg/detection_array.hpp"

namespace
{

geometry_msgs::msg::Quaternion toMsg(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

}  // namespace

namespace insta360_cv
{

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode()
  : Node("aruco_detector")
  {
    const int frame_width = declare_parameter("frame_width", 1920);
    const double marker_length = declare_parameter("marker_length", 0.15);
    const int dictionary = declare_parameter("dictionary", 9);
    draw_overlay_ = declare_parameter("draw_overlay", true);

    const insta360_geometry::LensParams front{
      declare_parameter("front_fx", 480.0),
      declare_parameter("front_fy", 480.0),
      declare_parameter("front_cx", 480.0),
      declare_parameter("front_cy", 480.0)};
    const insta360_geometry::LensParams rear{
      declare_parameter("rear_fx", 480.0),
      declare_parameter("rear_fy", 480.0),
      declare_parameter("rear_cx", 480.0),
      declare_parameter("rear_cy", 480.0)};

    camera_ = std::make_unique<insta360_geometry::FisheyeCamera>(
      front, rear, frame_width,
      std::make_unique<insta360_geometry::EquidistantModel>());

    detector_ = std::make_unique<ArucoDetector>(
      dictionary, marker_length, frame_width, front, rear, *camera_);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/dual_fisheye/image", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { onImage(msg); });

    detections_pub_ = create_publisher<insta360_cv::msg::DetectionArray>(
      "~/detections", 10);
    visualization_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "~/visualization", 10);
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    const std::vector<Detection> detections = detector_->detect(cv_ptr->image);

    insta360_cv::msg::DetectionArray array_msg;
    array_msg.header = msg->header;
    array_msg.detections.reserve(detections.size());
    for (const Detection & d : detections) {
      insta360_cv::msg::Detection m;
      m.header = msg->header;
      m.class_id = d.class_id;
      m.confidence = d.confidence;
      m.center_x = d.center.x;
      m.center_y = d.center.y;
      m.size_x = d.size_x;
      m.size_y = d.size_y;
      m.distance_m = d.distance_m;
      m.orientation = toMsg(d.orientation);
      array_msg.detections.push_back(m);
    }
    detections_pub_->publish(array_msg);

    if (draw_overlay_) {
      cv::Mat viz = cv_ptr->image.clone();
      for (const Detection & d : detections) {
        const cv::Rect bbox(
          static_cast<int>(d.center.x - d.size_x / 2.0),
          static_cast<int>(d.center.y - d.size_y / 2.0),
          static_cast<int>(d.size_x),
          static_cast<int>(d.size_y));
        cv::rectangle(viz, bbox, cv::Scalar(0, 0, 255), 2);

        char label[64];
        if (std::isnan(d.distance_m)) {
          std::snprintf(label, sizeof(label), "id %u nan", d.class_id);
        } else {
          std::snprintf(label, sizeof(label), "id %u %.1fm",
            d.class_id, d.distance_m);
        }
        cv::putText(viz, label, cv::Point(bbox.x, std::max(bbox.y - 5, 15)),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
      }
      cv_bridge::CvImage viz_msg(
        msg->header, sensor_msgs::image_encodings::BGR8, viz);
      visualization_pub_->publish(*viz_msg.toImageMsg());
    }
  }

  std::unique_ptr<insta360_geometry::FisheyeCamera> camera_;
  std::unique_ptr<ArucoDetector> detector_;
  bool draw_overlay_ = true;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<insta360_cv::msg::DetectionArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
};

}  // namespace insta360_cv

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<insta360_cv::ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
