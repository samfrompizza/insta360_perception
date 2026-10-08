#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <insta360_geometry/equidistant_model.hpp>
#include <insta360_geometry/equirectangular_camera.hpp>
#include <insta360_geometry/fisheye_camera.hpp>

#include "insta360_cv/aruco_detector.hpp"
#include "insta360_cv/detection.hpp"
#include "insta360_cv/erp_margin.hpp"
#include "insta360_cv/erp_pose_estimator.hpp"
#include "insta360_cv/marker_pose_estimator.hpp"
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
    const std::string mode = declare_parameter("mode", std::string("erp"));
    const double marker_length = declare_parameter("marker_length", 0.15);
    const int dictionary = declare_parameter("dictionary", 9);
    draw_overlay_ = declare_parameter("draw_overlay", true);

    if (mode == "erp") {
      const int erp_width = declare_parameter("erp_width", 1920);
      const int erp_height = declare_parameter("erp_height", 960);
      seam_margin_px_ = declare_parameter("erp_seam_margin_px", 0);
      pole_margin_px_ = declare_parameter("erp_pole_margin_px", 0);

      auto camera = std::make_unique<insta360_geometry::EquirectangularCamera>(
        erp_width, erp_height);
      auto estimator = std::make_unique<ErpPoseEstimator>(*camera);
      detector_ = std::make_unique<ArucoDetector>(
        dictionary, marker_length, *camera,
        std::vector<const PoseEstimator *>{estimator.get()},
        [](const cv::Point2d &) { return std::size_t{0}; });
      erp_camera_ = std::move(camera);
      erp_estimator_ = std::move(estimator);
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/equirectangular/image", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { onImage(msg); });
    } else if (mode == "fisheye") {
      const int frame_width = declare_parameter("frame_width", 1920);
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

      auto camera = std::make_unique<insta360_geometry::FisheyeCamera>(
        front, rear, frame_width,
        std::make_unique<insta360_geometry::EquidistantModel>());
      auto front_est = std::make_unique<MarkerPoseEstimator>(front);
      auto rear_est = std::make_unique<MarkerPoseEstimator>(
        rear, frame_width / 2.0);
      const double half = frame_width / 2.0;
      detector_ = std::make_unique<ArucoDetector>(
        dictionary, marker_length, *camera,
        std::vector<const PoseEstimator *>{front_est.get(), rear_est.get()},
        [half](const cv::Point2d & c) {
          return c.x < half ? std::size_t{0} : std::size_t{1};
        });
      fisheye_camera_ = std::move(camera);
      front_estimator_ = std::move(front_est);
      rear_estimator_ = std::move(rear_est);
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/dual_fisheye/image", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { onImage(msg); });
    } else {
      throw std::invalid_argument("mode must be 'erp' or 'fisheye'");
    }

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

    std::vector<Detection> detections = detector_->detect(cv_ptr->image);
    if (erp_camera_) {
      detections.erase(
        std::remove_if(detections.begin(), detections.end(),
          [&](const Detection & d) {
            return isInErpMarginBand(
              d.center, erp_camera_->width(), erp_camera_->height(),
              seam_margin_px_, pole_margin_px_);
          }),
        detections.end());
    }

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

  std::unique_ptr<insta360_geometry::EquirectangularCamera> erp_camera_;
  std::unique_ptr<ErpPoseEstimator> erp_estimator_;
  std::unique_ptr<insta360_geometry::FisheyeCamera> fisheye_camera_;
  std::unique_ptr<MarkerPoseEstimator> front_estimator_;
  std::unique_ptr<MarkerPoseEstimator> rear_estimator_;
  std::unique_ptr<ArucoDetector> detector_;
  bool draw_overlay_ = true;
  int seam_margin_px_ = 0;
  int pole_margin_px_ = 0;

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
