#include "insta360_cv/aruco_detector.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace insta360_cv
{

ArucoDetector::ArucoDetector(
  int dictionary_id,
  double marker_length_m,
  const insta360_geometry::CameraModel & camera,
  std::vector<const PoseEstimator *> estimators,
  std::function<std::size_t(const cv::Point2d &)> estimator_selector)
: marker_length_m_(marker_length_m),
  camera_(&camera),
  estimators_(std::move(estimators)),
  estimator_selector_(std::move(estimator_selector))
{
  if (marker_length_m <= 0.0) {
    throw std::invalid_argument("marker_length_m must be > 0");
  }
  if (estimators_.empty()) {
    throw std::invalid_argument("at least one pose estimator is required");
  }
  if (!estimator_selector_) {
    throw std::invalid_argument("estimator_selector must be set");
  }
  try {
    dictionary_ = cv::aruco::getPredefinedDictionary(
      static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary_id));
  } catch (const cv::Exception &) {
    throw std::invalid_argument("unknown dictionary id");
  }
}

std::vector<Detection> ArucoDetector::detect(const cv::Mat & frame_bgr) const
{
  cv::Mat gray;
  cv::cvtColor(frame_bgr, gray, cv::COLOR_BGR2GRAY);

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  cv::aruco::detectMarkers(gray, dictionary_, corners, ids);

  std::vector<Detection> detections;
  detections.reserve(ids.size());

  for (std::size_t i = 0; i < ids.size(); ++i) {
    const cv::Rect bbox = cv::boundingRect(corners[i]);
    const cv::Point2d center(
      bbox.x + bbox.width / 2.0, bbox.y + bbox.height / 2.0);

    const auto orientation = camera_->pixelToQuaternion(center);
    if (!orientation) {
      continue;  // center outside the camera's valid region — skip
    }

    Detection d;
    d.class_id = static_cast<std::uint32_t>(ids[i]);
    d.confidence = 1.0;
    d.center = center;
    d.size_x = bbox.width;
    d.size_y = bbox.height;
    d.orientation = *orientation;

    const std::size_t idx = estimator_selector_(center);
    if (idx >= estimators_.size()) {
      continue;
    }
    const auto distance =
      estimators_[idx]->estimateDistance(corners[i], marker_length_m_);
    d.distance_m = distance ? *distance :
      std::numeric_limits<double>::quiet_NaN();

    detections.push_back(d);
  }

  return detections;
}

}  // namespace insta360_cv
