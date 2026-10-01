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
  int frame_width,
  insta360_geometry::LensParams front,
  insta360_geometry::LensParams rear,
  const insta360_geometry::FisheyeCamera & camera)
: marker_length_m_(marker_length_m),
  frame_width_(frame_width),
  front_estimator_(front),
  rear_estimator_(rear),
  camera_(&camera)
{
  if (marker_length_m <= 0.0) {
    throw std::invalid_argument("marker_length_m must be > 0");
  }
  if (frame_width <= 0) {
    throw std::invalid_argument("frame_width must be > 0");
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

    const bool is_front = center.x < frame_width_ / 2.0;
    const double offset_x = is_front ? 0.0 : frame_width_ / 2.0;

    std::vector<cv::Point2f> corners_local(4);
    for (std::size_t k = 0; k < 4; ++k) {
      corners_local[k] = cv::Point2f(
        static_cast<float>(corners[i][k].x - offset_x), corners[i][k].y);
    }

    const auto orientation = camera_->pixelToQuaternion(center);
    if (!orientation) {
      continue;  // marker center outside fisheye FOV — skip
    }

    Detection d;
    d.class_id = static_cast<std::uint32_t>(ids[i]);
    d.confidence = 1.0;
    d.center = center;
    d.size_x = bbox.width;
    d.size_y = bbox.height;
    d.orientation = *orientation;

    const MarkerPoseEstimator & estimator =
      is_front ? front_estimator_ : rear_estimator_;
    const auto distance = estimator.estimateDistance(corners_local, marker_length_m_);
    d.distance_m = distance ? *distance :
      std::numeric_limits<double>::quiet_NaN();

    detections.push_back(d);
  }

  return detections;
}

}  // namespace insta360_cv
