#ifndef INSTA360_CV__ARUCO_DETECTOR_HPP_
#define INSTA360_CV__ARUCO_DETECTOR_HPP_

#include <functional>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <insta360_geometry/camera_model.hpp>

#include "insta360_cv/detection.hpp"
#include "insta360_cv/pose_estimator.hpp"

namespace insta360_cv
{

class ArucoDetector
{
public:
  /// `estimator_selector` maps a marker center pixel to an index into
  /// `estimators`. For ERP pass a single estimator and a selector returning 0;
  /// for dual-fisheye pass front/rear estimators and select by frame half.
  ArucoDetector(
    int dictionary_id,
    double marker_length_m,
    const insta360_geometry::CameraModel & camera,
    std::vector<const PoseEstimator *> estimators,
    std::function<std::size_t(const cv::Point2d &)> estimator_selector);

  std::vector<Detection> detect(const cv::Mat & frame_bgr) const;

private:
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  double marker_length_m_;
  const insta360_geometry::CameraModel * camera_;
  std::vector<const PoseEstimator *> estimators_;
  std::function<std::size_t(const cv::Point2d &)> estimator_selector_;
};

}  // namespace insta360_cv

#endif  // INSTA360_CV__ARUCO_DETECTOR_HPP_
