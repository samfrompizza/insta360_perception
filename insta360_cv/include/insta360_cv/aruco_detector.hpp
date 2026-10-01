#ifndef INSTA360_CV__ARUCO_DETECTOR_HPP_
#define INSTA360_CV__ARUCO_DETECTOR_HPP_

#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <insta360_geometry/fisheye_camera.hpp>
#include <insta360_geometry/lens_params.hpp>

#include "insta360_cv/detection.hpp"
#include "insta360_cv/marker_pose_estimator.hpp"

namespace insta360_cv
{

class ArucoDetector
{
public:
  ArucoDetector(
    int dictionary_id,
    double marker_length_m,
    int frame_width,
    insta360_geometry::LensParams front,
    insta360_geometry::LensParams rear,
    const insta360_geometry::FisheyeCamera & camera);

  std::vector<Detection> detect(const cv::Mat & frame_bgr) const;

private:
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  double marker_length_m_;
  int frame_width_;
  MarkerPoseEstimator front_estimator_;
  MarkerPoseEstimator rear_estimator_;
  const insta360_geometry::FisheyeCamera * camera_;
};

}  // namespace insta360_cv

#endif  // INSTA360_CV__ARUCO_DETECTOR_HPP_
