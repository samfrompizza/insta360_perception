#ifndef INSTA360_CV__ERP_POSE_ESTIMATOR_HPP_
#define INSTA360_CV__ERP_POSE_ESTIMATOR_HPP_

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include <insta360_geometry/equirectangular_camera.hpp>

#include "insta360_cv/pose_estimator.hpp"

namespace insta360_cv
{

/// Distance estimation for markers detected in an equirectangular frame.
/// Uses a local pinhole approximation: a virtual pinhole camera is oriented
/// along the marker's central ray, with focal length scaled by the ERP angular
/// resolution at the marker's latitude.
class ErpPoseEstimator : public PoseEstimator
{
public:
  explicit ErpPoseEstimator(const insta360_geometry::EquirectangularCamera & camera);

  std::optional<double> estimateDistance(
    const std::vector<cv::Point2f> & corners,
    double marker_length_m) const override;

private:
  const insta360_geometry::EquirectangularCamera * camera_;
};

}  // namespace insta360_cv

#endif  // INSTA360_CV__ERP_POSE_ESTIMATOR_HPP_
