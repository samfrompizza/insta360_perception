#ifndef INSTA360_CV__MARKER_POSE_ESTIMATOR_HPP_
#define INSTA360_CV__MARKER_POSE_ESTIMATOR_HPP_

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include <insta360_geometry/lens_params.hpp>

#include "insta360_cv/pose_estimator.hpp"

namespace insta360_cv
{

class MarkerPoseEstimator : public PoseEstimator
{
public:
  explicit MarkerPoseEstimator(insta360_geometry::LensParams lens);

  std::optional<double> estimateDistance(
    const std::vector<cv::Point2f> & corners_local,
    double marker_length_m) const override;

private:
  insta360_geometry::LensParams lens_;
};

}  // namespace insta360_cv

#endif  // INSTA360_CV__MARKER_POSE_ESTIMATOR_HPP_
