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
  /// `frame_offset_x` shifts each corner's x-coordinate before pose recovery.
  /// The estimator owns the half-frame intrinsics (`cx`), so it owns the
  /// normalization for a lens whose pixels live in the right half of a
  /// dual-fisheye frame (subtract `frame_width/2`). Front lens keeps the
  /// default 0.
  explicit MarkerPoseEstimator(
    insta360_geometry::LensParams lens, double frame_offset_x = 0.0);

  std::optional<double> estimateDistance(
    const std::vector<cv::Point2f> & corners_local,
    double marker_length_m) const override;

private:
  insta360_geometry::LensParams lens_;
  double frame_offset_x_;
};

}  // namespace insta360_cv

#endif  // INSTA360_CV__MARKER_POSE_ESTIMATOR_HPP_
