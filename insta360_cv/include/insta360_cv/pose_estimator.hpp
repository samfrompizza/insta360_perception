#ifndef INSTA360_CV__POSE_ESTIMATOR_HPP_
#define INSTA360_CV__POSE_ESTIMATOR_HPP_

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace insta360_cv
{

/// Estimates the distance to a square marker from its four projected corners.
class PoseEstimator
{
public:
  virtual ~PoseEstimator() = default;

  virtual std::optional<double> estimateDistance(
    const std::vector<cv::Point2f> & corners,
    double marker_length_m) const = 0;
};

}  // namespace insta360_cv

#endif  // INSTA360_CV__POSE_ESTIMATOR_HPP_
