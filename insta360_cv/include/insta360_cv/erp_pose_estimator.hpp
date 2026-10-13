#ifndef INSTA360_CV__ERP_POSE_ESTIMATOR_HPP_
#define INSTA360_CV__ERP_POSE_ESTIMATOR_HPP_

#include <optional>
#include <vector>

#include <Eigen/Geometry>
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
  /// Right-handed basis of the virtual pinhole camera looking along `forward`.
  struct PinholeBasis
  {
    Eigen::Vector3d right;
    Eigen::Vector3d up;
    Eigen::Vector3d forward;
  };

  /// Unit body-frame rays from the camera center to each of the four corners,
  /// plus their normalized mean direction (the marker center ray).
  /// Returns nullopt if any pixel is outside the frame or the mean is degenerate.
  std::optional<std::pair<std::vector<Eigen::Vector3d>, Eigen::Vector3d>>
  cornerRays(const std::vector<cv::Point2f> & corners) const;

  /// Builds the virtual pinhole basis oriented along `forward`.
  static PinholeBasis pinholeBasis(const Eigen::Vector3d & forward);

  /// ERP angular resolution (pixels per radian) at the latitude of `forward`.
  /// Returns nullopt near the poles, where the resolution collapses.
  std::optional<double> localFocalLength(const Eigen::Vector3d & forward) const;

  /// Projects each body-frame ray into the virtual pinhole image plane.
  /// Returns nullopt if any ray lies behind the camera plane.
  static std::optional<std::vector<cv::Point2f>> projectToPinhole(
    const std::vector<Eigen::Vector3d> & rays,
    const PinholeBasis & basis,
    double focal_length,
    double cx,
    double cy);

  /// Marker corners in its own plane (z=0), TL,TR,BR,BL (aruco convention).
  static std::vector<cv::Point3f> markerObjectPoints(double marker_length_m);

  const insta360_geometry::EquirectangularCamera * camera_;
};

}  // namespace insta360_cv

#endif  // INSTA360_CV__ERP_POSE_ESTIMATOR_HPP_
