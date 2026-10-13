#include "insta360_cv/erp_pose_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace insta360_cv
{

ErpPoseEstimator::ErpPoseEstimator(
  const insta360_geometry::EquirectangularCamera & camera)
: camera_(&camera)
{
}

std::optional<double> ErpPoseEstimator::estimateDistance(
  const std::vector<cv::Point2f> & corners,
  double marker_length_m) const
{
  if (corners.size() != 4 || marker_length_m <= 0.0) {
    return std::nullopt;
  }
  if (cv::contourArea(corners) <= 0.0) {
    return std::nullopt;
  }

  const auto rays_and_center = cornerRays(corners);
  if (!rays_and_center.has_value()) {
    return std::nullopt;
  }
  const auto & [rays, center_dir] = *rays_and_center;

  const PinholeBasis basis = pinholeBasis(center_dir);

  const auto f_local = localFocalLength(basis.forward);
  if (!f_local.has_value()) {
    return std::nullopt;
  }

  const double cx = camera_->width() / 2.0;
  const double cy = camera_->height() / 2.0;
  const auto pinhole = projectToPinhole(rays, basis, *f_local, cx, cy);
  if (!pinhole.has_value()) {
    return std::nullopt;
  }

  const cv::Mat K = (cv::Mat_<double>(3, 3) <<
    *f_local, 0.0, cx,
    0.0, *f_local, cy,
    0.0, 0.0, 1.0);
  const cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);

  cv::Mat rvec;
  cv::Mat tvec;
  try {
    cv::solvePnP(
      markerObjectPoints(marker_length_m), *pinhole, K, D, rvec, tvec);
  } catch (const cv::Exception &) {
    return std::nullopt;
  }

  return cv::norm(tvec);
}

std::optional<std::pair<std::vector<Eigen::Vector3d>, Eigen::Vector3d>>
ErpPoseEstimator::cornerRays(const std::vector<cv::Point2f> & corners) const
{
  // Marker center direction (body frame) from the mean of the corner rays.
  Eigen::Vector3d center_dir = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> rays(4);
  for (std::size_t i = 0; i < 4; ++i) {
    auto q = camera_->pixelToQuaternion(cv::Point2d(corners[i].x, corners[i].y));
    if (!q.has_value()) {
      return std::nullopt;
    }
    rays[i] = q->toRotationMatrix() * Eigen::Vector3d::UnitX();
    center_dir += rays[i];
  }
  if (center_dir.norm() < 1e-9) {
    return std::nullopt;
  }
  center_dir.normalize();
  return std::make_pair(std::move(rays), center_dir);
}

ErpPoseEstimator::PinholeBasis ErpPoseEstimator::pinholeBasis(
  const Eigen::Vector3d & forward)
{
  // Virtual pinhole camera looking along `forward`. Build a right-handed
  // basis (right, up, forward); fall back to +Z when `forward` is near +Y.
  Eigen::Vector3d up_ref = Eigen::Vector3d::UnitY();
  if (std::abs(forward.dot(up_ref)) > 0.99) {
    up_ref = Eigen::Vector3d::UnitZ();
  }
  const Eigen::Vector3d right = forward.cross(up_ref).normalized();
  const Eigen::Vector3d up = right.cross(forward).normalized();
  return PinholeBasis{right, up, forward};
}

std::optional<double> ErpPoseEstimator::localFocalLength(
  const Eigen::Vector3d & forward) const
{
  // ERP angular resolution at the marker latitude: pixels per radian.
  // ray_body.z = -sin(lat) under the ERP->body mapping, so lat = asin(-z).
  const double lat = std::asin(std::clamp(-forward.z(), -1.0, 1.0));
  const double f_local = (camera_->width() / (2.0 * M_PI)) * std::cos(lat);
  if (f_local <= 1e-6) {
    return std::nullopt;
  }
  return f_local;
}

std::optional<std::vector<cv::Point2f>> ErpPoseEstimator::projectToPinhole(
  const std::vector<Eigen::Vector3d> & rays,
  const PinholeBasis & basis,
  double focal_length,
  double cx,
  double cy)
{
  std::vector<cv::Point2f> pinhole(rays.size());
  for (std::size_t i = 0; i < rays.size(); ++i) {
    const double z = rays[i].dot(basis.forward);
    if (z <= 1e-9) {
      return std::nullopt;
    }
    const double x = rays[i].dot(basis.right) / z;
    const double y = rays[i].dot(basis.up) / z;
    pinhole[i] = cv::Point2f(
      static_cast<float>(cx + focal_length * x),
      static_cast<float>(cy - focal_length * y));
  }
  return pinhole;
}

std::vector<cv::Point3f> ErpPoseEstimator::markerObjectPoints(
  double marker_length_m)
{
  const float m = static_cast<float>(marker_length_m / 2.0);
  return {{-m, m, 0.0f}, {m, m, 0.0f}, {m, -m, 0.0f}, {-m, -m, 0.0f}};
}

}  // namespace insta360_cv
