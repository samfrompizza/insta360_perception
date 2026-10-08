#include "insta360_cv/erp_pose_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

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

  // Virtual pinhole camera looking along center_dir. Build a right-handed
  // basis (right, up, forward) and project each corner ray into it.
  const Eigen::Vector3d forward = center_dir;
  Eigen::Vector3d up_ref = Eigen::Vector3d::UnitY();
  if (std::abs(forward.dot(up_ref)) > 0.99) {
    up_ref = Eigen::Vector3d::UnitZ();
  }
  const Eigen::Vector3d right = forward.cross(up_ref).normalized();
  const Eigen::Vector3d up = right.cross(forward).normalized();

  // ERP angular resolution at the marker latitude: pixels per radian.
  // ray_body.z = -sin(lat) under the ERP->body mapping, so lat = asin(-z).
  const double lat = std::asin(std::clamp(-forward.z(), -1.0, 1.0));
  const double f_local = (camera_->width() / (2.0 * M_PI)) * std::cos(lat);
  if (f_local <= 1e-6) {
    return std::nullopt;
  }

  const double cx = camera_->width() / 2.0;
  const double cy = camera_->height() / 2.0;
  const cv::Mat K = (cv::Mat_<double>(3, 3) <<
    f_local, 0.0, cx,
    0.0, f_local, cy,
    0.0, 0.0, 1.0);

  std::vector<cv::Point2f> pinhole(4);
  for (std::size_t i = 0; i < 4; ++i) {
    const double z = rays[i].dot(forward);
    if (z <= 1e-9) {
      return std::nullopt;
    }
    const double x = rays[i].dot(right) / z;
    const double y = rays[i].dot(up) / z;
    pinhole[i] = cv::Point2f(
      static_cast<float>(cx + f_local * x),
      static_cast<float>(cy - f_local * y));
  }

  const float m = static_cast<float>(marker_length_m / 2.0);
  const std::vector<cv::Point3f> object_points{
    {-m, m, 0.0f}, {m, m, 0.0f}, {m, -m, 0.0f}, {-m, -m, 0.0f}};

  const cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
  cv::Mat rvec;
  cv::Mat tvec;
  try {
    cv::solvePnP(object_points, pinhole, K, D, rvec, tvec);
  } catch (const cv::Exception &) {
    return std::nullopt;
  }

  return cv::norm(tvec);
}

}  // namespace insta360_cv
