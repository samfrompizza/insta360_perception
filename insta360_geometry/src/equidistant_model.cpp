#include "insta360_geometry/equidistant_model.hpp"

#include <algorithm>
#include <cmath>

namespace insta360_geometry
{

std::optional<Eigen::Vector3d> EquidistantModel::pixelToRay(
  const cv::Point2d & pixel, const LensParams & params) const
{
  const double dx = pixel.x - params.cx;
  const double dy = pixel.y - params.cy;
  const double r = std::hypot(dx, dy);

  // FOV limit: theta_max = pi/2 -> r_max = f * pi/2.
  const double r_max = params.fx * (M_PI / 2.0);
  if (r > r_max) {
    return std::nullopt;
  }

  if (r == 0.0) {
    return Eigen::Vector3d(0.0, 0.0, 1.0);
  }

  const double theta = r / params.fx;
  const double sin_theta = std::sin(theta);
  return Eigen::Vector3d(
    sin_theta * dx / r,
    sin_theta * dy / r,
    std::cos(theta));
}

std::optional<cv::Point2d> EquidistantModel::rayToPixel(
  const Eigen::Vector3d & ray, const LensParams & params) const
{
  const double norm = ray.norm();
  if (norm == 0.0) {
    return std::nullopt;
  }
  const Eigen::Vector3d unit = ray / norm;

  const double z = std::clamp(unit.z(), -1.0, 1.0);
  const double theta = std::acos(z);
  if (theta > M_PI / 2.0) {
    return std::nullopt;
  }

  const double r = params.fx * theta;
  const double phi = std::atan2(unit.y(), unit.x());
  return cv::Point2d(
    params.cx + r * std::cos(phi),
    params.cy + r * std::sin(phi));
}

}  // namespace insta360_geometry
