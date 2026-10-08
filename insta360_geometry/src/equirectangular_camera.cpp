#include "insta360_geometry/equirectangular_camera.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace insta360_geometry
{

EquirectangularCamera::EquirectangularCamera(int width, int height)
: width_(width), height_(height)
{
  if (width_ <= 0) {
    throw std::invalid_argument("EquirectangularCamera: width must be positive");
  }
  if (height_ <= 0) {
    throw std::invalid_argument("EquirectangularCamera: height must be positive");
  }
}

std::optional<Eigen::Quaterniond> EquirectangularCamera::pixelToQuaternion(
  const cv::Point2d & pixel) const
{
  if (pixel.x < 0.0 || pixel.x >= width_ || pixel.y < 0.0 || pixel.y >= height_) {
    return std::nullopt;
  }

  const double lon = (pixel.x / width_) * 2.0 * M_PI - M_PI;
  const double lat = (pixel.y / height_) * M_PI - M_PI / 2.0;

  const double cos_lat = std::cos(lat);
  const Eigen::Vector3d ray_erp(
    cos_lat * std::sin(lon), std::sin(lat), cos_lat * std::cos(lon));

  // ERP frame -> body frame: +Z_erp -> +X_body (front lens axis).
  const Eigen::Vector3d ray_body(ray_erp.z(), -ray_erp.x(), -ray_erp.y());

  return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), ray_body);
}

std::optional<cv::Point2d> EquirectangularCamera::quaternionToPixel(
  const Eigen::Quaterniond & q) const
{
  const Eigen::Vector3d ray_body = q * Eigen::Vector3d::UnitX();

  // Inverse of the ERP -> body mapping.
  const Eigen::Vector3d ray_erp(-ray_body.y(), -ray_body.z(), ray_body.x());

  const double lon = std::atan2(ray_erp.x(), ray_erp.z());
  const double lat = std::asin(std::clamp(ray_erp.y(), -1.0, 1.0));

  const double x = (lon + M_PI) / (2.0 * M_PI) * width_;
  const double y = (lat + M_PI / 2.0) / M_PI * height_;

  if (x < 0.0 || x >= width_ || y < 0.0 || y >= height_) {
    return std::nullopt;
  }
  return cv::Point2d(x, y);
}

}  // namespace insta360_geometry
