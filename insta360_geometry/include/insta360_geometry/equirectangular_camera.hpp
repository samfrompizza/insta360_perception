#ifndef INSTA360_GEOMETRY__EQUIRECTANGULAR_CAMERA_HPP_
#define INSTA360_GEOMETRY__EQUIRECTANGULAR_CAMERA_HPP_

#include <optional>

#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>

#include "insta360_geometry/camera_model.hpp"

namespace insta360_geometry
{

/// Equirectangular (ERP) camera: maps pixels of a 2:1 equirectangular frame to
/// unit quaternions in the camera body frame, and back.
///
/// Convention matches insta360_ros_driver's equirectangular node:
///   lon = (x / width) * 2*pi - pi,  lat = (y / height) * pi - pi/2
///   ray_erp = (cos(lat)sin(lon), sin(lat), cos(lat)cos(lon))
/// with +Z_erp forward (image center), +X_erp right, +Y_erp up.
/// Body frame: +X along the front lens axis (ray_erp +Z -> body +X).
class EquirectangularCamera : public CameraModel
{
public:
  EquirectangularCamera(int width, int height);

  std::optional<Eigen::Quaterniond> pixelToQuaternion(
    const cv::Point2d & pixel) const override;

  std::optional<cv::Point2d> quaternionToPixel(
    const Eigen::Quaterniond & q) const override;

private:
  int width_;
  int height_;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__EQUIRECTANGULAR_CAMERA_HPP_
