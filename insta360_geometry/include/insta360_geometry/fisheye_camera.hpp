#ifndef INSTA360_GEOMETRY__FISHEYE_CAMERA_HPP_
#define INSTA360_GEOMETRY__FISHEYE_CAMERA_HPP_

#include <memory>
#include <optional>

#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>

#include "insta360_geometry/fisheye_model.hpp"
#include "insta360_geometry/lens_params.hpp"

namespace insta360_geometry
{

/// Dual-fisheye camera: maps pixels of a side-by-side dual-fisheye frame to
/// unit quaternions in the camera body frame, and back.
///
/// Body frame: X along the lens axis (front lens -> +X, rear lens -> -X),
/// Y and Z perpendicular, right-handed.
class FisheyeCamera
{
public:
  FisheyeCamera(
    LensParams front, LensParams rear, int frame_width,
    std::unique_ptr<const FisheyeModel> model);

  /// Pixel -> rotation quaternion (A1): rotates +X body onto the ray direction.
  std::optional<Eigen::Quaterniond> pixelToQuaternion(const cv::Point2d & pixel) const;

  /// Rotation quaternion -> pixel.
  std::optional<cv::Point2d> quaternionToPixel(const Eigen::Quaterniond & q) const;

private:
  LensParams front_;
  LensParams rear_;
  int frame_width_;
  std::unique_ptr<const FisheyeModel> model_;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__FISHEYE_CAMERA_HPP_
