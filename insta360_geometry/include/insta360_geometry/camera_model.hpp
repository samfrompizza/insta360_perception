#ifndef INSTA360_GEOMETRY__CAMERA_MODEL_HPP_
#define INSTA360_GEOMETRY__CAMERA_MODEL_HPP_

#include <optional>

#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>

namespace insta360_geometry
{

/// Projection-agnostic camera: maps image pixels to unit quaternions in the
/// camera body frame, and back. Body frame: +X along the front lens axis.
/// Quaternion semantics A1: rotates body +X onto the ray direction.
class CameraModel
{
public:
  virtual ~CameraModel() = default;

  virtual std::optional<Eigen::Quaterniond> pixelToQuaternion(
    const cv::Point2d & pixel) const = 0;

  virtual std::optional<cv::Point2d> quaternionToPixel(
    const Eigen::Quaterniond & q) const = 0;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__CAMERA_MODEL_HPP_
