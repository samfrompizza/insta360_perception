#ifndef INSTA360_GEOMETRY__FISHEYE_MODEL_HPP_
#define INSTA360_GEOMETRY__FISHEYE_MODEL_HPP_

#include <optional>

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

#include "insta360_geometry/lens_params.hpp"

namespace insta360_geometry
{

/// Projection model for a fisheye lens.
///
/// Implementations map between a pixel and a unit ray expressed in the lens
/// optical frame (Z along the optical axis, X right, Y down).
class FisheyeModel
{
public:
  virtual ~FisheyeModel() = default;

  /// Pixel -> unit ray in the lens optical frame.
  /// Returns nullopt if the pixel lies outside the fisheye image circle.
  virtual std::optional<Eigen::Vector3d> pixelToRay(
    const cv::Point2d & pixel, const LensParams & params) const = 0;

  /// Unit ray in the lens optical frame -> pixel.
  /// Returns nullopt if the ray falls outside the lens field of view.
  virtual std::optional<cv::Point2d> rayToPixel(
    const Eigen::Vector3d & ray, const LensParams & params) const = 0;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__FISHEYE_MODEL_HPP_
