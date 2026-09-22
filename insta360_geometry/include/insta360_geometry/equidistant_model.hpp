#ifndef INSTA360_GEOMETRY__EQUIDISTANT_MODEL_HPP_
#define INSTA360_GEOMETRY__EQUIDISTANT_MODEL_HPP_

#include <optional>

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

#include "insta360_geometry/fisheye_model.hpp"
#include "insta360_geometry/lens_params.hpp"

namespace insta360_geometry
{

/// Equidistant fisheye projection: r = f * theta.
class EquidistantModel : public FisheyeModel
{
public:
  std::optional<Eigen::Vector3d> pixelToRay(
    const cv::Point2d & pixel, const LensParams & params) const override;

  std::optional<cv::Point2d> rayToPixel(
    const Eigen::Vector3d & ray, const LensParams & params) const override;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__EQUIDISTANT_MODEL_HPP_
