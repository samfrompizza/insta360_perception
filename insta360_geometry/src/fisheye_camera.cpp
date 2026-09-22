#include "insta360_geometry/fisheye_camera.hpp"

#include <stdexcept>

namespace insta360_geometry
{

FisheyeCamera::FisheyeCamera(
  LensParams front, LensParams rear, int frame_width,
  std::unique_ptr<const FisheyeModel> model)
: front_(front), rear_(rear), frame_width_(frame_width), model_(std::move(model))
{
  if (front_.fx <= 0.0 || front_.fy <= 0.0) {
    throw std::invalid_argument("FisheyeCamera: front focal length must be positive");
  }
  if (rear_.fx <= 0.0 || rear_.fy <= 0.0) {
    throw std::invalid_argument("FisheyeCamera: rear focal length must be positive");
  }
  if (frame_width_ <= 0) {
    throw std::invalid_argument("FisheyeCamera: frame_width must be positive");
  }
  if (!model_) {
    throw std::invalid_argument("FisheyeCamera: model must not be null");
  }
}

std::optional<Eigen::Quaterniond> FisheyeCamera::pixelToQuaternion(
  const cv::Point2d &) const
{
  return std::nullopt;  // implemented in Task 6
}

std::optional<cv::Point2d> FisheyeCamera::quaternionToPixel(
  const Eigen::Quaterniond &) const
{
  return std::nullopt;  // implemented in Task 6
}

}  // namespace insta360_geometry
