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

namespace
{

// Optical frame -> body frame basis, expressed in body coordinates.
// Front lens: Z_opt = +X_body, X_opt = -Y_body, Y_opt = -Z_body.
Eigen::Vector3d opticalToBodyFront(const Eigen::Vector3d & r)
{
  return Eigen::Vector3d(r.z(), -r.x(), -r.y());
}

// Rear lens: Z_opt = -X_body, X_opt = +Y_body, Y_opt = -Z_body.
Eigen::Vector3d opticalToBodyRear(const Eigen::Vector3d & r)
{
  return Eigen::Vector3d(-r.z(), r.x(), -r.y());
}

// Inverse of the above: body -> optical frame.
Eigen::Vector3d bodyToOpticalFront(const Eigen::Vector3d & r)
{
  return Eigen::Vector3d(-r.y(), -r.z(), r.x());
}

Eigen::Vector3d bodyToOpticalRear(const Eigen::Vector3d & r)
{
  return Eigen::Vector3d(r.y(), -r.z(), -r.x());
}

}  // namespace

std::optional<Eigen::Quaterniond> FisheyeCamera::pixelToQuaternion(
  const cv::Point2d & pixel) const
{
  const bool is_front = pixel.x < static_cast<double>(frame_width_) / 2.0;
  const LensParams & params = is_front ? front_ : rear_;

  // The model works in a lens-local frame whose origin is the lens sub-image.
  // The rear sub-image starts at x = frame_width_/2, so shift it back.
  const cv::Point2d local =
    is_front ? pixel
             : cv::Point2d(pixel.x - static_cast<double>(frame_width_) / 2.0, pixel.y);

  auto ray_opt = model_->pixelToRay(local, params);
  if (!ray_opt.has_value()) {
    return std::nullopt;
  }

  const Eigen::Vector3d ray_body =
    is_front ? opticalToBodyFront(*ray_opt) : opticalToBodyRear(*ray_opt);

  return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), ray_body);
}

std::optional<cv::Point2d> FisheyeCamera::quaternionToPixel(
  const Eigen::Quaterniond & q) const
{
  const Eigen::Vector3d ray_body = q * Eigen::Vector3d::UnitX();

  // Front lens covers the +X hemisphere, rear the -X hemisphere.
  const bool is_front = ray_body.x() >= 0.0;
  const LensParams & params = is_front ? front_ : rear_;

  const Eigen::Vector3d ray_opt =
    is_front ? bodyToOpticalFront(ray_body) : bodyToOpticalRear(ray_body);

  auto local = model_->rayToPixel(ray_opt, params);
  if (!local.has_value()) {
    return std::nullopt;
  }

  // Undo the rear sub-image shift applied in pixelToQuaternion.
  return is_front ? *local
                  : cv::Point2d(local->x + static_cast<double>(frame_width_) / 2.0, local->y);
}

}  // namespace insta360_geometry
