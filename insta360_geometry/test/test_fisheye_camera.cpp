#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>

#include "insta360_geometry/equidistant_model.hpp"
#include "insta360_geometry/fisheye_camera.hpp"

using insta360_geometry::EquidistantModel;
using insta360_geometry::FisheyeCamera;
using insta360_geometry::LensParams;

namespace
{
const LensParams kFront{480.0, 480.0, 480.0, 480.0};
const LensParams kRear{480.0, 480.0, 480.0, 480.0};

std::unique_ptr<const insta360_geometry::FisheyeModel> makeModel()
{
  return std::make_unique<EquidistantModel>();
}
}  // namespace

TEST(FisheyeCamera, RejectsNonPositiveFocalLength)
{
  LensParams bad{0.0, 480.0, 480.0, 480.0};
  EXPECT_THROW(
    FisheyeCamera(bad, kRear, 1920, makeModel()),
    std::invalid_argument);
}

TEST(FisheyeCamera, RejectsNonPositiveFrameWidth)
{
  EXPECT_THROW(
    FisheyeCamera(kFront, kRear, 0, makeModel()),
    std::invalid_argument);
}

TEST(FisheyeCamera, ConstructsWithValidParams)
{
  EXPECT_NO_THROW(FisheyeCamera(kFront, kRear, 1920, makeModel()));
}

TEST(FisheyeCamera, FrontCenterPixelGivesIdentityRotation)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  // Front lens center is at (480, 480); ray along +X body -> identity quaternion.
  auto q = cam.pixelToQuaternion(cv::Point2d(480.0, 480.0));
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(std::abs(q->w()), 1.0, 1e-9);
  EXPECT_NEAR(q->x(), 0.0, 1e-9);
  EXPECT_NEAR(q->y(), 0.0, 1e-9);
  EXPECT_NEAR(q->z(), 0.0, 1e-9);
}

TEST(FisheyeCamera, RearCenterPixelGives180Rotation)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  // Rear lens center is at (1440, 480); ray along -X body.
  auto q = cam.pixelToQuaternion(cv::Point2d(1440.0, 480.0));
  ASSERT_TRUE(q.has_value());
  Eigen::Vector3d axis = q->toRotationMatrix() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(axis.x(), -1.0, 1e-9);
  EXPECT_NEAR(axis.y(), 0.0, 1e-9);
  EXPECT_NEAR(axis.z(), 0.0, 1e-9);
}

TEST(FisheyeCamera, QuaternionIsNormalized)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  auto q = cam.pixelToQuaternion(cv::Point2d(600.0, 400.0));
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(q->norm(), 1.0, 1e-12);
}

TEST(FisheyeCamera, ReferenceAxisMapsToRay)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  auto q = cam.pixelToQuaternion(cv::Point2d(600.0, 400.0));
  ASSERT_TRUE(q.has_value());
  Eigen::Vector3d ray = q->toRotationMatrix() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(ray.norm(), 1.0, 1e-12);
}

TEST(FisheyeCamera, RoundTripFrontAndRear)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  const double pixels[][2] = {
    {480.0, 480.0}, {600.0, 400.0}, {300.0, 700.0},
    {1440.0, 480.0}, {1300.0, 600.0}, {1600.0, 300.0},
  };
  for (const auto & p : pixels) {
    auto q = cam.pixelToQuaternion(cv::Point2d(p[0], p[1]));
    ASSERT_TRUE(q.has_value()) << "u=" << p[0] << " v=" << p[1];
    auto px = cam.quaternionToPixel(*q);
    ASSERT_TRUE(px.has_value()) << "u=" << p[0] << " v=" << p[1];
    EXPECT_NEAR(px->x, p[0], 1e-6) << "u=" << p[0] << " v=" << p[1];
    EXPECT_NEAR(px->y, p[1], 1e-6) << "u=" << p[0] << " v=" << p[1];
  }
}

TEST(FisheyeCamera, PixelOutsideCircleReturnsNullopt)
{
  FisheyeCamera cam(kFront, kRear, 1920, makeModel());
  // Front lens: 800 px below center (480, 480) -> r = 800 px > f*pi/2 ~ 754 px.
  EXPECT_FALSE(cam.pixelToQuaternion(cv::Point2d(480.0, 1280.0)).has_value());
}
