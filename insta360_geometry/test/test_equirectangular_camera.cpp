#include <gtest/gtest.h>

#include <cmath>

#include "insta360_geometry/equirectangular_camera.hpp"

using insta360_geometry::EquirectangularCamera;

namespace
{
constexpr int kW = 1920;
constexpr int kH = 960;
}  // namespace

TEST(EquirectangularCamera, RejectsNonPositiveDimensions)
{
  EXPECT_THROW(EquirectangularCamera(0, kH), std::invalid_argument);
  EXPECT_THROW(EquirectangularCamera(kW, 0), std::invalid_argument);
}

TEST(EquirectangularCamera, CenterPixelGivesIdentityRotation)
{
  EquirectangularCamera cam(kW, kH);
  // Center (W/2, H/2) -> lon=0, lat=0 -> ray_erp=(0,0,1) -> ray_body=(1,0,0).
  auto q = cam.pixelToQuaternion(cv::Point2d(kW / 2.0, kH / 2.0));
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(std::abs(q->w()), 1.0, 1e-9);
  EXPECT_NEAR(q->x(), 0.0, 1e-9);
  EXPECT_NEAR(q->y(), 0.0, 1e-9);
  EXPECT_NEAR(q->z(), 0.0, 1e-9);
}

TEST(EquirectangularCamera, QuaternionIsNormalized)
{
  EquirectangularCamera cam(kW, kH);
  auto q = cam.pixelToQuaternion(cv::Point2d(600.0, 400.0));
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(q->norm(), 1.0, 1e-12);
}

TEST(EquirectangularCamera, RoundTripGrid)
{
  EquirectangularCamera cam(kW, kH);
  for (double u = 1.0; u < kW; u += 120.0) {
    for (double v = 1.0; v < kH; v += 120.0) {
      auto q = cam.pixelToQuaternion(cv::Point2d(u, v));
      ASSERT_TRUE(q.has_value()) << "u=" << u << " v=" << v;
      auto px = cam.quaternionToPixel(*q);
      ASSERT_TRUE(px.has_value()) << "u=" << u << " v=" << v;
      EXPECT_NEAR(px->x, u, 1e-6) << "u=" << u << " v=" << v;
      EXPECT_NEAR(px->y, v, 1e-6) << "u=" << u << " v=" << v;
    }
  }
}

TEST(EquirectangularCamera, OutOfBoundsPixelReturnsNullopt)
{
  EquirectangularCamera cam(kW, kH);
  EXPECT_FALSE(cam.pixelToQuaternion(cv::Point2d(-1.0, 100.0)).has_value());
  EXPECT_FALSE(cam.pixelToQuaternion(cv::Point2d(kW, 100.0)).has_value());
  EXPECT_FALSE(cam.pixelToQuaternion(cv::Point2d(100.0, kH)).has_value());
}
