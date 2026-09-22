#include <gtest/gtest.h>

#include <cmath>

#include "insta360_geometry/equidistant_model.hpp"

using insta360_geometry::EquidistantModel;
using insta360_geometry::LensParams;

namespace
{
const LensParams kParams{480.0, 480.0, 480.0, 480.0};
}

TEST(EquidistantModel, CenterPixelMapsToOpticalAxis)
{
  EquidistantModel model;
  auto ray = model.pixelToRay(cv::Point2d(480.0, 480.0), kParams);
  ASSERT_TRUE(ray.has_value());
  EXPECT_NEAR(ray->x(), 0.0, 1e-12);
  EXPECT_NEAR(ray->y(), 0.0, 1e-12);
  EXPECT_NEAR(ray->z(), 1.0, 1e-12);
}

TEST(EquidistantModel, KnownRadiusGivesKnownAngle)
{
  EquidistantModel model;
  // r = 240 px, f = 480 px -> theta = 0.5 rad, along +X
  auto ray = model.pixelToRay(cv::Point2d(720.0, 480.0), kParams);
  ASSERT_TRUE(ray.has_value());
  EXPECT_NEAR(ray->x(), std::sin(0.5), 1e-12);
  EXPECT_NEAR(ray->y(), 0.0, 1e-12);
  EXPECT_NEAR(ray->z(), std::cos(0.5), 1e-12);
}

TEST(EquidistantModel, RayIsUnitLength)
{
  EquidistantModel model;
  auto ray = model.pixelToRay(cv::Point2d(600.0, 400.0), kParams);
  ASSERT_TRUE(ray.has_value());
  EXPECT_NEAR(ray->norm(), 1.0, 1e-12);
}

TEST(EquidistantModel, PixelOutsideCircleReturnsNullopt)
{
  EquidistantModel model;
  // r = 480*sqrt(2) ~ 679 px > f*pi/2 ~ 754? No: 679 < 754, so use a corner
  // far enough: r must exceed 754 px. Use (480 + 800, 480).
  auto ray = model.pixelToRay(cv::Point2d(1280.0, 480.0), kParams);
  EXPECT_FALSE(ray.has_value());
}
