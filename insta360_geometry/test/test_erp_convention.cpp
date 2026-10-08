#include <gtest/gtest.h>

#include <cmath>

#include "insta360_geometry/equirectangular_camera.hpp"

using insta360_geometry::EquirectangularCamera;

namespace
{
constexpr int kW = 1920;
constexpr int kH = 960;

// Expected body-frame ray for a pixel, computed from the driver's convention:
//   lon = (x/W)*2pi - pi, lat = (y/H)*pi - pi/2
//   ray_erp = (cos(lat)sin(lon), sin(lat), cos(lat)cos(lon))
//   ray_body = (ray_erp.z, -ray_erp.x, -ray_erp.y)
Eigen::Vector3d expectedRay(double x, double y)
{
  const double lon = (x / kW) * 2.0 * M_PI - M_PI;
  const double lat = (y / kH) * M_PI - M_PI / 2.0;
  const double cl = std::cos(lat);
  const Eigen::Vector3d erp(cl * std::sin(lon), std::sin(lat), cl * std::cos(lon));
  return Eigen::Vector3d(erp.z(), -erp.x(), -erp.y());
}
}  // namespace

TEST(ErpConvention, MatchesDriverFormulaAtKeyPixels)
{
  EquirectangularCamera cam(kW, kH);
  const double pixels[][2] = {
    {960.0, 480.0},   // center: lon=0, lat=0
    {0.0, 480.0},     // seam: lon=-pi
    {480.0, 480.0},   // lon=-pi/2
    {1440.0, 480.0},  // lon=+pi/2
    {960.0, 0.0},     // south pole: lat=-pi/2
    {960.0, 959.0},   // near north pole
    {300.0, 200.0},   // arbitrary
  };
  for (const auto & p : pixels) {
    auto q = cam.pixelToQuaternion(cv::Point2d(p[0], p[1]));
    ASSERT_TRUE(q.has_value()) << "x=" << p[0] << " y=" << p[1];
    const Eigen::Vector3d ray = q->toRotationMatrix() * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d exp = expectedRay(p[0], p[1]);
    EXPECT_NEAR(ray.x(), exp.x(), 1e-9) << "x=" << p[0] << " y=" << p[1];
    EXPECT_NEAR(ray.y(), exp.y(), 1e-9) << "x=" << p[0] << " y=" << p[1];
    EXPECT_NEAR(ray.z(), exp.z(), 1e-9) << "x=" << p[0] << " y=" << p[1];
  }
}
