#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Geometry>

#include "insta360_cv/erp_pose_estimator.hpp"
#include "insta360_geometry/equirectangular_camera.hpp"

using insta360_cv::ErpPoseEstimator;
using insta360_geometry::EquirectangularCamera;

namespace
{
constexpr int kW = 1920;
constexpr int kH = 960;

// Project a marker of side `marker_length` at `distance_m` along body +X into
// ERP pixels. Marker plane is perpendicular to +X (facing the camera).
std::vector<cv::Point2f> projectMarker(double marker_length, double distance_m)
{
  const double m = marker_length / 2.0;
  // Marker corners in body frame: +X forward, +Y up, +Z right.
  const Eigen::Vector3d corners_body[4] = {
    {distance_m, m, -m}, {distance_m, m, m},
    {distance_m, -m, m}, {distance_m, -m, -m}};

  EquirectangularCamera cam(kW, kH);
  std::vector<cv::Point2f> out;
  for (const auto & c : corners_body) {
    const Eigen::Vector3d dir = c.normalized();
    const Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), dir);
    auto px = cam.quaternionToPixel(q);
    if (!px.has_value()) {
      return {};
    }
    out.push_back(cv::Point2f(static_cast<float>(px->x), static_cast<float>(px->y)));
  }
  return out;
}
}  // namespace

TEST(ErpPoseEstimator, RecoversDistanceFromProjectedMarker)
{
  const double marker_length = 0.15;
  const double true_distance = 2.0;
  const auto corners = projectMarker(marker_length, true_distance);
  ASSERT_EQ(corners.size(), 4u);

  EquirectangularCamera cam(kW, kH);
  ErpPoseEstimator estimator(cam);
  const auto distance = estimator.estimateDistance(corners, marker_length);

  ASSERT_TRUE(distance.has_value());
  EXPECT_NEAR(*distance, true_distance, true_distance * 0.10);
}

TEST(ErpPoseEstimator, ReturnsNulloptOnDegenerateCorners)
{
  const std::vector<cv::Point2f> degenerate{
    {960.0f, 480.0f}, {960.0f, 480.0f}, {960.0f, 480.0f}, {960.0f, 480.0f}};

  EquirectangularCamera cam(kW, kH);
  ErpPoseEstimator estimator(cam);
  EXPECT_FALSE(estimator.estimateDistance(degenerate, 0.15).has_value());
}
