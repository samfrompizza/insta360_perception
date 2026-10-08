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

// Body-frame ray direction of an ERP point at (lat, lon), per the ERP->body
// mapping ray_body = (ray_erp.z, -ray_erp.x, -ray_erp.y).
Eigen::Vector3d bodyRay(double lat, double lon)
{
  const double cl = std::cos(lat);
  const Eigen::Vector3d ray_erp(
    cl * std::sin(lon), std::sin(lat), cl * std::cos(lon));
  return Eigen::Vector3d(ray_erp.z(), -ray_erp.x(), -ray_erp.y());
}

// Project a square marker of side `marker_length` at `distance_m` along body
// direction `center` into ERP pixels. The marker plane is perpendicular to
// `center` (facing the camera). Corner order is TL,TR,BR,BL in the marker's
// own (right, up) plane, matching the estimator's object_points.
std::vector<cv::Point2f> projectMarkerAt(
  const Eigen::Vector3d & center, double marker_length, double distance_m)
{
  const double m = marker_length / 2.0;

  const Eigen::Vector3d forward = center.normalized();
  Eigen::Vector3d up_ref = Eigen::Vector3d::UnitY();
  if (std::abs(forward.dot(up_ref)) > 0.99) {
    up_ref = Eigen::Vector3d::UnitZ();
  }
  const Eigen::Vector3d right = forward.cross(up_ref).normalized();
  const Eigen::Vector3d up = right.cross(forward).normalized();

  const Eigen::Vector3d origin = forward * distance_m;
  const Eigen::Vector3d corners_body[4] = {
    origin - m * right + m * up, origin + m * right + m * up,
    origin + m * right - m * up, origin - m * right - m * up};

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
  const auto corners =
    projectMarkerAt(Eigen::Vector3d::UnitX(), marker_length, true_distance);
  ASSERT_EQ(corners.size(), 4u);

  EquirectangularCamera cam(kW, kH);
  ErpPoseEstimator estimator(cam);
  const auto distance = estimator.estimateDistance(corners, marker_length);

  ASSERT_TRUE(distance.has_value());
  EXPECT_NEAR(*distance, true_distance, true_distance * 0.10);
}

TEST(ErpPoseEstimator, RecoversDistanceFromOffAxisMarker)
{
  const double marker_length = 0.15;
  const double true_distance = 2.0;
  // Center ray at nonzero latitude/longitude: exercises the off-axis
  // right/up/forward basis and mean-of-rays center_dir path for a non-+X
  // center. Does not validate the latitude-dependent focal length — f_local
  // cancels out of solvePnP, so distance is invariant to the latitude axis.
  const Eigen::Vector3d center = bodyRay(M_PI / 6.0, M_PI / 4.0);
  const auto corners = projectMarkerAt(center, marker_length, true_distance);
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
