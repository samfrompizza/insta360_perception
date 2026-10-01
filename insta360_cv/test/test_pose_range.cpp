#include <gtest/gtest.h>

#include <vector>

#include <opencv2/calib3d.hpp>

#include <insta360_geometry/lens_params.hpp>

#include "insta360_cv/marker_pose_estimator.hpp"

using insta360_cv::MarkerPoseEstimator;

namespace
{

const insta360_geometry::LensParams kLens{480.0, 480.0, 480.0, 480.0};

// Project a marker of the given side length, placed distance_m in front of the
// lens, into equidistant (fisheye) image coordinates (matches EquidistantModel).
std::vector<cv::Point2f> projectMarker(double marker_length, double distance_m)
{
  const float m = static_cast<float>(marker_length / 2.0);
  const std::vector<cv::Point3f> object{
    {-m, m, static_cast<float>(distance_m)},
    {m, m, static_cast<float>(distance_m)},
    {m, -m, static_cast<float>(distance_m)},
    {-m, -m, static_cast<float>(distance_m)}};

  const cv::Mat K = (cv::Mat_<double>(3, 3) <<
    kLens.fx, 0.0, kLens.cx,
    0.0, kLens.fy, kLens.cy,
    0.0, 0.0, 1.0);
  const cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);

  std::vector<cv::Point2f> image;
  cv::fisheye::projectPoints(
    object, image, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
    K, D);
  return image;
}

}  // namespace

TEST(MarkerPoseEstimatorTest, RecoversDistanceFromProjectedMarker)
{
  const double marker_length = 0.15;
  const double true_distance = 2.0;
  const auto corners = projectMarker(marker_length, true_distance);

  MarkerPoseEstimator estimator(kLens);
  const auto distance = estimator.estimateDistance(corners, marker_length);

  ASSERT_TRUE(distance.has_value());
  EXPECT_NEAR(*distance, true_distance, true_distance * 0.05);
}

TEST(MarkerPoseEstimatorTest, ReturnsNulloptOnDegenerateCorners)
{
  const std::vector<cv::Point2f> degenerate{
    {480.0f, 480.0f}, {480.0f, 480.0f}, {480.0f, 480.0f}, {480.0f, 480.0f}};

  MarkerPoseEstimator estimator(kLens);
  const auto distance = estimator.estimateDistance(degenerate, 0.15);

  EXPECT_FALSE(distance.has_value());
}
