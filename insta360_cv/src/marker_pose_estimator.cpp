#include "insta360_cv/marker_pose_estimator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace insta360_cv
{

MarkerPoseEstimator::MarkerPoseEstimator(
  insta360_geometry::LensParams lens, double frame_offset_x)
: lens_(lens),
  frame_offset_x_(frame_offset_x)
{
}

std::optional<double> MarkerPoseEstimator::estimateDistance(
  const std::vector<cv::Point2f> & corners_local,
  double marker_length_m) const
{
  if (corners_local.size() != 4 || marker_length_m <= 0.0) {
    return std::nullopt;
  }

  // Normalize full-frame pixels of a rear lens into its half-frame coordinates.
  std::vector<cv::Point2f> corners(corners_local);
  for (cv::Point2f & c : corners) {
    c.x -= static_cast<float>(frame_offset_x_);
  }

  // Degenerate corners (zero projected area) cannot constrain a pose.
  if (cv::contourArea(corners) <= 0.0) {
    return std::nullopt;
  }

  // Marker in its own plane (z=0), corners TL,TR,BR,BL (aruco convention).
  const float m = static_cast<float>(marker_length_m / 2.0);
  const std::vector<cv::Point3f> object_points{
    {-m, m, 0.0f}, {m, m, 0.0f}, {m, -m, 0.0f}, {-m, -m, 0.0f}};

  const cv::Mat K = (cv::Mat_<double>(3, 3) <<
    lens_.fx, 0.0, lens_.cx,
    0.0, lens_.fy, lens_.cy,
    0.0, 0.0, 1.0);

  // OpenCV 4.5.4 (Humble) has no cv::fisheye::solvePnP. Undistort equidistant
  // (r = f*theta) pixels to pinhole (r = f*tan(theta)) pixels with the same K,
  // then recover pose with the standard pinhole solvePnP.
  const cv::Mat D_fisheye = cv::Mat::zeros(4, 1, CV_64F);
  const cv::Mat D_pinhole = cv::Mat::zeros(5, 1, CV_64F);

  std::vector<cv::Point2f> undistorted;
  cv::Mat rvec;
  cv::Mat tvec;
  try {
    cv::fisheye::undistortPoints(
      corners, undistorted, K, D_fisheye, cv::noArray(), K);
    cv::solvePnP(object_points, undistorted, K, D_pinhole, rvec, tvec);
  } catch (const cv::Exception &) {
    return std::nullopt;
  }

  return cv::norm(tvec);
}

}  // namespace insta360_cv
