#include <gtest/gtest.h>

#include <functional>
#include <memory>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <insta360_geometry/equidistant_model.hpp>
#include <insta360_geometry/fisheye_camera.hpp>

#include "insta360_cv/aruco_detector.hpp"
#include "insta360_cv/marker_pose_estimator.hpp"

using insta360_cv::ArucoDetector;

namespace
{

const insta360_geometry::LensParams kLens{480.0, 480.0, 480.0, 480.0};

class ArucoDetectorTest : public ::testing::Test
{
protected:
  ArucoDetectorTest()
  : camera_(kLens, kLens, kFrameWidth,
      std::make_unique<insta360_geometry::EquidistantModel>()),
    front_estimator_(kLens),
    rear_estimator_(kLens),
    detector_(
      cv::aruco::DICT_6X6_250, kMarkerLength, camera_,
      std::vector<const insta360_cv::PoseEstimator *>{
        &front_estimator_, &rear_estimator_},
      [](const cv::Point2d & c) {
        return c.x < kFrameWidth / 2.0 ? std::size_t{0} : std::size_t{1};
      })
  {
  }

  cv::Mat makeFrameWithMarker(int id, cv::Point center, int size) const
  {
    cv::Mat frame(960, 1920, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat marker;
    cv::aruco::drawMarker(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250),
      id, size, marker, 1);
    // drawMarker emits CV_8UC1; convert to BGR so copyTo lands it in the 3-channel frame.
    cv::Mat marker_bgr;
    cv::cvtColor(marker, marker_bgr, cv::COLOR_GRAY2BGR);
    cv::Mat roi = frame(cv::Rect(
      center.x - size / 2, center.y - size / 2, size, size));
    marker_bgr.copyTo(roi);
    return frame;
  }

  static constexpr int kFrameWidth = 1920;
  static constexpr double kMarkerLength = 0.15;

  insta360_geometry::FisheyeCamera camera_;
  insta360_cv::MarkerPoseEstimator front_estimator_;
  insta360_cv::MarkerPoseEstimator rear_estimator_;
  ArucoDetector detector_;
};

TEST_F(ArucoDetectorTest, DetectsSingleMarkerInFrontLens)
{
  const cv::Mat frame = makeFrameWithMarker(5, cv::Point(480, 480), 200);
  const auto detections = detector_.detect(frame);

  ASSERT_EQ(detections.size(), 1u);
  const auto & d = detections[0];
  EXPECT_EQ(d.class_id, 5u);
  EXPECT_NEAR(d.center.x, 480.0, 30.0);
  EXPECT_NEAR(d.center.y, 480.0, 30.0);
  EXPECT_NEAR(d.orientation.norm(), 1.0, 1e-6);

  const Eigen::Vector3d axis =
    d.orientation.toRotationMatrix() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(axis.x(), 1.0, 1e-3);  // front-lens center -> body +X
  EXPECT_NEAR(axis.y(), 0.0, 1e-1);
  EXPECT_NEAR(axis.z(), 0.0, 1e-1);
}

TEST_F(ArucoDetectorTest, EmptyFrameYieldsNoDetections)
{
  const cv::Mat frame = cv::Mat::zeros(960, 1920, CV_8UC3);
  const auto detections = detector_.detect(frame);
  EXPECT_TRUE(detections.empty());
}

}  // namespace
