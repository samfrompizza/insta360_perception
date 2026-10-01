#ifndef INSTA360_CV__DETECTION_HPP_
#define INSTA360_CV__DETECTION_HPP_

#include <cstdint>
#include <limits>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace insta360_cv
{

struct Detection
{
  std::uint32_t class_id = 0;
  double confidence = 1.0;
  cv::Point2d center;   // pixel in full frame (1920x960)
  double size_x = 0.0;
  double size_y = 0.0;
  double distance_m = std::numeric_limits<double>::quiet_NaN();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

}  // namespace insta360_cv

#endif  // INSTA360_CV__DETECTION_HPP_
