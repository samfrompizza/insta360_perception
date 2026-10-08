#include <gtest/gtest.h>

#include <opencv2/core/types.hpp>

#include "insta360_cv/erp_margin.hpp"

using insta360_cv::isInErpMarginBand;

namespace
{
constexpr int kWidth = 1920;
constexpr int kHeight = 960;
}  // namespace

TEST(ErpMarginBand, RejectsDetectionInSeamBand)
{
  const cv::Point2d left_seam(10.0, 480.0);    // x < seam_margin
  const cv::Point2d right_seam(1910.0, 480.0); // x > width - seam_margin
  EXPECT_TRUE(isInErpMarginBand(left_seam, kWidth, kHeight, 20, 0));
  EXPECT_TRUE(isInErpMarginBand(right_seam, kWidth, kHeight, 20, 0));
}

TEST(ErpMarginBand, RejectsDetectionInPoleBand)
{
  const cv::Point2d top_pole(960.0, 10.0);    // y < pole_margin
  const cv::Point2d bottom_pole(960.0, 950.0); // y > height - pole_margin
  EXPECT_TRUE(isInErpMarginBand(top_pole, kWidth, kHeight, 0, 20));
  EXPECT_TRUE(isInErpMarginBand(bottom_pole, kWidth, kHeight, 0, 20));
}

TEST(ErpMarginBand, AcceptsDetectionInCenter)
{
  const cv::Point2d center(960.0, 480.0);
  EXPECT_FALSE(isInErpMarginBand(center, kWidth, kHeight, 20, 20));
}

TEST(ErpMarginBand, ZeroMarginAcceptsEverything)
{
  // A margin of 0 disables its band, so a detection exactly on the seam (x=0)
  // or pole (y=0) is not rejected.
  const cv::Point2d seam_exact(0.0, 480.0);
  const cv::Point2d pole_exact(960.0, 0.0);
  EXPECT_FALSE(isInErpMarginBand(seam_exact, kWidth, kHeight, 0, 0));
  EXPECT_FALSE(isInErpMarginBand(pole_exact, kWidth, kHeight, 0, 0));
}
