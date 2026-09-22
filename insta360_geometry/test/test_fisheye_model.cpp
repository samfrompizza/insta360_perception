#include <gtest/gtest.h>

#include <memory>

#include "insta360_geometry/fisheye_model.hpp"

namespace
{

class StubModel : public insta360_geometry::FisheyeModel
{
public:
  std::optional<Eigen::Vector3d> pixelToRay(
    const cv::Point2d &, const insta360_geometry::LensParams &) const override
  {
    return Eigen::Vector3d::UnitZ();
  }

  std::optional<cv::Point2d> rayToPixel(
    const Eigen::Vector3d &, const insta360_geometry::LensParams &) const override
  {
    return cv::Point2d(0.0, 0.0);
  }
};

}  // namespace

TEST(FisheyeModel, PolymorphicDeletion)
{
  std::unique_ptr<insta360_geometry::FisheyeModel> model = std::make_unique<StubModel>();
  insta360_geometry::LensParams p{1.0, 1.0, 0.0, 0.0};
  auto ray = model->pixelToRay(cv::Point2d(0.0, 0.0), p);
  ASSERT_TRUE(ray.has_value());
  EXPECT_DOUBLE_EQ(ray->z(), 1.0);
}
