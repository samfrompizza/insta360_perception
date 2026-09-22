#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

#include "insta360_geometry/equidistant_model.hpp"
#include "insta360_geometry/fisheye_camera.hpp"

using insta360_geometry::EquidistantModel;
using insta360_geometry::FisheyeCamera;
using insta360_geometry::LensParams;

namespace
{
const LensParams kFront{480.0, 480.0, 480.0, 480.0};
const LensParams kRear{480.0, 480.0, 480.0, 480.0};

std::unique_ptr<const insta360_geometry::FisheyeModel> makeModel()
{
  return std::make_unique<EquidistantModel>();
}
}  // namespace

TEST(FisheyeCamera, RejectsNonPositiveFocalLength)
{
  LensParams bad{0.0, 480.0, 480.0, 480.0};
  EXPECT_THROW(
    FisheyeCamera(bad, kRear, 1920, makeModel()),
    std::invalid_argument);
}

TEST(FisheyeCamera, RejectsNonPositiveFrameWidth)
{
  EXPECT_THROW(
    FisheyeCamera(kFront, kRear, 0, makeModel()),
    std::invalid_argument);
}

TEST(FisheyeCamera, ConstructsWithValidParams)
{
  EXPECT_NO_THROW(FisheyeCamera(kFront, kRear, 1920, makeModel()));
}
