#include <gtest/gtest.h>

#include "insta360_geometry/lens_params.hpp"

TEST(LensParams, HoldsIntrinsics)
{
  insta360_geometry::LensParams p{480.0, 480.0, 480.0, 480.0};
  EXPECT_DOUBLE_EQ(p.fx, 480.0);
  EXPECT_DOUBLE_EQ(p.fy, 480.0);
  EXPECT_DOUBLE_EQ(p.cx, 480.0);
  EXPECT_DOUBLE_EQ(p.cy, 480.0);
}
