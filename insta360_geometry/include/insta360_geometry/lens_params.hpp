#ifndef INSTA360_GEOMETRY__LENS_PARAMS_HPP_
#define INSTA360_GEOMETRY__LENS_PARAMS_HPP_

namespace insta360_geometry
{

/// Intrinsic parameters of a single fisheye lens, in pixels.
struct LensParams
{
  double fx;
  double fy;
  double cx;
  double cy;
};

}  // namespace insta360_geometry

#endif  // INSTA360_GEOMETRY__LENS_PARAMS_HPP_
