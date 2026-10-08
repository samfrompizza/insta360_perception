#ifndef INSTA360_CV__ERP_MARGIN_HPP_
#define INSTA360_CV__ERP_MARGIN_HPP_

#include <opencv2/core/types.hpp>

namespace insta360_cv
{

/// True when a detection center falls inside the ERP seam (x) or pole (y)
/// exclusion bands and must be rejected. A margin of 0 disables its band, so a
/// center exactly on the seam (x == 0) or pole (y == 0) is accepted.
inline bool isInErpMarginBand(
  const cv::Point2d & center, int width, int height,
  int seam_margin_px, int pole_margin_px)
{
  return center.x < seam_margin_px ||
         center.x > width - seam_margin_px ||
         center.y < pole_margin_px ||
         center.y > height - pole_margin_px;
}

}  // namespace insta360_cv

#endif  // INSTA360_CV__ERP_MARGIN_HPP_
