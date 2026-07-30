// ScanProjector - the ported pointcloud_to_laserscan binning core.
//
// Collapses the non-ground points of a gravity-aligned LabeledPointCloud into a
// ProjectedScan: B angular bins over [-pi, +pi], nearest return per bin, empty bin = NaN.
// This is the module that neutralises obstacle_detector's Livox-hostile assumptions - the
// detector downstream sees a uniform ordered scan and never learns that the sensor was a
// non-repetitive rosette (architecture doc sections 4.2, 5.2, 7).
//
// PORTED FROM, verbatim in semantics: pointcloud_to_laserscan_node.cpp:178-229 (the
// per-point loop of `cloudCallback`), audited at that path in this repository. The order of
// the gates is part of the port and is NOT an implementation detail - a point rejected by
// the height band must never be counted as a range rejection, or the census in
// ProjectionStats stops describing the same thing upstream's debug log does:
//
//     non-finite skip -> height band -> hypot(x,y) range bounds -> atan2(y,x) window
//     -> index = (angle - angle_min)/increment -> keep the minimum range in each bin
//
// THREE DELIBERATE DIVERGENCES FROM UPSTREAM, each measured rather than assumed. They are
// listed here because "verbatim port" is a claim that has to name its own exceptions.
//
//   1. THE OUT-OF-BOUNDS INDEX (an upstream defect, fixed here). Upstream computes
//      `int index = (angle - angle_min) / angle_increment` and indexes `ranges[index]`
//      with no upper check, having already accepted `angle == angle_max`. At the shipped
//      geometry that quotient evaluates to exactly `bins` - not merely for `angle == +pi`
//      but for the top TWO representable angles at or below +pi, because the true quotient
//      `360 - 5.09e-14` is closer to 360.0 than to the next double below it and the
//      division rounds up before the truncation ever happens. `atan2(+0.0f, -1.0f)` returns
//      exactly +pi, so a single point on the -X axis directly behind the robot is enough to
//      reach it. Upstream therefore writes one float past the end of `scan_msg->ranges`.
//      This port CLAMPS the index to `bins - 1`. Clamping rather than wrapping to bin 0 is
//      the correct repair: the out-of-range quotient is a rounding artefact at the TOP of
//      the window, and every angle in that neighbourhood genuinely belongs to the last bin.
//      Wrapping would be right only for the single exact-seam value +pi and would move the
//      other reachable angle a full bin width. Verified by the boundary-angle property test.
//
//   2. THE EMPTY-BIN SENTINEL. Upstream pre-fills with +inf (`use_inf=true` default) and
//      tests `range < ranges[index]`. This contract's empty bin is NaN (see
//      projected_scan.h for why), and `range < NaN` is false, so the test is written as
//      "the bin is empty, OR the new range beats what is there". That is exactly equivalent
//      to upstream's comparison, because every finite range is < +inf. The NaN -> +inf
//      conversion belongs to the optional ROS2 adapter, not here.
//
//   3. THE ARITHMETIC WIDTH. Upstream writes `double range = hypot(*iter_x, *iter_y)` on
//      float iterators with an unqualified call, so which overload it resolves to depends
//      on whether a transitively-included <math.h> has pulled the C++ float overloads into
//      the global namespace - `::hypot(double,double)` from C, or `std::hypot(float,float)`
//      as an exact match. This port pins the DOUBLE overload, explicitly, by widening x and
//      y first. The choice was measured, not assumed: over the in-window sample corpus the
//      two overloads disagree in 100% of cases BEFORE the result is rounded to the float
//      the scan actually stores, and in 0% of cases after it - the narrowing to `float`
//      absorbs the entire difference. The equivalence test drives the reference
//      implementation in BOTH modes and asserts they agree bin-for-bin, so if that ever
//      stops being true a test fails rather than a number quietly shifts.
//
// OWNERSHIP. The projector owns nothing per frame. `Project` fills a caller-owned
// ProjectedScan and performs no allocation once that buffer has been sized once, which is
// what the architecture's allocation-free steady state requires of this module (section 6).
#ifndef PERCEPTION_CORE_PROJECTION_SCAN_PROJECTOR_H_
#define PERCEPTION_CORE_PROJECTION_SCAN_PROJECTOR_H_

#include "perception/core/contracts/labeled_point_cloud.h"
#include "perception/core/contracts/projected_scan.h"
#include "perception/core/contracts/validation.h"

namespace perception::core {

// The projector's resolved parameters.
//
// WHY THIS IS NOT `ProjectionConfig`. `ProjectionConfig` lives in
// perception/integration/perception_config.h, and `perception_core` links
// `perception_contracts` and nothing else - that link set is how the architecture doc's
// "core must not reach yaml-cpp / MuJoCo / dpcbf" rule is enforced rather than merely
// stated (perception/CMakeLists.txt). Reaching into the integration header from core would
// compile, and would quietly make that enforcement a comment. The one-way mapping lives in
// perception/integration/projection_params.h, on the side of the boundary that is allowed
// to see both types, and the projection test asserts it is field-complete against the
// SHIPPED configs/perception.yaml rather than against a copy.
struct ScanProjectorParams {
  int bins = 360;

  // Fixed at the full circle by section 7. Kept as fields rather than constants because
  // the reference implementation is parameterised on them and the equivalence test drives
  // both through the same numbers.
  double angle_min_rad = -kPi;
  double angle_max_rad = kPi;

  double range_min_m = 0.10;
  double range_max_m = 10.0;

  double height_band_min_m = -0.70;
  double height_band_max_m = 0.20;

  // Fills ProjectedScan::bin_point_counts - the points-per-bin histogram the section 12
  // projection gate asks to have reported.
  bool collect_bin_histogram = true;

  const char* Validate() const;
  bool IsValid() const { return Validate() == nullptr; }
};

class ScanProjector {
 public:
  explicit ScanProjector(const ScanProjectorParams& params) : params_(params) {}

  const ScanProjectorParams& params() const { return params_; }

  // Fills `scan` from the kNonGround points of `cloud`. Returns nullptr on success, or a
  // STATIC reason string - the same non-allocating convention the contracts use, because
  // this runs on the perception thread.
  //
  // Every other label is skipped BEFORE the census begins, so ProjectionStats::input_points
  // is "non-ground points offered", exactly as projected_scan.h declares its denominator to
  // be. Ground, self and invalid points are not rejections; they were never candidates.
  const char* Project(const LabeledPointCloud& cloud, ProjectedScan* scan) const;

 private:
  ScanProjectorParams params_;
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_PROJECTION_SCAN_PROJECTOR_H_
