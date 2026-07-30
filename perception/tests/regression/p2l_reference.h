// A VERBATIM re-implementation of upstream pointcloud_to_laserscan's conversion core.
//
// TEST-ONLY. Nothing outside tests/ may include this header, and it must never be linked
// into a production target - it exists to be compared against, not to be used.
//
// This is the same device P3 used: the pre-change code transcribed into the test binary so
// the new implementation is checked against the old one rather than against a description
// of the old one. The transcription source is
// `pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp`, function `cloudCallback`,
// lines 155-229 as they stand in this repository (re-read and confirmed at P7 rather than
// taken from the architecture doc's citation - P3 was burned once by a stale line number).
//
// WHAT WAS CHANGED IN TRANSCRIPTION, and nothing else:
//
//   * The ROS plumbing is gone - the message header, the frame check, the TF transform, the
//     publish. None of it touches `ranges`. What remains is the bin allocation and the
//     per-point loop.
//   * `sensor_msgs::PointCloud2ConstIterator<float>` becomes a span of Eigen::Vector3f, and
//     `*iter_x` / `*iter_y` / `*iter_z` become `.x()` / `.y()` / `.z()`. Both are float
//     reads, which is the property that matters: the arithmetic sees exactly the same bits.
//   * The RCLCPP_DEBUG calls are dropped. They have no effect on `ranges`.
//   * `scan_msg->ranges` gains ONE guard slot past `ranges_size`, and writes into it are
//     counted instead of being undefined behaviour. Upstream really does index
//     `ranges[index]` with `index == ranges_size` when a point sits within about 1e-14 rad
//     of `angle_max` (`atan2(+0.0f, -1.0f)` returns exactly +pi, so this is reachable from
//     a single point on the -X axis). Reproducing the crash would prove nothing; counting
//     the event lets the equivalence test state precisely where the port diverges and why.
//
// Everything else - the gate order, the inclusive comparisons, the +inf pre-fill, the
// truncating int conversion, the float storage with a double comparison - is upstream's,
// unchanged. Do not tidy it.
#ifndef PERCEPTION_TESTS_REGRESSION_P2L_REFERENCE_H_
#define PERCEPTION_TESTS_REGRESSION_P2L_REFERENCE_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>

namespace p2l_reference {

// Upstream's node parameters, under upstream's names, with upstream's defaults where they
// are not overridden by this repository's config.
struct Params {
  double angle_min = -M_PI;
  double angle_max = M_PI;
  double angle_increment = M_PI / 180.0;
  double range_min = 0.0;
  double range_max = std::numeric_limits<double>::max();
  double min_height = -std::numeric_limits<double>::max();
  double max_height = std::numeric_limits<double>::max();
  bool use_inf = true;         // Upstream default.
  double inf_epsilon = 1.0;    // Upstream default; only read when use_inf is false.
};

// Which overload upstream's unqualified `hypot(*iter_x, *iter_y)` and
// `atan2(*iter_y, *iter_x)` resolve to on float arguments depends on whether a transitively
// included <math.h> has pulled the C++ float overloads into the global namespace. That is
// not knowable from the source alone, so the reference computes BOTH and the equivalence
// test asserts the resulting scans agree - turning an unanswerable question about upstream's
// include graph into a checked property of the output.
enum class Arithmetic {
  kDouble,        // ::hypot(double,double) / ::atan2(double,double) after widening.
  kFloatOverload  // std::hypot(float,float) / std::atan2(float,float), result widened.
};

struct Result {
  // Length is `ranges_size + 1`; the final element is the guard slot described above.
  std::vector<float> ranges;
  std::uint32_t ranges_size = 0;
  int guard_writes = 0;       // Points whose index landed at or past `ranges_size`.
  int guard_reads = 0;        // Same events, counted at the read that precedes the write.
};

inline Result Run(const std::vector<Eigen::Vector3f>& points, const Params& p,
                  Arithmetic arithmetic) {
  Result result;

  // determine amount of rays to create
  std::uint32_t ranges_size = std::ceil((p.angle_max - p.angle_min) / p.angle_increment);
  result.ranges_size = ranges_size;

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  // (+1 for the guard slot - the only structural change, see the header comment)
  if (p.use_inf) {
    result.ranges.assign(ranges_size + 1, std::numeric_limits<double>::infinity());
  } else {
    result.ranges.assign(ranges_size + 1, p.range_max + p.inf_epsilon);
  }

  // Iterate through pointcloud
  for (const Eigen::Vector3f& point : points) {
    const float iter_x = point.x();
    const float iter_y = point.y();
    const float iter_z = point.z();

    if (std::isnan(iter_x) || std::isnan(iter_y) || std::isnan(iter_z)) {
      continue;
    }

    if (iter_z > p.max_height || iter_z < p.min_height) {
      continue;
    }

    double range;
    if (arithmetic == Arithmetic::kDouble) {
      range = std::hypot(static_cast<double>(iter_x), static_cast<double>(iter_y));
    } else {
      range = std::hypot(iter_x, iter_y);  // float overload; result widened on assignment.
    }
    if (range < p.range_min) {
      continue;
    }
    if (range > p.range_max) {
      continue;
    }

    double angle;
    if (arithmetic == Arithmetic::kDouble) {
      angle = std::atan2(static_cast<double>(iter_y), static_cast<double>(iter_x));
    } else {
      angle = std::atan2(iter_y, iter_x);  // float overload; result widened on assignment.
    }
    if (angle < p.angle_min || angle > p.angle_max) {
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - p.angle_min) / p.angle_increment;
    if (index >= static_cast<int>(ranges_size)) {
      ++result.guard_reads;  // Upstream reads one past the end here.
    }
    if (range < result.ranges[index]) {
      if (index >= static_cast<int>(ranges_size)) ++result.guard_writes;
      result.ranges[index] = range;
    }
  }

  return result;
}

}  // namespace p2l_reference

#endif  // PERCEPTION_TESTS_REGRESSION_P2L_REFERENCE_H_
