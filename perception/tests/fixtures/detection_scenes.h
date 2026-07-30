// Synthetic ProjectedScan scenes and degenerate point sets for the P8 detector tests.
//
// TWO KINDS OF FIXTURE, AND THEY ARE NOT INTERCHANGEABLE.
//
//   * The SCAN scenes are RAY-CAST, not surface-sampled. For each bin the exact analytic
//     ray/cylinder or ray/wall intersection is solved and the near root stored. That matters
//     for the circle-fitting acceptance gates: a surface-sampled cylinder gives points at
//     bearings the scan does not actually have, so the measured centre error would be an
//     artefact of the sampling rather than of the estimator. Ray-casting also produces the
//     self-occlusion for free - only the sensor-facing arc can be hit - which is the whole
//     subject of the short-arc experiment.
//
//   * The DEGENERATE point sets are hand-built and are NOT scans. They exist to drive the
//     least-squares fit directly, into the corners where `arma::pinv` and any QR-based
//     substitute could legitimately disagree: sensor-collinear points (the design matrix loses
//     rank because the model cannot represent a line through the origin), duplicated points,
//     two-point groups, and points at the origin itself.
//
// GROUND TRUTH IS CARRIED, NOT RE-DERIVED. Each scene names the cylinders it was built from,
// so the acceptance metrics compare against the generator's own numbers rather than against
// something re-estimated from the same points.
#ifndef PERCEPTION_TESTS_FIXTURES_DETECTION_SCENES_H_
#define PERCEPTION_TESTS_FIXTURES_DETECTION_SCENES_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "perception/core/contracts/projected_scan.h"

namespace detection_scenes {

using perception::core::FrameId;
using perception::core::kPi;
using perception::core::MotionCompensation;
using perception::core::ProjectedScan;

inline constexpr int kBins = 360;
inline constexpr double kRangeMin = 0.10;
inline constexpr double kRangeMax = 10.0;

struct Cylinder {
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
};

// A wall as a finite segment in the plane, for the scenes that need something that is NOT a
// circle - the detector must not turn a wall into an obstacle circle, and a corpus of nothing
// but cylinders could never show that.
struct Wall {
  Eigen::Vector2d a{0.0, 0.0};
  Eigen::Vector2d b{0.0, 0.0};
};

// Per-bin provenance. Cylinders are labelled by their index; every wall shares kWallLabel,
// because "which wall" is never a question any acceptance metric asks. -1 means an empty bin.
inline constexpr int kWallLabel = 1000;
inline constexpr int kNoLabel = -1;

struct Scene {
  std::string name;
  std::string purpose;
  ProjectedScan scan;
  std::vector<Cylinder> cylinders;  // Ground truth, in the same frame as the scan.

  // Parallel to scan.ranges: which object won each bin. This is what makes cluster purity and
  // the over/under-segmentation rates measurable rather than eyeballed.
  std::vector<int> bin_labels;
};

// ---------------------------------------------------------------------------------------
// Ray casting.
// ---------------------------------------------------------------------------------------

// Near-root intersection of the ray {t * dir : t > 0} with a circle. Returns a negative value
// when the ray misses.
inline double RayCylinder(double bearing, const Cylinder& cylinder) {
  const double dx = std::cos(bearing);
  const double dy = std::sin(bearing);
  const double b = dx * cylinder.x + dy * cylinder.y;
  const double c = cylinder.x * cylinder.x + cylinder.y * cylinder.y -
                   cylinder.radius * cylinder.radius;
  const double discriminant = b * b - c;
  if (discriminant < 0.0) return -1.0;
  const double t = b - std::sqrt(discriminant);
  return t > 0.0 ? t : -1.0;
}

inline double RayWall(double bearing, const Wall& wall) {
  const Eigen::Vector2d dir(std::cos(bearing), std::sin(bearing));
  const Eigen::Vector2d edge = wall.b - wall.a;
  const double denominator = dir.x() * edge.y() - dir.y() * edge.x();
  if (std::abs(denominator) < 1e-12) return -1.0;
  const double t = (wall.a.x() * edge.y() - wall.a.y() * edge.x()) / denominator;
  if (t <= 0.0) return -1.0;
  // The position along the wall, from `a x dir` - NOT `dir x a`, which is its negation and
  // would clip the wall to the half it does not occupy.
  const double s = (wall.a.x() * dir.y() - wall.a.y() * dir.x()) / denominator;
  if (s < 0.0 || s > 1.0) return -1.0;
  return t;
}

// ---------------------------------------------------------------------------------------
// Scan assembly.
// ---------------------------------------------------------------------------------------
struct ScanBuilder {
  ProjectedScan scan;
  std::vector<int> labels;

  explicit ScanBuilder(int bins = kBins) {
    scan.angle_min_rad = -kPi;
    scan.angle_max_rad = kPi;
    scan.range_min_m = kRangeMin;
    scan.range_max_m = kRangeMax;
    scan.height_band_min_m = -0.70;
    scan.height_band_max_m = 0.20;
    scan.frame = FrameId::kGravityAlignedBase;
    scan.motion_compensation = MotionCompensation::kDeskewedToStamp;
    scan.Resize(bins, false);
    labels.assign(static_cast<std::size_t>(bins), kNoLabel);
  }

  // Nearest return wins, exactly as the projector's reduction does - and the label follows the
  // winning range, so the provenance can never disagree with the value it describes.
  void Set(int bin, double range, int label) {
    if (bin < 0 || bin >= scan.bins()) return;
    if (range < scan.range_min_m || range > scan.range_max_m) return;
    const auto index = static_cast<std::size_t>(bin);
    const auto value = static_cast<float>(range);
    if (std::isnan(scan.ranges[index]) || value < scan.ranges[index]) {
      scan.ranges[index] = value;
      labels[index] = label;
    }
  }

  ProjectedScan Finish(double stamp_s, std::uint64_t sequence) {
    scan.stamp_s = stamp_s;
    scan.sequence = sequence;
    int occupied = 0;
    for (const float range : scan.ranges) {
      if (!std::isnan(range)) ++occupied;
    }
    // The census describes an already-reduced scan: every point that reached a bin won it, so
    // binned == occupied and nothing was rejected. That keeps ProjectionStats::IsBalanced true
    // without pretending to a provenance these scenes do not have.
    scan.stats.input_points = occupied;
    scan.stats.binned_points = occupied;
    scan.stats.occupied_bins = occupied;
    scan.stats.total_bins = scan.bins();
    return scan;
  }
};

// `arc_fraction` in (0, 1] keeps that leading fraction of the bins each cylinder is visible in,
// which is how the half-arc half of the short-arc experiment is produced: the same cylinder at
// the same range, seen through less of its own surface.
struct CastResult {
  ProjectedScan scan;
  std::vector<int> labels;
};

inline CastResult CastScene(const std::vector<Cylinder>& cylinders,
                            const std::vector<Wall>& walls, double arc_fraction, double stamp_s,
                            std::uint64_t sequence, int bins = kBins) {
  ScanBuilder builder(bins);
  const double increment = 2.0 * kPi / static_cast<double>(bins);

  for (std::size_t c = 0; c < cylinders.size(); ++c) {
    const Cylinder& cylinder = cylinders[c];
    std::vector<int> hit_bins;
    hit_bins.reserve(static_cast<std::size_t>(bins));
    for (int bin = 0; bin < bins; ++bin) {
      const double bearing = -kPi + static_cast<double>(bin) * increment;
      if (RayCylinder(bearing, cylinder) > 0.0) hit_bins.push_back(bin);
    }
    // A cylinder never straddles the +/-pi seam in these scenes, so the hit bins are contiguous
    // and "the leading fraction" is unambiguous.
    const auto keep = static_cast<std::size_t>(
        std::max(1.0, std::floor(arc_fraction * static_cast<double>(hit_bins.size()))));
    for (std::size_t i = 0; i < hit_bins.size() && i < keep; ++i) {
      const double bearing = -kPi + static_cast<double>(hit_bins[i]) * increment;
      builder.Set(hit_bins[i], RayCylinder(bearing, cylinder), static_cast<int>(c));
    }
  }

  for (const Wall& wall : walls) {
    for (int bin = 0; bin < bins; ++bin) {
      const double bearing = -kPi + static_cast<double>(bin) * increment;
      const double range = RayWall(bearing, wall);
      if (range > 0.0) builder.Set(bin, range, kWallLabel);
    }
  }

  CastResult result;
  result.scan = builder.Finish(stamp_s, sequence);
  result.labels = builder.labels;
  return result;
}

// Assembles a Scene from a cast, so no scene has to keep its scan and its labels in step by
// hand.
inline Scene MakeScene(std::string name, std::string purpose,
                       const std::vector<Cylinder>& cylinders, const std::vector<Wall>& walls,
                       double arc_fraction, double stamp_s, std::uint64_t sequence) {
  CastResult cast = CastScene(cylinders, walls, arc_fraction, stamp_s, sequence);
  return {std::move(name), std::move(purpose), cast.scan, cylinders, cast.labels};
}

// ---------------------------------------------------------------------------------------
// The scenes. Radii follow dpcbf_config.yaml's 0.20-0.30 m range so the corpus is this
// repository's problem rather than a generic one.
// ---------------------------------------------------------------------------------------

// The arena geometry, named once. Two scenes differ only in arc fraction and a third drops the
// cylinders entirely, so three copies of the same literals would be three chances to drift.
inline std::vector<Cylinder> ArenaCylinders() {
  return {{2.5, 0.0, 0.25}, {-1.8, 2.2, 0.30}, {0.5, -3.1, 0.20}, {4.0, 3.5, 0.28}};
}

inline std::vector<Wall> ArenaWalls() {
  return {{{6.0, -6.0}, {6.0, 6.0}},
          {{-6.0, -6.0}, {-6.0, 6.0}},
          {{-6.0, 6.0}, {6.0, 6.0}},
          {{-6.0, -6.0}, {6.0, -6.0}}};
}

inline Scene SingleCylinderFullArc() {
  return MakeScene("single_cylinder_full_arc", "one cylinder, the whole sensor-facing arc visible",
                   {{2.5, 0.0, 0.25}}, {}, 1.0, 1.0, 1);
}

inline Scene SingleCylinderHalfArc() {
  return MakeScene("single_cylinder_half_arc", "the same cylinder through half of its visible arc",
                   {{2.5, 0.0, 0.25}}, {}, 0.5, 2.0, 2);
}

// Two cylinders whose angular footprints nearly touch. This is the under-segmentation case:
// if the grouping threshold is too loose the two merge into one cluster and one circle.
inline Scene TwoAdjacentCylinders() {
  return MakeScene("two_adjacent_cylinders", "two cylinders with nearly touching angular footprints",
                   {{2.0, -0.45, 0.25}, {2.0, 0.45, 0.25}}, {}, 1.0, 3.0, 3);
}

// A near cylinder in front of a far one on almost the same bearing. The far cylinder's cluster
// boundary is an OCCLUSION edge, not a real object edge, which is what the visibility flags
// exist to say.
inline Scene OcclusionPair() {
  // The geometry is chosen so the far cylinder keeps ENOUGH unoccluded bins to survive
  // min_group_points - roughly nine. An occlusion scene whose occluded object falls below the
  // grouping floor tests nothing: there is no cluster left to carry the flag.
  return MakeScene("occlusion_pair", "a near cylinder occluding part of a far one",
                   {{1.2, 0.0, 0.20}, {2.6, 0.55, 0.30}}, {}, 1.0, 4.0, 4);
}

// Cylinders at four ranges. The short-arc bias is a function of the arc angle at the circle
// centre, which for a fixed radius barely changes with range - so a range sweep is the control
// that shows the bias is NOT simply a range effect.
inline Scene RangeSweep() {
  return MakeScene("range_sweep", "identical cylinders at 1.2, 3.0, 5.0 and 7.5 m",
                   {{1.2, 0.0, 0.25}, {0.0, 3.0, 0.25}, {-5.0, 0.0, 0.25}, {0.0, -7.5, 0.25}}, {},
                   1.0, 5.0, 5);
}

inline Scene RadiusSweep() {
  return MakeScene("radius_sweep", "cylinders of 0.20, 0.25, 0.28 and 0.30 m at equal range",
                   {{2.5, 0.0, 0.20}, {0.0, 2.5, 0.25}, {-2.5, 0.0, 0.28}, {0.0, -2.5, 0.30}}, {},
                   1.0, 6.0, 6);
}

inline Scene RadiusSweepHalfArc() {
  return MakeScene("radius_sweep_half_arc", "the radius sweep through half of each visible arc",
                   {{2.5, 0.0, 0.20}, {0.0, 2.5, 0.25}, {-2.5, 0.0, 0.28}, {0.0, -2.5, 0.30}}, {},
                   0.5, 7.0, 7);
}

// Walls only. Every circle this produces is a false positive by construction.
inline Scene WallsOnly() {
  return MakeScene("walls_only", "four walls and no cylinders; every circle here is a false positive",
                   {}, ArenaWalls(), 1.0, 8.0, 8);
}

// The arena: cylinders inside walls, which is the geometry the simulator actually presents.
inline Scene ArenaFullArc() {
  return MakeScene("arena_full_arc", "the dpcbf arena: four cylinders inside four walls",
                   ArenaCylinders(), ArenaWalls(), 1.0, 9.0, 9);
}

inline Scene ArenaHalfArc() {
  return MakeScene("arena_half_arc", "the same arena with every cylinder cut to half its visible arc",
                   ArenaCylinders(), ArenaWalls(), 0.5, 10.0, 10);
}

inline Scene Empty() {
  return MakeScene("empty", "no returns at all", {}, {}, 1.0, 11.0, 11);
}

inline std::vector<Scene> AllScenes() {
  return {SingleCylinderFullArc(), SingleCylinderHalfArc(), TwoAdjacentCylinders(),
          OcclusionPair(),         RangeSweep(),            RadiusSweep(),
          RadiusSweepHalfArc(),    WallsOnly(),             ArenaFullArc(),
          ArenaHalfArc(),          Empty()};
}

// Scenes whose acceptance metrics are reported as FULL-ARC, and those reported as HALF-ARC.
// The split is the measurement, so it is declared here once rather than inferred per test.
inline std::vector<Scene> FullArcScenes() {
  return {SingleCylinderFullArc(), TwoAdjacentCylinders(), RangeSweep(), RadiusSweep(),
          ArenaFullArc()};
}

inline std::vector<Scene> HalfArcScenes() {
  return {SingleCylinderHalfArc(), RadiusSweepHalfArc(), ArenaHalfArc()};
}

// ---------------------------------------------------------------------------------------
// Degenerate point sets, for the least-squares fit itself.
// ---------------------------------------------------------------------------------------
struct FitCase {
  std::string name;
  std::string why;
  std::vector<Eigen::Vector2d> points;
};

inline std::vector<FitCase> DegenerateFitCases() {
  std::vector<FitCase> cases;

  {
    // Well conditioned: the control. A line that does not pass near the origin.
    std::vector<Eigen::Vector2d> points;
    for (int i = 0; i < 12; ++i) {
      points.emplace_back(2.0, -0.5 + 0.1 * i);
    }
    cases.push_back({"well_conditioned_line", "a wall at x = 2, the ordinary case", points});
  }

  {
    // EXACTLY sensor-collinear: every point is a multiple of one direction, so the design
    // matrix has rank 1 and the model cannot represent the line at all.
    std::vector<Eigen::Vector2d> points;
    const Eigen::Vector2d dir(std::cos(0.7), std::sin(0.7));
    for (int i = 0; i < 10; ++i) {
      points.push_back((1.0 + 0.05 * i) * dir);
    }
    cases.push_back({"exactly_radial", "points on a ray from the sensor; rank 1 by construction",
                     points});
  }

  {
    // NEARLY sensor-collinear, at four decades of perturbation. This is the band where a
    // rank-truncating solver and a plain solve part company.
    for (int exponent = 4; exponent <= 12; exponent += 4) {
      const double epsilon = std::pow(10.0, -static_cast<double>(exponent));
      std::vector<Eigen::Vector2d> points;
      const Eigen::Vector2d dir(std::cos(0.7), std::sin(0.7));
      const Eigen::Vector2d perp(-dir.y(), dir.x());
      for (int i = 0; i < 10; ++i) {
        points.push_back((1.0 + 0.05 * i) * dir + (epsilon * static_cast<double>(i)) * perp);
      }
      cases.push_back({"near_radial_1e-" + std::to_string(exponent),
                       "a ray perturbed sideways by 1e-" + std::to_string(exponent) + " m",
                       points});
    }
  }

  {
    std::vector<Eigen::Vector2d> points(8, Eigen::Vector2d(1.7, -0.9));
    cases.push_back({"duplicated_point", "eight copies of one point; rank 1", points});
  }

  {
    std::vector<Eigen::Vector2d> points = {{1.0, 2.0}, {1.0, 2.0000000001}};
    cases.push_back({"two_near_duplicate_points", "the smallest possible group, barely distinct",
                     points});
  }

  {
    std::vector<Eigen::Vector2d> points = {{1.0, 2.0}, {3.0, -1.0}};
    cases.push_back({"two_distinct_points", "the smallest possible group, well separated",
                     points});
  }

  {
    std::vector<Eigen::Vector2d> points(5, Eigen::Vector2d(0.0, 0.0));
    cases.push_back({"all_at_origin", "rank 0; the fit has no line to return", points});
  }

  {
    // A line that passes exactly through the origin but is not a ray: points on both sides.
    // Same rank-1 design matrix, different geometry - the min-norm answer is not the line.
    std::vector<Eigen::Vector2d> points;
    const Eigen::Vector2d dir(std::cos(0.3), std::sin(0.3));
    for (int i = -5; i <= 5; ++i) {
      if (i == 0) continue;
      points.push_back(static_cast<double>(i) * 0.4 * dir);
    }
    cases.push_back({"line_through_origin", "collinear WITH the sensor, straddling it", points});
  }

  {
    // An arc, which is what the detector mostly fits: well conditioned but not a line.
    std::vector<Eigen::Vector2d> points;
    for (int i = 0; i < 20; ++i) {
      const double theta = kPi * (0.5 + static_cast<double>(i) / 19.0);
      points.emplace_back(2.5 + 0.25 * std::cos(theta), 0.25 * std::sin(theta));
    }
    cases.push_back({"circular_arc", "a cylinder's visible arc; the fit's actual diet", points});
  }

  return cases;
}

}  // namespace detection_scenes

#endif  // PERCEPTION_TESTS_FIXTURES_DETECTION_SCENES_H_
