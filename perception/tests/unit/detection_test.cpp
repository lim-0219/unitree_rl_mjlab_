// P8 acceptance: the ported obstacle_extractor detection core, sub-stage by sub-stage, and the
// short-arc circle-fitting experiment the architecture doc's section 12 asks for.
//
// The upstream-parity half of P8 lives in tests/regression/detection_oracle_test.cpp, which
// needs Armadillo and is therefore an optional target. This binary needs nothing but the core,
// and it is where the claims that do NOT reduce to "same as upstream" are proved - because
// agreeing with upstream cannot establish that upstream's answer is the one we want, and the
// circle-fitting gates are exactly the place where it is not obvious that it is.
//
// The sections, in order:
//   A  parameters and the config surface (the shipped YAML actually reaches the detector)
//   B  the line fit: the 2x2 SVD identity, exact recovery, and rank on degenerate input
//   C  grouping: gaps, min_group_points, and the occlusion visibility flags
//   D  split-and-merge: a corner splits, a straight run does not
//   E  circle-from-segment: the sqrt(3)/3 rule, enlargement, and both rejection paths
//   F  merging: collinear segments, contained and overlapping circles
//   G  contract validity and per-frame statelessness
//   H  cluster metrics on the scene corpus (precision, recall, purity, over/under-segmentation)
//   I  circle-fitting accuracy, reported SEPARATELY for full and half arcs
//   J  allocation-free steady state
//
// argv[1] is the perception/ directory, for the shipped configs/perception.yaml.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <map>
#include <new>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "perception/core/detection/line_fit.h"
#include "perception/core/detection/segment_circle_detector.h"
#include "perception/integration/detection_params.h"
#include "perception/integration/perception_config.h"

#include "check.h"
#include "detection_scenes.h"

using perception::core::Cluster2D;
using perception::core::CircleObservation;
using perception::core::Detection2DResult;
using perception::core::FittedPrimitive2D;
using perception::core::kPi;
using perception::core::LineFit2D;
using perception::core::LineFitAccumulator;
using perception::core::PrimitiveKind;
using perception::core::ProjectedScan;
using perception::core::SegmentCircleDetector;
using perception::core::SegmentCircleDetectorParams;
using perception::core::SvdUpperTriangular2x2;
using perception::core::UpperTriangular2x2Svd;
using perception::integration::PerceptionConfig;
using perception_test::Check;
using perception_test::CheckValid;
using perception_test::Section;

// ---------------------------------------------------------------------------------------
// Allocation counting for section J, the same device the projection test uses. Global
// operator new replacement is the only way to make "no steady-state allocation" a checked
// property rather than a comment.
// ---------------------------------------------------------------------------------------
namespace {
std::size_t g_allocations = 0;
}  // namespace

void* operator new(std::size_t size) {
  ++g_allocations;
  if (void* memory = std::malloc(size == 0 ? 1 : size)) return memory;
  throw std::bad_alloc();
}
void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }

namespace {

std::filesystem::path g_perception_dir;

// The shipped detection.* values, restated as literals for the same reason the projection test
// restates the projection ones: a fixture whose meaning depends on a file that can change is
// not a fixture. Section A asserts these still equal the YAML.
SegmentCircleDetectorParams ShippedParams() {
  SegmentCircleDetectorParams params;
  params.min_group_points = 5;
  params.max_group_distance_m = 0.10;
  params.distance_proportion = 2.0 * kPi / 360.0;
  params.max_split_distance_m = 0.20;
  params.max_merge_separation_m = 0.20;
  params.max_merge_spread_m = 0.20;
  params.max_circle_radius_m = 0.60;
  params.radius_enlargement_m = 0.17;
  params.circles_from_visibles = true;
  params.use_split_and_merge = true;
  params.discard_converted_segments = true;
  params.max_clusters = 256;
  params.max_primitives = 512;
  params.max_circles = 256;
  return params;
}

bool Near(double a, double b, double tolerance) { return std::abs(a - b) <= tolerance; }

// ---------------------------------------------------------------------------------------
// A  Parameters and the config surface.
// ---------------------------------------------------------------------------------------
void SectionA() {
  Section("A. parameters and the config surface");

  const SegmentCircleDetectorParams params = ShippedParams();
  Check(params.Validate() == nullptr, "the shipped parameters validate",
        params.Validate() == nullptr ? "" : params.Validate());

  SegmentCircleDetectorParams bad = params;
  bad.min_group_points = 1;
  Check(bad.Validate() != nullptr, "min_group_points = 1 is rejected");
  bad = params;
  bad.radius_enlargement_m = 0.60;
  Check(bad.Validate() != nullptr, "an enlargement at the radius cap is rejected");
  bad = params;
  bad.distance_proportion = 1.5;
  Check(bad.Validate() != nullptr, "a distance_proportion outside (0, 1) is rejected");

  // The shipped YAML must produce exactly the parameters this test reasons about. Without
  // this, detection.* could be edited and every acceptance number below would silently start
  // describing a different detector.
  PerceptionConfig config;
  bool loaded = false;
  std::string error;
  try {
    config = PerceptionConfig::LoadFromYaml(g_perception_dir / "configs" / "perception.yaml");
    loaded = true;
  } catch (const std::exception& exception) {
    error = exception.what();
  }
  Check(loaded, "the shipped configs/perception.yaml loads", error);
  if (!loaded) return;

  const SegmentCircleDetectorParams mapped =
      perception::integration::MakeSegmentCircleDetectorParams(config.detection);

  Check(mapped.min_group_points == params.min_group_points, "min_group_points reaches the detector");
  Check(Near(mapped.max_group_distance_m, params.max_group_distance_m, 1e-12),
        "max_group_distance_m reaches the detector");
  Check(Near(mapped.distance_proportion, params.distance_proportion, 1e-12),
        "distance_proportion reaches the detector, re-derived from projection.bins");
  Check(Near(mapped.max_split_distance_m, params.max_split_distance_m, 1e-12),
        "max_split_distance_m reaches the detector");
  Check(Near(mapped.max_merge_separation_m, params.max_merge_separation_m, 1e-12),
        "max_merge_separation_m reaches the detector");
  Check(Near(mapped.max_merge_spread_m, params.max_merge_spread_m, 1e-12),
        "max_merge_spread_m reaches the detector");
  Check(Near(mapped.max_circle_radius_m, params.max_circle_radius_m, 1e-12),
        "max_circle_radius_m reaches the detector");
  Check(Near(mapped.radius_enlargement_m, params.radius_enlargement_m, 1e-12),
        "radius_enlargement_m reaches the detector, still at upstream's value");
  Check(mapped.circles_from_visibles == params.circles_from_visibles,
        "circles_from_visibles reaches the detector");
  Check(mapped.use_split_and_merge == params.use_split_and_merge,
        "use_split_and_merge reaches the detector");
  Check(mapped.discard_converted_segments == params.discard_converted_segments,
        "discard_converted_segments reaches the detector");
  Check(mapped.max_clusters == params.max_clusters && mapped.max_primitives == params.max_primitives &&
            mapped.max_circles == params.max_circles,
        "the preallocation maxima reach the detector");

  // The shipped scan geometry is what makes the derived distance_proportion correct.
  Check(config.projection.bins == detection_scenes::kBins,
        "the scene fixtures use the shipped bin count",
        std::to_string(config.projection.bins));
}

// ---------------------------------------------------------------------------------------
// B  The line fit.
// ---------------------------------------------------------------------------------------
void SectionB() {
  Section("B. the least-squares line fit");

  // The 2x2 SVD identity, on a matrix chosen to have neither symmetry nor a zero.
  {
    const double f = 3.0;
    const double g = -1.5;
    const double h = 0.75;
    const UpperTriangular2x2Svd svd = SvdUpperTriangular2x2(f, g, h);
    const double cp = std::cos(svd.phi);
    const double sp = std::sin(svd.phi);
    const double ct = std::cos(svd.theta);
    const double st = std::sin(svd.theta);
    // U * diag(s1, s2) * V^T, written out.
    const double m11 = cp * svd.s1 * ct + (-sp) * svd.s2 * (-st);
    const double m12 = cp * svd.s1 * st + (-sp) * svd.s2 * ct;
    const double m21 = sp * svd.s1 * ct + cp * svd.s2 * (-st);
    const double m22 = sp * svd.s1 * st + cp * svd.s2 * ct;
    Check(Near(m11, f, 1e-12) && Near(m12, g, 1e-12) && Near(m21, 0.0, 1e-12) &&
              Near(m22, h, 1e-12),
          "U * diag(s) * V^T reconstructs the upper-triangular matrix");
    Check(svd.s1 >= svd.s2 && svd.s2 >= 0.0, "the singular values come out ordered and non-negative");
    Check(Near(svd.s1 * svd.s2, std::abs(f * h), 1e-12),
          "the product of the singular values equals |det|");
  }

  // Exact recovery of a line that does not pass through the origin. The fit is exact, so the
  // residual is zero and the coefficients are the closed-form ones.
  {
    LineFitAccumulator accumulator;
    // The line 0.25*x + 0.5*y = 1, i.e. x = 4 - 2y.
    for (int i = 0; i < 9; ++i) {
      const double y = -1.0 + 0.25 * i;
      accumulator.Add(4.0 - 2.0 * y, y);
    }
    const LineFit2D fit = accumulator.Solve();
    Check(fit.rank == 2, "a well-conditioned line fit is full rank");
    Check(Near(fit.a, 0.25, 1e-12) && Near(fit.b, 0.5, 1e-12),
          "the coefficients of an exactly-fittable line are recovered",
          "a=" + std::to_string(fit.a) + " b=" + std::to_string(fit.b));
    Check(fit.algebraic_residual_sq < 1e-24, "the residual of an exact fit is zero",
          std::to_string(fit.algebraic_residual_sq));
    Check(fit.point_count == 9, "the point count is carried");
  }

  // Rank truncation, case by case, on the degenerate corpus.
  {
    int rank_two = 0;
    int rank_one = 0;
    int rank_zero = 0;
    for (const auto& fit_case : detection_scenes::DegenerateFitCases()) {
      LineFitAccumulator accumulator;
      for (const Eigen::Vector2d& point : fit_case.points) accumulator.Add(point.x(), point.y());
      const LineFit2D fit = accumulator.Solve();
      if (fit.rank == 2) ++rank_two;
      if (fit.rank == 1) ++rank_one;
      if (fit.rank == 0) ++rank_zero;
      std::printf("      %-28s n=%2zu rank=%d  cond=%.3e  s=(%.3e, %.3e)\n",
                  fit_case.name.c_str(), fit_case.points.size(), fit.rank,
                  fit.ConditionNumber(), fit.singular_max, fit.singular_min);
    }
    Check(rank_one >= 3, "the exactly-degenerate cases truncate to rank 1",
          std::to_string(rank_one) + " of them");
    Check(rank_zero == 1, "the all-at-origin case truncates to rank 0",
          std::to_string(rank_zero));
    Check(rank_two >= 4, "the well-conditioned cases stay full rank", std::to_string(rank_two));
  }

  // The rank-1 answer is the min-norm one, which for a set of points on a ray is the line
  // perpendicular to that ray. Checking the VALUE, not just the rank, is what proves the
  // truncation branch produces upstream's answer rather than merely refusing to divide by zero.
  {
    const Eigen::Vector2d dir(std::cos(0.7), std::sin(0.7));
    LineFitAccumulator accumulator;
    double sum_r = 0.0;
    double sum_rr = 0.0;
    for (int i = 0; i < 10; ++i) {
      const double r = 1.0 + 0.05 * i;
      accumulator.Add(r * dir.x(), r * dir.y());
      sum_r += r;
      sum_rr += r * r;
    }
    const LineFit2D fit = accumulator.Solve();
    // Min-norm solution of (r_i * dir) . p = 1 is p = dir * (sum r_i) / (sum r_i^2).
    const Eigen::Vector2d expected = dir * (sum_r / sum_rr);
    Check(Near(fit.a, expected.x(), 1e-12) && Near(fit.b, expected.y(), 1e-12),
          "the rank-1 fit returns the minimum-norm solution, not an arbitrary one");
  }

  // Rank 0: no line at all, and D == 0 so the caller keeps the raw endpoints.
  {
    LineFitAccumulator accumulator;
    for (int i = 0; i < 5; ++i) accumulator.Add(0.0, 0.0);
    const LineFit2D fit = accumulator.Solve();
    Check(fit.rank == 0 && fit.D() == 0.0, "points at the origin produce no line and D == 0");
  }
}

// ---------------------------------------------------------------------------------------
// C  Grouping.
// ---------------------------------------------------------------------------------------
void SectionC() {
  Section("C. grouping and occlusion visibility");

  const SegmentCircleDetectorParams params = ShippedParams();
  SegmentCircleDetector detector(params);
  Detection2DResult result;

  // Two well-separated cylinders must be two clusters, and nothing else.
  {
    const auto scene = detection_scenes::TwoAdjacentCylinders();
    Check(detector.Detect(scene.scan, &result) == nullptr, "the two-cylinder scene detects");
    Check(result.clusters.size() == 2, "two separated cylinders give exactly two clusters",
          std::to_string(result.clusters.size()));
    for (const Cluster2D& cluster : result.clusters) CheckValid(cluster, "the cluster is valid");
  }

  // The empty scan must produce nothing, and must not be an error.
  {
    const auto scene = detection_scenes::Empty();
    Check(detector.Detect(scene.scan, &result) == nullptr, "an empty scan is not an error");
    Check(result.clusters.empty() && result.primitives.empty() && result.circles.empty(),
          "an empty scan produces no clusters, primitives or circles");
    Check(result.stats.input_points == 0, "an empty scan reports no input points");
  }

  // min_group_points: a cylinder small enough to give fewer than 5 bins is grouped but never
  // fitted. This is the angular-resolution floor, and it is a property of the shipped
  // configuration rather than of this fixture.
  {
    SegmentCircleDetectorParams strict = params;
    strict.min_group_points = 40;
    SegmentCircleDetector picky(strict);
    const auto scene = detection_scenes::SingleCylinderFullArc();
    Check(picky.Detect(scene.scan, &result) == nullptr, "the picky detector runs");
    Check(result.clusters.size() == 1, "the cluster is still formed");
    Check(result.stats.clusters_below_min_points == 1,
          "a cluster below min_group_points is counted, not fitted");
    Check(result.primitives.empty() && result.circles.empty(),
          "and it produces no primitive and no circle");
  }

  // The occlusion pair. The far cylinder's boundary against the near one is an occlusion edge,
  // so at least one cluster in this scene must report an untrustworthy endpoint - and the
  // detector must have counted it.
  {
    const auto scene = detection_scenes::OcclusionPair();
    Check(detector.Detect(scene.scan, &result) == nullptr, "the occlusion scene detects");
    int occluded = 0;
    for (const Cluster2D& cluster : result.clusters) {
      if (!cluster.is_visible()) ++occluded;
    }
    Check(result.stats.occluded_boundaries >= 1,
          "an occlusion boundary is detected in the occlusion scene",
          std::to_string(result.stats.occluded_boundaries) + " boundaries");
    Check(occluded >= 1, "at least one cluster is flagged as not fully visible",
          std::to_string(occluded) + " of " + std::to_string(result.clusters.size()));

    // And the control: a scene with a single isolated object has no occlusion edge at all.
    const auto isolated = detection_scenes::SingleCylinderFullArc();
    Check(detector.Detect(isolated.scan, &result) == nullptr, "the isolated scene detects");
    Check(result.stats.occluded_boundaries == 0,
          "a scene with one isolated object reports no occlusion boundary",
          std::to_string(result.stats.occluded_boundaries));
  }
}

// ---------------------------------------------------------------------------------------
// D  Split-and-merge.
// ---------------------------------------------------------------------------------------
void SectionD() {
  Section("D. split-and-merge");

  const SegmentCircleDetectorParams params = ShippedParams();
  SegmentCircleDetector detector(params);
  Detection2DResult result;

  // THE SPLIT RULE IS WEAKER THAN ITS NAME SUGGESTS, and this is a measured finding rather than
  // a test convenience. Upstream seeks the divider as the point FARTHEST FROM THE FITTED LINE,
  // then refuses to split unless that point's 1-based index exceeds min_group_points
  // (obstacle_extractor.cpp:248). With `use_split_and_merge` ON - upstream's default - the seed
  // is the least-squares line, and for an L-shaped cluster the least-squares line is tilted so
  // that the cluster's own FIRST point is the largest deviation. split_index comes out as 1, the
  // guard rejects it, and the corner is never split. Turning the flag OFF seeds the search with
  // the endpoint chord instead, whose first point is at distance zero by construction, so the
  // corner wins and the split happens.
  //
  // The consequence is asserted both ways below, because "the corner did not split" read on its
  // own looks like a broken port.
  {
    const auto scene = detection_scenes::WallsOnly();
    Check(detector.Detect(scene.scan, &result) == nullptr, "the walls scene detects");
    const int ls_segments = result.stats.segments_before_merge;

    SegmentCircleDetectorParams iepf = params;
    iepf.use_split_and_merge = false;
    SegmentCircleDetector chord_seeded(iepf);
    Detection2DResult chord_result;
    Check(chord_seeded.Detect(scene.scan, &chord_result) == nullptr,
          "the iterative-end-point-fit path runs on the same scene");

    std::printf("      closed square: least-squares seed -> %d segments (%d splits);"
                " chord seed -> %d segments (%d splits)\n",
                ls_segments, result.stats.splits, chord_result.stats.segments_before_merge,
                chord_result.stats.splits);
    Check(chord_result.stats.splits > result.stats.splits,
          "the chord-seeded search splits the corners that the least-squares seed does not",
          std::to_string(result.stats.splits) + " -> " +
              std::to_string(chord_result.stats.splits) + " splits");
    Check(chord_result.stats.segments_before_merge >= 4,
          "the chord-seeded search separates the four walls",
          std::to_string(chord_result.stats.segments_before_merge));
  }

  // A single straight wall must not be split at all. The scene is one wall only, so any split
  // here is the threshold firing on noise that does not exist.
  {
    const std::vector<detection_scenes::Wall> wall = {{{4.0, -3.0}, {4.0, 3.0}}};
    const auto cast = detection_scenes::CastScene({}, wall, 1.0, 20.0, 20);
    Check(detector.Detect(cast.scan, &result) == nullptr, "the single-wall scene detects");
    Check(result.stats.splits == 0, "a straight wall is never split",
          std::to_string(result.stats.splits) + " splits");
    Check(result.primitives.size() == 1, "a straight wall gives exactly one primitive",
          std::to_string(result.primitives.size()));
    if (result.primitives.size() == 1) {
      const FittedPrimitive2D& primitive = result.primitives.front();
      Check(primitive.kind == PrimitiveKind::kSegment, "and it is a segment, not a circle");
      // The wall is x = 4, so both fitted endpoints must sit on it.
      Check(Near(primitive.first_point.x(), 4.0, 1e-6) && Near(primitive.last_point.x(), 4.0, 1e-6),
            "the fitted segment lies on the wall",
            "x=" + std::to_string(primitive.first_point.x()) + ", " +
                std::to_string(primitive.last_point.x()));
      Check(primitive.fit_residual_m < 1e-6, "the fit residual on an exact wall is zero",
            std::to_string(primitive.fit_residual_m));
      Check(Near(primitive.normal.norm(), 1.0, 1e-12), "the reported normal is a unit vector");
    }
  }

  // A split that DOES fire on the shipped path, so the stage is proved live rather than only
  // proved not to fire. The geometry is a flat wall with a step-in bump in the MIDDLE, ramped
  // over enough bins that the grouping threshold never breaks - so the whole thing is one
  // cluster whose largest deviation from the least-squares line is at its centre, which is the
  // one place the min_group_points guard cannot reject.
  {
    detection_scenes::ScanBuilder builder;
    const double increment = 2.0 * kPi / detection_scenes::kBins;
    for (int bin = 158; bin < 203; ++bin) {
      const double bearing = -kPi + static_cast<double>(bin) * increment;
      const int from_centre = std::abs(bin - 180);
      // 3.0 m wall, a 0.7 m step in over the middle 15 bins, ramped over 6 bins on each side so
      // no adjacent pair ever jumps more than 0.12 m.
      double depth = 3.0;
      if (from_centre <= 7) {
        depth = 2.3;
      } else if (from_centre <= 13) {
        depth = 3.0 - 0.7 * (13.0 - from_centre) / 6.0;
      }
      builder.Set(bin, depth / std::cos(bearing), 0);
    }
    const ProjectedScan bump = builder.Finish(23.0, 23);

    Check(detector.Detect(bump, &result) == nullptr, "the stepped-wall scene detects");
    Check(result.stats.clusters == 1, "the ramp keeps it a single cluster",
          std::to_string(result.stats.clusters));
    Check(result.stats.splits >= 1, "the step in the middle is split",
          std::to_string(result.stats.splits) + " splits");
    Check(result.stats.segments_before_merge >= 2, "and yields at least two segments",
          std::to_string(result.stats.segments_before_merge));
  }
}

// ---------------------------------------------------------------------------------------
// E  Circle from segment.
// ---------------------------------------------------------------------------------------
void SectionE() {
  Section("E. circle from segment");

  const SegmentCircleDetectorParams params = ShippedParams();
  Detection2DResult result;

  // The sqrt(3)/3 rule, checked against a hand-computed answer. The scene is one cylinder, and
  // the emitted circle's radius must equal 0.5773502 * (fitted segment length) + enlargement.
  {
    SegmentCircleDetector detector(params);
    const auto scene = detection_scenes::SingleCylinderFullArc();
    Check(detector.Detect(scene.scan, &result) == nullptr, "the single-cylinder scene detects");
    Check(result.circles.size() == 1, "one cylinder gives one circle",
          std::to_string(result.circles.size()));
    if (result.circles.size() == 1) {
      const CircleObservation& circle = result.circles.front();
      CheckValid(circle, "the circle observation is valid");

      // The circle's own primitive carries the same numbers - that indirection has to be right
      // or every downstream consumer reads the wrong primitive.
      Check(circle.primitive_index >= 0 &&
                circle.primitive_index < static_cast<int>(result.primitives.size()),
            "primitive_index addresses a real primitive");
      if (circle.primitive_index >= 0 &&
          circle.primitive_index < static_cast<int>(result.primitives.size())) {
        const FittedPrimitive2D& primitive =
            result.primitives[static_cast<std::size_t>(circle.primitive_index)];
        Check(primitive.kind == PrimitiveKind::kCircle, "and it is the kCircle primitive");
        Check(Near(primitive.radius_m, circle.radius_fitted_m, 1e-15) &&
                  (primitive.center - circle.center).norm() < 1e-15,
              "the primitive and the observation agree on centre and radius");
      }

      // The inflated / true distinction, which is upstream's CircleObstacle{radius, true_radius}.
      const double true_radius = circle.radius_fitted_m - params.radius_enlargement_m;
      Check(true_radius > 0.0, "the enlargement is recoverable, so the true radius survives",
            "true=" + std::to_string(true_radius) +
                " inflated=" + std::to_string(circle.radius_fitted_m));

      // The enclosing radius is a genuinely different number, and it must not be smaller than
      // it would be if it were merely a copy of the fitted one.
      Check(circle.radius_enclosing_m > 0.0 &&
                circle.radius_enclosing_m != circle.radius_fitted_m,
            "the enclosing radius is a distinct measurement, not a copy",
            "enclosing=" + std::to_string(circle.radius_enclosing_m));

      // Every contributing point must actually be inside the enclosing circle - that is its
      // definition, and it is the property the safety stage may end up relying on.
      double worst = 0.0;
      for (const Eigen::Vector2d& point : detector.points()) {
        const double distance = (point - circle.center).norm();
        if (distance <= circle.radius_enclosing_m + 1e-12) continue;
        worst = std::max(worst, distance - circle.radius_enclosing_m);
      }
      Check(worst == 0.0, "every contributing point lies inside the enclosing radius",
            std::to_string(worst) + " m outside");

      // The two arc definitions must not have been confused: the sensor-referenced extent is
      // far smaller than the centre-referenced one for an object at range.
      const double centre_referenced = circle.arc_length_m / circle.radius_fitted_m;
      Check(circle.arc_extent_rad < centre_referenced,
            "the sensor-referenced arc extent is smaller than the centre-referenced one",
            "sensor=" + std::to_string(circle.arc_extent_rad) +
                " centre=" + std::to_string(centre_referenced));
      Check(Near(circle.range_to_center_m, circle.center.norm(), 1e-12),
            "range_to_center_m is the range to the reported centre");
      Check(circle.sigma_center_m > 0.0 && circle.sigma_radius_m > 0.0,
            "the measurement sigmas are positive");
    }
  }

  // max_circle_radius: with the cap just below what this cylinder produces, the circle is
  // rejected and the segment survives instead.
  {
    SegmentCircleDetectorParams capped = params;
    capped.radius_enlargement_m = 0.0;
    capped.max_circle_radius_m = 0.05;  // Far below 0.577 * chord.
    SegmentCircleDetector detector(capped);
    const auto scene = detection_scenes::SingleCylinderFullArc();
    Check(detector.Detect(scene.scan, &result) == nullptr, "the capped detector runs");
    Check(result.circles.empty(), "a radius above the cap yields no circle");
    Check(result.stats.circles_rejected_radius == 1, "and the rejection is counted");
    Check(result.primitives.size() == 1,
          "the segment survives, because discard only applies to converted ones");
  }

  // radius_enlargement is additive and nothing else: doubling it must move the radius by
  // exactly that much and leave the centre alone.
  {
    SegmentCircleDetector plain(params);
    const auto scene = detection_scenes::SingleCylinderFullArc();
    plain.Detect(scene.scan, &result);
    const CircleObservation base = result.circles.front();

    SegmentCircleDetectorParams enlarged = params;
    enlarged.radius_enlargement_m = params.radius_enlargement_m + 0.1;
    SegmentCircleDetector bigger(enlarged);
    bigger.Detect(scene.scan, &result);
    const CircleObservation grown = result.circles.front();

    Check(Near(grown.radius_fitted_m - base.radius_fitted_m, 0.1, 1e-12),
          "the enlargement is purely additive on the radius");
    Check((grown.center - base.center).norm() < 1e-15,
          "and it does not move the centre");
  }

  // circles_from_visibles: with it on, a cluster with an occluded boundary contributes no
  // circle; with it off, it does. Anything else means the flag is not wired to the decision.
  {
    const auto scene = detection_scenes::OcclusionPair();

    SegmentCircleDetector guarded(params);
    guarded.Detect(scene.scan, &result);
    const std::size_t guarded_circles = result.circles.size();
    const int rejected = result.stats.circles_rejected_visibility;

    SegmentCircleDetectorParams permissive = params;
    permissive.circles_from_visibles = false;
    SegmentCircleDetector open(permissive);
    open.Detect(scene.scan, &result);
    const std::size_t open_circles = result.circles.size();

    Check(rejected >= 1, "circles_from_visibles rejects at least one occluded segment",
          std::to_string(rejected));
    Check(open_circles > guarded_circles,
          "and turning it off admits more circles",
          std::to_string(guarded_circles) + " -> " + std::to_string(open_circles));
  }
}

// ---------------------------------------------------------------------------------------
// F  Merging.
// ---------------------------------------------------------------------------------------
void SectionF() {
  Section("F. segment and circle merging");

  const SegmentCircleDetectorParams params = ShippedParams();
  Detection2DResult result;

  // A wall interrupted by a gap smaller than max_merge_separation: grouping breaks it into two
  // clusters, and merging must put it back together. That is the whole purpose of the stage.
  {
    // THE HOLE HAS TO LAND BETWEEN TWO THRESHOLDS, which is why it is specified in bins rather
    // than in metres. It must be WIDER than the grouping threshold (0.1 + range * proportion,
    // about 0.126 m here) so the wall breaks into two clusters at all, and NARROWER than
    // max_merge_separation (0.20 m) so the merge stage can put it back. At 1.5 m the bin pitch
    // is 0.026 m, so dropping the five central bins leaves a 0.157 m gap - inside the window
    // with room on both sides. A wall at 3 m would have had no such window: its bin pitch alone
    // is 0.052 m and the smallest breakable gap already exceeds the merge threshold.
    detection_scenes::ScanBuilder builder;
    const double increment = 2.0 * kPi / detection_scenes::kBins;
    const detection_scenes::Wall wall{{1.5, -1.5}, {1.5, 1.5}};
    for (int bin = 0; bin < detection_scenes::kBins; ++bin) {
      if (bin >= 178 && bin <= 182) continue;  // Bin 180 is bearing 0 exactly.
      const double bearing = -kPi + static_cast<double>(bin) * increment;
      const double range = detection_scenes::RayWall(bearing, wall);
      if (range <= 0.0) continue;
      builder.Set(bin, range, 0);
    }
    const ProjectedScan scan = builder.Finish(21.0, 21);

    SegmentCircleDetectorParams no_merge = params;
    no_merge.max_merge_separation_m = 0.001;  // Too tight to merge anything.
    SegmentCircleDetector unmerged(no_merge);
    unmerged.Detect(scan, &result);
    const int before = result.stats.segments_before_merge;
    const int after_without = result.stats.segments_after_merge;

    SegmentCircleDetector merged(params);
    merged.Detect(scan, &result);
    const int after_with = result.stats.segments_after_merge;

    Check(before >= 2, "the interrupted wall starts as at least two segments",
          std::to_string(before));
    Check(after_without == before, "a tight separation threshold merges nothing",
          std::to_string(after_without));
    Check(after_with < before, "the shipped threshold merges the two halves back together",
          std::to_string(before) + " -> " + std::to_string(after_with));
    Check(result.stats.segment_merges >= 1, "and the merge is counted",
          std::to_string(result.stats.segment_merges));
  }

  // Circle merging. Two cylinders close enough that their enlarged circles contain one another
  // must come out as one, and the merge must be counted.
  //
  // A FINDING FROM THE ENLARGEMENT RE-DERIVATION, RECORDED BECAUSE IT CHANGED THIS CHECK'S
  // ANSWER. At upstream's 0.25 m enlargement this scene produced ZERO circles: the pair merges
  // into one circle whose enlarged radius exceeded `max_circle_radius_m` (0.60 m), so the radius
  // cap dropped it and `circles.size() <= 1` passed vacuously - two real obstacles reported as
  // nothing at all. At the re-derived 0.17 m the same merged circle is 0.08 m smaller, clears the
  // cap, and is emitted. The enlargement is added BEFORE the cap is applied
  // (segment_circle_detector.cpp), so shrinking it widens what the detector will report, and this
  // is the one place in the corpus where that crosses a threshold.
  //
  // The check therefore asserts the count from BOTH sides now. "At most one" was the original
  // question and is still the merging property under test; "at least one" is what stops the test
  // silently going green again if a future change pushes this scene back over the cap.
  {
    const std::vector<detection_scenes::Cylinder> pair = {{2.0, -0.12, 0.25}, {2.0, 0.12, 0.25}};
    const auto cast = detection_scenes::CastScene(pair, {}, 1.0, 22.0, 22);
    SegmentCircleDetector detector(params);
    detector.Detect(cast.scan, &result);
    std::printf("      two overlapping 0.25 m cylinders at 2 m: %d circles from %d attempted"
                " (%d rejected by the %.2f m radius cap at a %.2f m enlargement), %d merges\n",
                static_cast<int>(result.circles.size()), result.stats.circles_attempted,
                result.stats.circles_rejected_radius, params.max_circle_radius_m,
                params.radius_enlargement_m, result.stats.circle_merges);
    Check(result.circles.size() <= 1,
          "two heavily overlapping cylinders do not produce two separate circles",
          std::to_string(result.circles.size()) + " circles, " +
              std::to_string(result.stats.circle_merges) + " merges");
    Check(!result.circles.empty(),
          "and they are not dropped ENTIRELY - the merged circle clears the radius cap at the "
          "re-derived enlargement, where at upstream's 0.25 m it did not and this scene reported "
          "no obstacle at all",
          std::to_string(result.circles.size()) + " circles");
    // THE MECHANISM, VERIFIED RATHER THAN ASSERTED. The claim above is that the 0.08 m the
    // enlargement gave up is exactly what moved this circle from the rejected side of the cap to
    // the accepted side. That is checkable without re-running the detector: re-add the difference
    // and the radius must land at or above the cap.
    if (!result.circles.empty()) {
      const double radius = result.circles.front().radius_fitted_m;
      const double at_upstream_enlargement = radius + (0.25 - params.radius_enlargement_m);
      Check(radius < params.max_circle_radius_m &&
                at_upstream_enlargement >= params.max_circle_radius_m,
            "and the mechanism is the radius cap and nothing else: this circle sits below the cap "
            "at the shipped enlargement and above it at upstream's 0.25 m",
            std::to_string(radius) + " m < " + std::to_string(params.max_circle_radius_m) +
                " m, but " + std::to_string(at_upstream_enlargement) + " m >= it");
    }
    for (const CircleObservation& circle : result.circles) {
      CheckValid(circle, "the surviving circle is valid");
    }
  }
}

// ---------------------------------------------------------------------------------------
// G  Contract validity and per-frame statelessness.
// ---------------------------------------------------------------------------------------
void SectionG() {
  Section("G. contract validity and per-frame statelessness");

  const SegmentCircleDetectorParams params = ShippedParams();
  SegmentCircleDetector detector(params);
  Detection2DResult result;

  int clusters = 0;
  int primitives = 0;
  int circles = 0;
  int invalid = 0;
  for (const auto& scene : detection_scenes::AllScenes()) {
    if (detector.Detect(scene.scan, &result) != nullptr) {
      ++invalid;
      continue;
    }
    for (const Cluster2D& cluster : result.clusters) {
      ++clusters;
      if (!cluster.IsValid()) ++invalid;
    }
    for (const FittedPrimitive2D& primitive : result.primitives) {
      ++primitives;
      if (!primitive.IsValid()) ++invalid;
    }
    for (const CircleObservation& circle : result.circles) {
      ++circles;
      if (!circle.IsValid()) ++invalid;
    }
  }
  Check(invalid == 0, "every contract the corpus produces is valid",
        std::to_string(clusters) + " clusters, " + std::to_string(primitives) + " primitives, " +
            std::to_string(circles) + " circles");

  // Statelessness. Interleaving a different scan must not change the answer to the first one -
  // which a detector holding cross-frame state would fail even though repeating one scan twice
  // would not catch it.
  {
    const auto first = detection_scenes::ArenaFullArc();
    const auto other = detection_scenes::WallsOnly();

    Detection2DResult reference;
    detector.Detect(first.scan, &reference);

    Detection2DResult interleaved;
    detector.Detect(other.scan, &interleaved);
    detector.Detect(first.scan, &interleaved);

    bool identical = reference.clusters.size() == interleaved.clusters.size() &&
                     reference.primitives.size() == interleaved.primitives.size() &&
                     reference.circles.size() == interleaved.circles.size();
    if (identical) {
      for (std::size_t i = 0; i < reference.circles.size(); ++i) {
        if ((reference.circles[i].center - interleaved.circles[i].center).norm() != 0.0 ||
            reference.circles[i].radius_fitted_m != interleaved.circles[i].radius_fitted_m) {
          identical = false;
          break;
        }
      }
    }
    Check(identical, "a scan interleaved between two identical scans changes nothing");

    // And a fresh detector must agree with a used one, bit for bit.
    SegmentCircleDetector fresh(params);
    Detection2DResult clean;
    fresh.Detect(first.scan, &clean);
    bool same_as_fresh = clean.circles.size() == reference.circles.size();
    for (std::size_t i = 0; same_as_fresh && i < clean.circles.size(); ++i) {
      same_as_fresh = (clean.circles[i].center - reference.circles[i].center).norm() == 0.0 &&
                      clean.circles[i].radius_fitted_m == reference.circles[i].radius_fitted_m;
    }
    Check(same_as_fresh, "a freshly constructed detector produces the identical result");
  }
}

// ---------------------------------------------------------------------------------------
// H  Cluster metrics.
//
// DEFINITIONS, stated here because none of these terms has one universal meaning:
//   * A GROUND-TRUTH cluster is a maximal run of consecutive points in the detector's ordered
//     point list carrying the same object label. Walls share one label, so a corner is not a
//     ground-truth boundary - the grouping stage has no way to see one and should not be
//     charged for it.
//   * PURITY of a detected cluster is the fraction of its points from its majority label.
//   * A detected cluster is CORRECT when its purity is >= 0.95 and it covers >= 0.95 of the
//     ground-truth cluster it overlaps most.
//   * PRECISION is correct / detected. RECALL is (ground-truth clusters with at least one
//     correct detection) / ground-truth.
//   * OVER-SEGMENTATION is the fraction of ground-truth clusters overlapped by two or more
//     detected clusters; UNDER-SEGMENTATION is the fraction of detected clusters drawing from
//     two or more ground-truth clusters. Both count only contributions of >= 2 points, so a
//     single stray point is not reported as a structural failure.
// ---------------------------------------------------------------------------------------
struct ClusterMetrics {
  int ground_truth = 0;
  int detected = 0;
  int correct = 0;
  int recalled = 0;
  int over = 0;
  int under = 0;
  double purity_sum = 0.0;

  double Precision() const { return detected > 0 ? static_cast<double>(correct) / detected : 1.0; }
  double Recall() const {
    return ground_truth > 0 ? static_cast<double>(recalled) / ground_truth : 1.0;
  }
  double MeanPurity() const { return detected > 0 ? purity_sum / detected : 1.0; }
  double OverRate() const {
    return ground_truth > 0 ? static_cast<double>(over) / ground_truth : 0.0;
  }
  double UnderRate() const { return detected > 0 ? static_cast<double>(under) / detected : 0.0; }
};

// The label of each point in the detector's ordered list, recovered from the scene's per-bin
// provenance through the detector's own bin mapping.
std::vector<int> PointLabels(const detection_scenes::Scene& scene,
                             const SegmentCircleDetector& detector) {
  std::vector<int> labels;
  labels.reserve(detector.point_bins().size());
  for (const std::int32_t bin : detector.point_bins()) {
    labels.push_back(scene.bin_labels[static_cast<std::size_t>(bin)]);
  }
  return labels;
}

void AccumulateClusterMetrics(const detection_scenes::Scene& scene,
                              const SegmentCircleDetector& detector,
                              const Detection2DResult& result, ClusterMetrics* metrics) {
  const std::vector<int> labels = PointLabels(scene, detector);
  if (labels.empty()) return;

  // Ground-truth clusters as [first, last] runs of one label.
  struct Run {
    int label;
    int first;
    int last;
  };
  std::vector<Run> truth;
  for (int i = 0; i < static_cast<int>(labels.size()); ++i) {
    if (!truth.empty() && truth.back().label == labels[static_cast<std::size_t>(i)] &&
        truth.back().last == i - 1) {
      truth.back().last = i;
    } else {
      truth.push_back({labels[static_cast<std::size_t>(i)], i, i});
    }
  }

  metrics->ground_truth += static_cast<int>(truth.size());
  metrics->detected += static_cast<int>(result.clusters.size());

  std::vector<int> covering(truth.size(), 0);
  std::vector<bool> recalled(truth.size(), false);

  for (const Cluster2D& cluster : result.clusters) {
    std::map<int, int> histogram;
    for (int i = cluster.first_index; i <= cluster.last_index; ++i) {
      ++histogram[labels[static_cast<std::size_t>(i)]];
    }
    int majority_label = -1;
    int majority_count = 0;
    int contributing_truths = 0;
    for (const auto& entry : histogram) {
      if (entry.second >= 2) ++contributing_truths;
      if (entry.second > majority_count) {
        majority_count = entry.second;
        majority_label = entry.first;
      }
    }
    const double purity =
        cluster.point_count > 0 ? static_cast<double>(majority_count) / cluster.point_count : 0.0;
    metrics->purity_sum += purity;
    if (contributing_truths >= 2) ++metrics->under;

    // The ground-truth run this cluster overlaps most.
    int best = -1;
    int best_overlap = 0;
    for (std::size_t t = 0; t < truth.size(); ++t) {
      if (truth[t].label != majority_label) continue;
      const int overlap = std::min(cluster.last_index, truth[t].last) -
                          std::max(cluster.first_index, truth[t].first) + 1;
      if (overlap > best_overlap) {
        best_overlap = overlap;
        best = static_cast<int>(t);
      }
    }
    if (best < 0) continue;
    if (best_overlap >= 2) ++covering[static_cast<std::size_t>(best)];

    const int truth_size = truth[static_cast<std::size_t>(best)].last -
                           truth[static_cast<std::size_t>(best)].first + 1;
    const double coverage = static_cast<double>(best_overlap) / truth_size;
    if (purity >= 0.95 && coverage >= 0.95) {
      ++metrics->correct;
      recalled[static_cast<std::size_t>(best)] = true;
    }
  }

  for (std::size_t t = 0; t < truth.size(); ++t) {
    if (recalled[t]) ++metrics->recalled;
    if (covering[t] >= 2) ++metrics->over;
  }
}

void SectionH() {
  Section("H. cluster precision, recall, purity and segmentation rates");

  const SegmentCircleDetectorParams params = ShippedParams();
  SegmentCircleDetector detector(params);
  Detection2DResult result;
  ClusterMetrics metrics;

  for (const auto& scene : detection_scenes::AllScenes()) {
    if (detector.Detect(scene.scan, &result) != nullptr) continue;
    AccumulateClusterMetrics(scene, detector, result, &metrics);
  }

  std::printf("      ground-truth clusters %d, detected %d, correct %d\n", metrics.ground_truth,
              metrics.detected, metrics.correct);
  std::printf("      over-segmentation %.3f, under-segmentation %.3f\n", metrics.OverRate(),
              metrics.UnderRate());

  Check(metrics.Precision() >= 0.95, "cluster precision >= 0.95",
        std::to_string(metrics.Precision()));
  Check(metrics.Recall() >= 0.95, "cluster recall >= 0.95", std::to_string(metrics.Recall()));
  Check(metrics.MeanPurity() >= 0.95, "mean cluster purity >= 0.95",
        std::to_string(metrics.MeanPurity()));
}

// ---------------------------------------------------------------------------------------
// I  Circle-fitting accuracy - the short-arc experiment.
//
// Reported as two populations, NEVER averaged together. The whole point of the measurement is
// that the sqrt(3)/3 rule's error is a function of how much of the arc was seen, so a single
// pooled number would hide exactly the effect the later phases need.
//
// "True radius" throughout means radius_fitted_m - radius_enlargement_m, which is upstream's
// CircleObstacle::true_radius and the number a radius-accuracy claim has to be about; comparing
// the inflated radius against a ground-truth radius would report the enlargement as an error.
// ---------------------------------------------------------------------------------------
// One ground-truth cylinder's worth of measurement.
struct CircleCase {
  std::string scene;
  int index = 0;
  double truth_radius = 0.0;
  double truth_range = 0.0;
  int visible_bins = 0;

  // The cylinder's bins reach BOTH ends of the scan, i.e. it straddles the +/-pi seam directly
  // behind the robot. The ordered point list is linear in bin index and does not wrap, so such an
  // object arrives as two clusters that each may fall under min_group_points. Classified
  // separately from a genuine miss because the cause is the scan's topology, not the estimator's
  // accuracy - see the seam check in section I.
  bool straddles_seam = false;

  bool matched = false;
  double center_error = 0.0;
  double center_signed = 0.0;  // Positive = pushed AWAY from the sensor.
  double radius_error = 0.0;   // Signed: negative is an UNDER-estimate.
  double range_error = 0.0;
  double centre_arc_rad = 0.0;

  // The same cylinder's points through an ALGEBRAIC circle fit, for comparison. See
  // AlgebraicCircleFit below for why this is measured at all.
  bool algebraic_ok = false;
  double algebraic_center_error = 0.0;
  double algebraic_radius_error = 0.0;
};

// The algebraic circle fit - upstream's OWN `fitCircle` (figure_fitting.h:175-201), which the
// extractor never calls. It solves a1*x + a2*y + a3 = -(x^2 + y^2) in the least-squares sense
// and reads the centre and radius off the coefficients.
//
// WHY IT IS MEASURED HERE. The sqrt(3)/3 rule is not a circle fit at all: it is the circumcircle
// of an equilateral triangle standing on the chord, so its centre sits behind the arc BY
// CONSTRUCTION and its radius is a function of chord length rather than of curvature. If the
// section-12 centre-error target is missed, the useful question is not "by how much" but "is it
// the estimator or the data", and the only way to separate those is to run a real fit on the
// same points. This is measurement only - no production code is added, and choosing a fitting
// backend is explicitly a later decision.
struct AlgebraicCircle {
  bool ok = false;
  Eigen::Vector2d center{0.0, 0.0};
  double radius = 0.0;
};

AlgebraicCircle AlgebraicCircleFit(const std::vector<Eigen::Vector2d>& points) {
  AlgebraicCircle out;
  if (points.size() < 3) return out;

  Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  for (const Eigen::Vector2d& point : points) {
    Eigen::Vector3d row(point.x(), point.y(), 1.0);
    normal += row * row.transpose();
    rhs += row * -(point.squaredNorm());
  }
  const Eigen::Vector3d parameters = normal.colPivHouseholderQr().solve(rhs);
  const double squared = 0.25 * (parameters(0) * parameters(0) + parameters(1) * parameters(1)) -
                         parameters(2);
  if (!(squared > 0.0)) return out;
  out.ok = true;
  out.center = Eigen::Vector2d(-0.5 * parameters(0), -0.5 * parameters(1));
  out.radius = std::sqrt(squared);
  return out;
}

// Collects one case per ground-truth cylinder. Cylinders below the angular-resolution floor are
// still recorded, with matched = false and visible_bins < min_group_points, so the report can
// distinguish "the estimator missed it" from "the scan never had enough of it to find".
void CollectCircleCases(const detection_scenes::Scene& scene, const SegmentCircleDetector& detector,
                        const Detection2DResult& result, double enlargement,
                        std::vector<CircleCase>* cases) {
  const std::vector<int> labels = PointLabels(scene, detector);

  for (std::size_t c = 0; c < scene.cylinders.size(); ++c) {
    const auto& cylinder = scene.cylinders[c];
    const Eigen::Vector2d truth(cylinder.x, cylinder.y);

    CircleCase entry;
    entry.scene = scene.name;
    entry.index = static_cast<int>(c);
    entry.truth_radius = cylinder.radius;
    entry.truth_range = truth.norm();

    std::vector<Eigen::Vector2d> owned;
    bool at_low_end = false;
    bool at_high_end = false;
    for (std::size_t i = 0; i < labels.size(); ++i) {
      if (labels[i] != static_cast<int>(c)) continue;
      ++entry.visible_bins;
      owned.push_back(detector.points()[i]);
      const std::int32_t bin = detector.point_bins()[i];
      if (bin == 0) at_low_end = true;
      if (bin == scene.scan.bins() - 1) at_high_end = true;
    }
    entry.straddles_seam = at_low_end && at_high_end;

    const AlgebraicCircle algebraic = AlgebraicCircleFit(owned);
    if (algebraic.ok) {
      entry.algebraic_ok = true;
      entry.algebraic_center_error = (algebraic.center - truth).norm();
      entry.algebraic_radius_error = algebraic.radius - cylinder.radius;
    }

    // Association: the nearest circle centre within a generous gate. A tight gate would make the
    // association itself part of the measurement.
    const CircleObservation* best = nullptr;
    double best_distance = 1.0;
    for (const CircleObservation& circle : result.circles) {
      const double distance = (circle.center - truth).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best = &circle;
      }
    }
    if (best != nullptr) {
      entry.matched = true;
      entry.center_error = (best->center - truth).norm();
      entry.center_signed = (best->center - truth).dot(truth.normalized());
      entry.radius_error = (best->radius_fitted_m - enlargement) - cylinder.radius;
      entry.range_error = std::abs(best->range_to_center_m - entry.truth_range);
      entry.centre_arc_rad = best->arc_length_m / std::max(best->radius_fitted_m, 1e-9);
    }
    cases->push_back(entry);
  }
}

// The aggregate of one population - full-arc or half-arc, never the two pooled.
struct FitSummary {
  int detectable = 0;
  int below_floor = 0;
  int seam_split = 0;
  int matched = 0;
  int missed = 0;
  double worst_center = 0.0;
  double worst_radius = 0.0;
  double worst_range = 0.0;
  double worst_algebraic_center = 0.0;
  double worst_algebraic_radius = 0.0;
  double center_bias_sum = 0.0;
  double radius_bias_sum = 0.0;
  double arc_sum = 0.0;
  std::string worst_center_case;
  std::string worst_radius_case;

  double FailureRate() const {
    return detectable > 0 ? static_cast<double>(missed) / detectable : 0.0;
  }
  double CenterBias() const { return matched > 0 ? center_bias_sum / matched : 0.0; }
  double RadiusBias() const { return matched > 0 ? radius_bias_sum / matched : 0.0; }
  double MeanArc() const { return matched > 0 ? arc_sum / matched : 0.0; }
};

FitSummary Summarise(const std::vector<CircleCase>& cases, int min_group_points) {
  FitSummary summary;
  for (const CircleCase& entry : cases) {
    if (entry.straddles_seam) {
      ++summary.seam_split;
      continue;
    }
    if (entry.visible_bins < min_group_points) {
      ++summary.below_floor;
      continue;
    }
    ++summary.detectable;
    if (!entry.matched) {
      ++summary.missed;
      continue;
    }
    ++summary.matched;
    if (entry.center_error > summary.worst_center) {
      summary.worst_center = entry.center_error;
      summary.worst_center_case = entry.scene + "#" + std::to_string(entry.index);
    }
    if (std::abs(entry.radius_error) > summary.worst_radius) {
      summary.worst_radius = std::abs(entry.radius_error);
      summary.worst_radius_case = entry.scene + "#" + std::to_string(entry.index);
    }
    summary.worst_range = std::max(summary.worst_range, entry.range_error);
    summary.center_bias_sum += entry.center_signed;
    summary.radius_bias_sum += entry.radius_error;
    summary.arc_sum += entry.centre_arc_rad;
    if (entry.algebraic_ok) {
      summary.worst_algebraic_center =
          std::max(summary.worst_algebraic_center, entry.algebraic_center_error);
      summary.worst_algebraic_radius =
          std::max(summary.worst_algebraic_radius, std::abs(entry.algebraic_radius_error));
    }
  }
  return summary;
}

void PrintCases(const char* label, const std::vector<CircleCase>& cases, int min_group_points) {
  std::printf("\n      %s cases (r = true radius, d = true range, bins = visible bins;"
              " * = below the angular floor, ~ = straddles the +/-pi seam)\n", label);
  std::printf("      %-26s %5s %5s %4s %8s %8s %8s %6s | %8s %8s\n", "scene#cyl", "r", "d",
              "bins", "dCentre", "dRadius", "dRange", "arc", "algCtr", "algRad");
  for (const CircleCase& entry : cases) {
    const bool floored = entry.visible_bins < min_group_points;
    const std::string name = entry.scene + "#" + std::to_string(entry.index) +
                             (entry.straddles_seam ? "~" : (floored ? "*" : ""));
    if (!entry.matched) {
      std::printf("      %-26s %5.2f %5.2f %4d %8s\n", name.c_str(), entry.truth_radius,
                  entry.truth_range, entry.visible_bins,
                  entry.straddles_seam ? "(seam)" : (floored ? "(floor)" : "MISSED"));
      continue;
    }
    std::printf("      %-26s %5.2f %5.2f %4d %+8.4f %+8.4f %8.4f %6.3f | %8.4f %+8.4f\n",
                name.c_str(), entry.truth_radius, entry.truth_range, entry.visible_bins,
                entry.center_signed, entry.radius_error, entry.range_error, entry.centre_arc_rad,
                entry.algebraic_center_error, entry.algebraic_radius_error);
  }
}

// The section-12 gate, evaluated and REPORTED rather than silently relaxed. Section 12 marks its
// numbers "[RECOMMENDATION, initial; refined by P4-P9 evidence]", and this phase is the evidence;
// where the ported estimator provably cannot meet a target, the honest outcome is a recorded
// finding plus a regression bound on what it DOES achieve - not a quietly widened threshold and
// not a red build that hides the measurement.
struct GateOutcome {
  bool met = false;
};

GateOutcome ReportGate(const char* label, const char* metric, double measured, double target,
                       double regression_bound) {
  GateOutcome outcome;
  outcome.met = measured <= target;
  std::printf("      section-12 gate | %-9s %-22s target %.3f, measured %.4f -> %s\n", label,
              metric, target, measured, outcome.met ? "MET" : "NOT MET");
  Check(measured <= regression_bound,
        std::string(label) + ": " + metric + " stays within the measured envelope " +
            std::to_string(regression_bound) + " m",
        std::to_string(measured));
  return outcome;
}

void SectionI() {
  Section("I. circle-fitting accuracy, full arc vs half arc");

  const SegmentCircleDetectorParams params = ShippedParams();
  SegmentCircleDetector detector(params);
  Detection2DResult result;

  std::vector<CircleCase> full;
  for (const auto& scene : detection_scenes::FullArcScenes()) {
    if (detector.Detect(scene.scan, &result) != nullptr) continue;
    CollectCircleCases(scene, detector, result, params.radius_enlargement_m, &full);
  }
  std::vector<CircleCase> half;
  for (const auto& scene : detection_scenes::HalfArcScenes()) {
    if (detector.Detect(scene.scan, &result) != nullptr) continue;
    CollectCircleCases(scene, detector, result, params.radius_enlargement_m, &half);
  }

  PrintCases("FULL-ARC", full, params.min_group_points);
  PrintCases("HALF-ARC", half, params.min_group_points);

  const FitSummary full_summary = Summarise(full, params.min_group_points);
  const FitSummary half_summary = Summarise(half, params.min_group_points);

  std::printf("\n      full-arc: %d detectable, %d below the angular floor, %d seam-split,"
              " %d matched, %d missed\n",
              full_summary.detectable, full_summary.below_floor, full_summary.seam_split,
              full_summary.matched, full_summary.missed);
  std::printf("      full-arc: centre bias %+.4f m, radius bias %+.4f m, mean centre arc %.3f rad\n",
              full_summary.CenterBias(), full_summary.RadiusBias(), full_summary.MeanArc());
  std::printf("      half-arc: %d detectable, %d below the angular floor, %d seam-split,"
              " %d matched, %d missed\n",
              half_summary.detectable, half_summary.below_floor, half_summary.seam_split,
              half_summary.matched, half_summary.missed);
  std::printf("      half-arc: centre bias %+.4f m, radius bias %+.4f m, mean centre arc %.3f rad\n",
              half_summary.CenterBias(), half_summary.RadiusBias(), half_summary.MeanArc());

  // The gates, stated separately per population - the split IS the measurement, so pooling the
  // two would destroy exactly the result the later phases are waiting for.
  //
  // The two centre gates and the full-arc radius gate are NOT MET by the ported sqrt(3)/3 rule.
  // The regression bounds below are the measured envelopes, rounded up; they exist so a
  // REGRESSION in the port is still a red test even though the target itself is unreachable.
  const GateOutcome full_center = ReportGate("full-arc", "centre error <= 0.05 m",
                                             full_summary.worst_center, 0.05, 0.12);
  const GateOutcome full_radius = ReportGate("full-arc", "true-radius error <= 0.05 m",
                                             full_summary.worst_radius, 0.05, 0.06);
  const GateOutcome full_range = ReportGate("full-arc", "distance-to-centre <= 0.05 m",
                                            full_summary.worst_range, 0.05, 0.12);
  const GateOutcome half_center = ReportGate("half-arc", "centre error <= 0.15 m",
                                             half_summary.worst_center, 0.15, 0.25);
  const GateOutcome half_radius = ReportGate("half-arc", "true-radius error <= 0.10 m",
                                             half_summary.worst_radius, 0.10, 0.20);
  const GateOutcome half_range = ReportGate("half-arc", "distance-to-centre <= 0.05 m",
                                            half_summary.worst_range, 0.05, 0.25);
  (void)full_range;
  (void)half_range;

  Check(full_summary.FailureRate() <= 0.01, "full-arc: failure rate <= 1% of detectable cylinders",
        std::to_string(full_summary.FailureRate()));
  Check(half_summary.FailureRate() <= 0.01, "half-arc: failure rate <= 1% of detectable cylinders",
        std::to_string(half_summary.FailureRate()));

  // The centre gates are expected to be unmet. Asserting the EXPECTATION is what keeps the
  // finding honest: if a later change made them pass, this check fails and the report that says
  // "the sqrt(3)/3 rule cannot meet the centre target" has to be revisited rather than left to
  // rot as a stale conclusion.
  Check(!full_center.met && !half_center.met,
        "the recorded finding still holds: the sqrt(3)/3 centre gates are NOT met");
  Check(full_radius.met && !half_radius.met,
        "the recorded finding still holds: the full-arc radius gate is met, the half-arc one is not");

  // THE SHORT-ARC BIAS ITSELF - the measurement R8 and the radius-policy question are waiting on.
  Check(half_summary.RadiusBias() < full_summary.RadiusBias(),
        "the radius bias is more negative on short arcs than on full ones",
        "full " + std::to_string(full_summary.RadiusBias()) + " m vs half " +
            std::to_string(half_summary.RadiusBias()) + " m");
  Check(half_summary.RadiusBias() < 0.0,
        "and the half-arc bias UNDER-estimates the radius - the unrecoverable direction",
        std::to_string(half_summary.RadiusBias()) + " m");

  // The estimator, not the data: an algebraic circle fit on the SAME points. If this meets the
  // centre gate that the sqrt(3)/3 rule misses, the gap is a property of the fitting rule and a
  // later phase can close it by changing the backend rather than by loosening the target.
  // NOTE ON WHAT THIS DOES AND DOES NOT SHOW. These scenes are noise-free ray casts, so the
  // points lie exactly on the true circle and an algebraic fit recovers it to machine precision -
  // even from five points. That is not a claim that an algebraic fit would be this good on real
  // data; it is the stronger and narrower claim that ALL of the sqrt(3)/3 rule's error here is
  // the fitting rule, none of it the sampling.
  std::printf("\n      algebraic circle fit on the same points, worst case:"
              " full-arc centre %.4f m radius %.4f m; half-arc centre %.4f m radius %.4f m\n",
              full_summary.worst_algebraic_center, full_summary.worst_algebraic_radius,
              half_summary.worst_algebraic_center, half_summary.worst_algebraic_radius);
  Check(full_summary.worst_algebraic_center < full_summary.worst_center,
        "an algebraic circle fit locates the centre better than the sqrt(3)/3 rule on full arcs",
        std::to_string(full_summary.worst_algebraic_center) + " m vs " +
            std::to_string(full_summary.worst_center) + " m");

  // THE +/-PI SEAM, isolated as its own named limitation rather than left to show up as a
  // mysterious miss. The ordered point list runs from bin 0 to bin B-1 and does not wrap, so an
  // object centred directly behind the robot arrives as two clusters - one at each end of the
  // list - and each half may fall below min_group_points and vanish. Upstream has the same
  // topology and the same consequence; its own source carries a TODO about points "on the
  // opposite sides of the scanner". This is a real coverage hole directly astern, and the phase
  // that owns the fix is whichever one can afford to make the point list circular.
  {
    const std::vector<detection_scenes::Cylinder> behind = {{-2.5, 0.0, 0.25}};
    const auto seam = detection_scenes::CastScene(behind, {}, 1.0, 24.0, 24);
    detector.Detect(seam.scan, &result);
    std::printf("\n      +/-pi seam: a cylinder centred at bearing pi gives %d clusters and"
                " %zu circles (%d clusters below min_group_points)\n",
                result.stats.clusters, result.circles.size(),
                result.stats.clusters_below_min_points);
    Check(result.stats.clusters == 2,
          "an object straddling the seam is split into two clusters, one at each end of the list",
          std::to_string(result.stats.clusters));

    // AND THE MERGE STAGE REPAIRS IT, which is worth knowing precisely because the split looks
    // fatal. When both halves clear min_group_points they each produce a segment, the two
    // segments are adjacent in bearing and collinear, so mergeSegments joins them and refits over
    // all the points - and the resulting circle is identical to the one the same cylinder gives
    // when seen dead ahead. The seam therefore costs nothing when the object is big enough, and
    // costs everything when it is not. The two cases are asserted separately below.
    double seam_error = 1.0;
    for (const CircleObservation& circle : result.circles) {
      seam_error = std::min(seam_error, (circle.center - Eigen::Vector2d(-2.5, 0.0)).norm());
    }
    const auto ahead = detection_scenes::SingleCylinderFullArc();
    Detection2DResult ahead_result;
    detector.Detect(ahead.scan, &ahead_result);
    double ahead_error = 1.0;
    for (const CircleObservation& circle : ahead_result.circles) {
      ahead_error = std::min(ahead_error, (circle.center - Eigen::Vector2d(2.5, 0.0)).norm());
    }
    std::printf("      +/-pi seam: the same 0.25 m cylinder at 2.5 m has a centre error of"
                " %.4f m astern vs %.4f m ahead\n", seam_error, ahead_error);
    Check(Near(seam_error, ahead_error, 1e-9),
          "segment merging rejoins the two halves, giving the same answer as the unsplit view",
          std::to_string(seam_error) + " m astern vs " + std::to_string(ahead_error) + " m ahead");

    // Further out, each half falls under min_group_points, so neither is ever fitted, there is no
    // segment for the merge stage to rejoin, and the object disappears entirely. THIS is the
    // failure mode that matters: nothing downstream can tell it from empty space. The same
    // cylinder at the same range ahead of the robot is detected, so the loss is the seam's, not
    // the range's - which is the comparison that makes it a coverage hole rather than a limit.
    const std::vector<detection_scenes::Cylinder> far_behind = {{-5.0, 0.0, 0.25}};
    const auto far_seam = detection_scenes::CastScene(far_behind, {}, 1.0, 25.0, 25);
    detector.Detect(far_seam.scan, &result);
    Check(result.circles.empty() && result.stats.clusters_below_min_points == result.stats.clusters,
          "at 5 m astern both halves fall below min_group_points and the object is not detected",
          std::to_string(result.stats.clusters) + " clusters, " +
              std::to_string(result.circles.size()) + " circles");

    const std::vector<detection_scenes::Cylinder> far_ahead = {{5.0, 0.0, 0.25}};
    const auto far_front = detection_scenes::CastScene(far_ahead, {}, 1.0, 26.0, 26);
    detector.Detect(far_front.scan, &result);
    Check(result.circles.size() == 1,
          "while the identical cylinder at 5 m AHEAD is detected - the loss is the seam's, not the"
          " range's",
          std::to_string(result.circles.size()) + " circles");
  }

  // The false-positive control: walls are not obstacles.
  {
    const auto walls = detection_scenes::WallsOnly();
    detector.Detect(walls.scan, &result);
    std::printf("      walls-only scene: %zu circles from %d attempted (%d rejected by radius)\n",
                result.circles.size(), result.stats.circles_attempted,
                result.stats.circles_rejected_radius);
    Check(result.circles.size() <= 4,
          "a walls-only scene yields at most one spurious circle per corner",
          std::to_string(result.circles.size()));
  }
}

// ---------------------------------------------------------------------------------------
// J  Allocation-free steady state.
// ---------------------------------------------------------------------------------------
void SectionJ() {
  Section("J. allocation-free steady state");

  const SegmentCircleDetectorParams params = ShippedParams();
  SegmentCircleDetector detector(params);
  Detection2DResult result;

  const auto scene = detection_scenes::ArenaFullArc();

  // The first calls size the caller's result buffers and the detector's point list; they are
  // allowed to allocate.
  for (int i = 0; i < 4; ++i) detector.Detect(scene.scan, &result);

  const std::size_t before = g_allocations;
  for (int i = 0; i < 50; ++i) detector.Detect(scene.scan, &result);
  const std::size_t steady = g_allocations - before;

  Check(steady == 0, "50 further detections on a warm buffer allocate nothing",
        std::to_string(steady) + " allocations");
  Check(!result.stats.capacity_exceeded,
        "and the arena scene stays inside the configured preallocation maxima");

  // The corpus as a whole, which exercises every branch. The first pass warms the high-water
  // mark; the second must be free.
  {
    const auto scenes = detection_scenes::AllScenes();
    for (const auto& s : scenes) detector.Detect(s.scan, &result);
    const std::size_t corpus_before = g_allocations;
    for (const auto& s : scenes) detector.Detect(s.scan, &result);
    const std::size_t corpus_steady = g_allocations - corpus_before;
    Check(corpus_steady == 0, "a second pass over the whole corpus allocates nothing",
          std::to_string(corpus_steady) + " allocations");
  }
}

// ---------------------------------------------------------------------------------------
// K. fit_residual_m reports FIT SCATTER, not the enlargement constant.
//
// The regression guard for the corrective pass. `fit_residual_m` used to be the RMS distance
// from the ENLARGED radius, so every circle reported a residual near radius_enlargement_m no
// matter how well its points fitted - on this repository's corpus the mean was 0.3076 m against
// a 0.2500 m enlargement, and the smallest residual any circle could produce was the
// enlargement itself. Its only consumer is the measurement-sigma formula, so a residual pinned
// to a configuration constant made sigma_center_m/sigma_radius_m constants too, and that
// propagated into the tracker's R and 66% of the safety stage's inflation.
//
// Three properties are asserted, in increasing strength.
// ---------------------------------------------------------------------------------------
std::string Number(double value, int digits) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
  return buffer;
}

void SectionK() {
  Section("K. fit_residual_m measures fit scatter, not radius_enlargement_m");

  const SegmentCircleDetectorParams shipped = ShippedParams();
  Detection2DResult result;

  // (1) INVARIANCE. The residual must not depend on the enlargement at all. This is the
  // structural property: it holds regardless of what the fit quality happens to be.
  //
  // ONE cylinder, so the comparison is per-circle rather than a mean over sets that could
  // differ: `max_circle_radius_m` (0.60 m) is applied to the ENLARGED radius, so a large
  // enlargement rejects circles a small one keeps, and a mean over the survivors would move for
  // a reason that has nothing to do with the residual. The sweep therefore stays inside the cap.
  {
    const auto scene = detection_scenes::SingleCylinderFullArc();
    std::vector<double> residual_at;
    std::vector<double> radius_at;
    for (const double enlargement : {0.0, 0.10, 0.25}) {
      SegmentCircleDetectorParams params = shipped;
      params.radius_enlargement_m = enlargement;
      SegmentCircleDetector detector(params);
      if (detector.Detect(scene.scan, &result) != nullptr || result.circles.size() != 1) {
        Check(false, "the single-cylinder scene gives exactly one circle at every enlargement",
              "enlargement " + Number(enlargement, 2));
        return;
      }
      residual_at.push_back(result.circles.front().fit_residual_m);
      radius_at.push_back(result.circles.front().radius_fitted_m);
    }
    std::printf("      enlargement 0.00 / 0.10 / 0.25 m ->"
                " radius_fitted_m %s / %s / %s m, fit_residual_m %s / %s / %s m\n",
                Number(radius_at[0], 4).c_str(), Number(radius_at[1], 4).c_str(),
                Number(radius_at[2], 4).c_str(), Number(residual_at[0], 6).c_str(),
                Number(residual_at[1], 6).c_str(), Number(residual_at[2], 6).c_str());
    Check(radius_at[2] > radius_at[0],
          "the sweep really is changing the enlargement - radius_fitted_m tracks it, so the "
          "invariance below is a property of the residual and not of an inert knob");
    Check(Near(residual_at[0], residual_at[1], 1e-12) &&
              Near(residual_at[1], residual_at[2], 1e-12),
          "fit_residual_m is INVARIANT under radius_enlargement_m - the enlargement is a safety "
          "pad on the reported radius, not a property of how well the points fit");
  }

  // (2) MAGNITUDE. On a clean, fully-visible fit the residual must be small in absolute terms,
  // and nowhere near the enlargement. A full-arc cylinder ray-cast at 1-degree bins is the
  // best-conditioned circle this detector ever sees.
  {
    SegmentCircleDetector detector(shipped);
    const auto scene = detection_scenes::SingleCylinderFullArc();
    if (detector.Detect(scene.scan, &result) == nullptr && !result.circles.empty()) {
      const double residual = result.circles.front().fit_residual_m;
      std::printf("      full-arc 0.25 m cylinder at 2.5 m: fit_residual_m %s m against a %s m "
                  "enlargement\n",
                  Number(residual, 6).c_str(), Number(shipped.radius_enlargement_m, 3).c_str());
      // The bound is the enlargement itself, because that is precisely the property the defect
      // violated: pre-fix the SMALLEST residual any circle on the corpus could report was
      // 0.2761 m, i.e. above the 0.2500 m enlargement, by construction. This circle reports
      // 0.0624 m, which is 0.37x the shipped 0.17 m enlargement.
      //
      // THAT RATIO IS NOT A FIXED PROPERTY and the bound deliberately does not encode one. The
      // residual is invariant under the enlargement (property 1 above) while the enlargement is
      // not invariant under re-derivation, so the ratio moved from 0.25x to 0.37x when the
      // enlargement went 0.25 m -> 0.17 m without a single residual changing. What is being
      // asserted is the sign of the inequality, which is the property the defect inverted.
      //
      // It is NOT near zero, and should not be: 0.062 m is the sqrt(3)/3 circumcircle's own
      // model mismatch on a full arc (it fits a 0.2383 m radius to a 0.25 m cylinder and offsets
      // the centre - P8 findings 1 and 2), which is real fit error and belongs in a fit residual.
      Check(residual < shipped.radius_enlargement_m,
            "a clean full-arc fit now reports a residual well below the enlargement, where the "
            "smallest residual the defect could produce was 0.2761 m - above it by construction",
            Number(residual, 6) + " m < " + Number(shipped.radius_enlargement_m, 4) + " m");
    } else {
      Check(false, "the single-cylinder full-arc scene produces a circle");
    }
  }

  // (3) RESPONSIVENESS. The strongest of the three, and the one an invariance check alone would
  // not catch: a residual that was simply hard-wired to zero would pass (1) and (2). Perturb the
  // points off the circle by a known amount and the residual must follow.
  //
  // The perturbation is applied to the RANGE of every occupied bin, alternating in sign, which
  // displaces each point radially by `amplitude` while leaving the cluster, the split and the
  // circle-from-segment construction to behave as they otherwise would.
  {
    const auto clean = detection_scenes::SingleCylinderFullArc();
    std::printf("      %12s | %14s\n", "amplitude", "fit_residual_m");
    std::vector<double> residuals;
    const std::vector<double> amplitudes = {0.0, 0.01, 0.03};
    for (const double amplitude : amplitudes) {
      ProjectedScan perturbed = clean.scan;
      int parity = 0;
      for (float& range : perturbed.ranges) {
        if (std::isnan(range)) continue;
        range += static_cast<float>((parity++ % 2 == 0) ? amplitude : -amplitude);
      }
      SegmentCircleDetector detector(shipped);
      if (detector.Detect(perturbed, &result) != nullptr || result.circles.empty()) {
        residuals.push_back(-1.0);
        continue;
      }
      residuals.push_back(result.circles.front().fit_residual_m);
      std::printf("      %12s | %14s\n", Number(amplitude, 4).c_str(),
                  Number(residuals.back(), 6).c_str());
    }
    if (residuals.size() == 3 && residuals[0] >= 0.0 && residuals[1] >= 0.0 &&
        residuals[2] >= 0.0) {
      Check(residuals[1] > residuals[0] && residuals[2] > residuals[1],
            "the residual RISES with injected point scatter - it is measuring something real, "
            "not reporting a constant",
            Number(residuals[0], 5) + " -> " + Number(residuals[1], 5) + " -> " +
                Number(residuals[2], 5) + " m");
      // QUANTITATIVELY, not just monotonically. The injected scatter is +/-amplitude on every
      // point, so its own RMS contribution is exactly `amplitude`, and it is independent of the
      // model mismatch already present - so the two should add in quadrature.
      const double predicted =
          std::sqrt(residuals[0] * residuals[0] + amplitudes[2] * amplitudes[2]);
      std::printf("      quadrature prediction at %s m: %s m; measured %s m\n",
                  Number(amplitudes[2], 3).c_str(), Number(predicted, 5).c_str(),
                  Number(residuals[2], 5).c_str());
      Check(Near(residuals[2], predicted, 0.25 * predicted),
            "and the rise matches the quadrature sum of the model mismatch already present and "
            "the injected scatter - so the residual is measuring point geometry, at the right "
            "scale, and not merely responding to it",
            "predicted " + Number(predicted, 5) + " m, measured " + Number(residuals[2], 5) + " m");
      // The dominant term is worth naming: on this detector a clean fit's residual is mostly
      // sqrt(3)/3 model mismatch, not sensor noise. Any future recalibration of the measurement
      // sigmas should know that is what it is scaling.
      std::printf("      NOTE: at zero injected scatter the residual is already %s m - the "
                  "sqrt(3)/3 model mismatch, not noise.\n", Number(residuals[0], 5).c_str());
    } else {
      Check(false, "the perturbation sweep produced a circle at every amplitude");
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <perception-directory>\n", argv[0]);
    return 2;
  }
  g_perception_dir = argv[1];

  std::printf("perception detection test (P8)\n");
  SectionA();
  SectionB();
  SectionC();
  SectionD();
  SectionE();
  SectionF();
  SectionG();
  SectionH();
  SectionI();
  SectionJ();
  SectionK();
  return perception_test::Report("perception_detection_test");
}
