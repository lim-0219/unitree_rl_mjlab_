// P8 upstream parity: the ported detection core against the real obstacle_extractor algorithm,
// compiled from upstream's own ROS-free headers and its own `arma::pinv`.
//
// This is the optional half of P8's testing (PERCEPTION_BUILD_UPSTREAM_ORACLE, needs Armadillo).
// It answers one question the port cannot answer about itself: did the transcription change the
// algorithm? The other half - whether upstream's answer is the answer we want - is
// tests/unit/detection_test.cpp, which needs neither Armadillo nor this file.
//
// THREE SECTIONS, and the order matters because the first one decides what the second is allowed
// to assert:
//
//   A  THE SOLVER PROBE. `arma::pinv(X) * 1` versus core/detection/line_fit.h, on the same point
//      sets, over the real corpus AND over constructed degenerate clusters. This is the
//      measurement the architecture doc leaves open ("pinv vs QR may differ on degenerate
//      clusters -> tolerance policy section 11"): it reports the coefficient and endpoint deltas
//      against the condition number of each design matrix, so the tolerance policy is chosen
//      from data instead of asserted.
//
//   B  END-TO-END PARITY. The whole pipeline, port versus upstream, on identical point lists,
//      with the port in UpstreamQuirks::Exact() mode so the two documented upstream defects are
//      reproduced and the comparison is about the algorithm rather than about the defects.
//
//   C  THE COST OF THE TWO FIXES. Corrected mode versus quirk mode on the same corpus, so the
//      size of the deliberate divergence is a measured number rather than an argument.
//
// argv[1] is the perception/ directory - for the shipped config and the committed P7 goldens.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <armadillo>

#include "perception/core/contracts/projected_scan.h"
#include "perception/core/detection/line_fit.h"
#include "perception/core/detection/segment_circle_detector.h"
#include "perception/core/diagnostics/dump_file.h"

#include "check.h"
#include "detection_scenes.h"
#include "upstream_extractor_reference.h"

namespace diagnostics = perception::core::diagnostics;

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
using perception::core::UpstreamQuirks;
using perception_test::Check;
using perception_test::Section;

namespace {

std::filesystem::path g_perception_dir;

SegmentCircleDetectorParams ShippedParams() {
  SegmentCircleDetectorParams params;
  params.min_group_points = 5;
  params.max_group_distance_m = 0.10;
  params.distance_proportion = 2.0 * kPi / 360.0;
  params.max_split_distance_m = 0.20;
  params.max_merge_separation_m = 0.20;
  params.max_merge_spread_m = 0.20;
  params.max_circle_radius_m = 0.60;
  params.radius_enlargement_m = 0.25;
  params.circles_from_visibles = true;
  params.use_split_and_merge = true;
  params.discard_converted_segments = true;
  params.max_clusters = 256;
  params.max_primitives = 512;
  params.max_circles = 256;
  return params;
}

upstream_reference::Params UpstreamParams(const SegmentCircleDetectorParams& params) {
  upstream_reference::Params out;
  out.use_split_and_merge = params.use_split_and_merge;
  out.circles_from_visibles = params.circles_from_visibles;
  out.discard_converted_segments = params.discard_converted_segments;
  out.min_group_points = params.min_group_points;
  out.distance_proportion = params.distance_proportion;
  out.max_group_distance = params.max_group_distance_m;
  out.max_split_distance = params.max_split_distance_m;
  out.max_merge_separation = params.max_merge_separation_m;
  out.max_merge_spread = params.max_merge_spread_m;
  out.max_circle_radius = params.max_circle_radius_m;
  out.radius_enlargement = params.radius_enlargement_m;
  return out;
}

std::vector<upstream_reference::Point> ToUpstreamPoints(
    const std::vector<Eigen::Vector2d>& points) {
  std::vector<upstream_reference::Point> out;
  out.reserve(points.size());
  for (const Eigen::Vector2d& point : points) out.emplace_back(point.x(), point.y());
  return out;
}

// ---------------------------------------------------------------------------------------
// A  The solver probe.
// ---------------------------------------------------------------------------------------

// Upstream's fit, verbatim: build the N x 2 design matrix, take the pseudo-inverse, multiply by
// the vector of ones. Nothing about this is reformulated - forming the full 2 x N pseudo-inverse
// and then multiplying is itself part of what is being compared against.
struct ArmaFit {
  double a = 0.0;
  double b = 0.0;
};

ArmaFit UpstreamLineFit(const std::vector<Eigen::Vector2d>& points) {
  const int n = static_cast<int>(points.size());
  arma::mat input = arma::mat(n, 2).zeros();
  arma::vec output = arma::vec(n).ones();
  for (int i = 0; i < n; ++i) {
    input(i, 0) = points[static_cast<std::size_t>(i)].x();
    input(i, 1) = points[static_cast<std::size_t>(i)].y();
  }
  const arma::vec parameters = arma::pinv(input) * output;
  ArmaFit fit;
  fit.a = parameters(0);
  fit.b = parameters(1);
  return fit;
}

// The endpoint projection upstream applies after the fit. Compared instead of the raw
// coefficients because it is what actually reaches the rest of the pipeline: a large coefficient
// difference on a near-singular fit can still project to the same point, and a small one need not.
Eigen::Vector2d ProjectEndpoint(double a, double b, const Eigen::Vector2d& p) {
  const double c = -1.0;
  const double d = a * a + b * b;
  if (!(d > 0.0)) return p;
  return Eigen::Vector2d((b * b * p.x() - a * b * p.y() - a * c) / d,
                         (-a * b * p.x() + a * a * p.y() - b * c) / d);
}

// THE THREE CLASSES THE MEASUREMENT ACTUALLY SPLITS INTO. The open question assumed the boundary
// would be "degenerate vs not"; it is not. What decides whether the two implementations agree is
// the conditioning of upstream's ENDPOINT PROJECTION, which is a separate step from the solve.
enum class ProbeClass {
  kWellPosed,        // >= 3 points, fitted line at a sane distance from the origin.
  kTwoPointGroup,    // Exactly 2 points: the fit is determined, and agreement is bounded by
                     // cond(X) * eps rather than by anything the solver chose to do.
  kIllPosedProjection,  // D -> 0: the line passes through the sensor, and 1/D amplifies without
                        // bound. Indeterminate in BOTH implementations.
};

const char* ToString(ProbeClass value) {
  switch (value) {
    case ProbeClass::kWellPosed:          return "well-posed";
    case ProbeClass::kTwoPointGroup:      return "2-point";
    case ProbeClass::kIllPosedProjection: return "ill-posed-proj";
  }
  return "?";
}

struct ProbeRow {
  std::string name;
  int point_count = 0;
  int rank = 0;
  double condition = 0.0;
  double coefficient_delta = 0.0;
  double endpoint_delta = 0.0;
  double line_offset_ratio = 0.0;  // (1/sqrt(D)) / max point range.
  ProbeClass klass = ProbeClass::kWellPosed;
};

ProbeRow Probe(const std::string& name, const std::vector<Eigen::Vector2d>& points) {
  ProbeRow row;
  row.name = name;
  row.point_count = static_cast<int>(points.size());

  LineFitAccumulator accumulator;
  for (const Eigen::Vector2d& point : points) accumulator.Add(point.x(), point.y());
  const LineFit2D ported = accumulator.Solve();
  const ArmaFit upstream = UpstreamLineFit(points);

  row.rank = ported.rank;
  row.condition = ported.ConditionNumber();
  row.coefficient_delta =
      std::max(std::abs(ported.a - upstream.a), std::abs(ported.b - upstream.b));

  const Eigen::Vector2d& first = points.front();
  const Eigen::Vector2d& last = points.back();
  row.endpoint_delta = std::max(
      (ProjectEndpoint(ported.a, ported.b, first) - ProjectEndpoint(upstream.a, upstream.b, first))
          .norm(),
      (ProjectEndpoint(ported.a, ported.b, last) - ProjectEndpoint(upstream.a, upstream.b, last))
          .norm());

  double max_range = 0.0;
  for (const Eigen::Vector2d& point : points) max_range = std::max(max_range, point.norm());
  const double d = ported.D();
  row.line_offset_ratio = d > 0.0 ? (1.0 / std::sqrt(d)) / std::max(max_range, 1e-12)
                                  : std::numeric_limits<double>::infinity();
  if (row.line_offset_ratio > perception::core::kIllConditionedProjectionFactor) {
    row.klass = ProbeClass::kIllPosedProjection;
  } else if (points.size() < 3) {
    row.klass = ProbeClass::kTwoPointGroup;
  } else {
    row.klass = ProbeClass::kWellPosed;
  }
  return row;
}

// Every top-level cluster the detector forms on a scan, as a point set - the real population of
// fit problems, rather than a hand-picked one.
void CollectClusterPointSets(const SegmentCircleDetector& detector, const Detection2DResult& result,
                             const std::string& scene, int min_group_points,
                             std::vector<ProbeRow>* rows) {
  for (std::size_t c = 0; c < result.clusters.size(); ++c) {
    const Cluster2D& cluster = result.clusters[c];
    // Clusters under min_group_points are never handed to fitSegment at all, so including them
    // would inflate the population with problems the pipeline does not solve - and would put
    // two-point groups into a table whose headline claim is about the fits that really happen.
    if (cluster.point_count < min_group_points) continue;
    std::vector<Eigen::Vector2d> points;
    points.reserve(static_cast<std::size_t>(cluster.point_count));
    for (int i = cluster.first_index; i <= cluster.last_index; ++i) {
      points.push_back(detector.points()[static_cast<std::size_t>(i)]);
    }
    rows->push_back(Probe(scene + "#" + std::to_string(c), points));
  }
}

std::vector<ProjectedScan> LoadGoldenScans() {
  std::vector<ProjectedScan> scans;
  const std::filesystem::path path =
      g_perception_dir / "tests" / "fixtures" / "projection_golden" / "projected_scans.jsonl";
  diagnostics::DumpReader reader;
  std::string error;
  if (!reader.Open(path.string(), error)) {
    Check(false, "the P7 golden ProjectedScan corpus opens", error);
    return scans;
  }
  if (!reader.ReadAll(scans, error)) {
    Check(false, "the P7 golden corpus reads to a well-formed footer", error);
    scans.clear();
  }
  return scans;
}

void SectionA(const std::vector<ProjectedScan>& goldens) {
  Section("A. the solver probe - arma::pinv vs the ported streaming QR");

  const SegmentCircleDetectorParams params = ShippedParams();
  SegmentCircleDetector detector(params, UpstreamQuirks::Exact());
  Detection2DResult result;

  std::vector<ProbeRow> rows;

  // The constructed degenerate cases first, because they are the ones the open question was
  // about, and they are named.
  for (const auto& fit_case : detection_scenes::DegenerateFitCases()) {
    rows.push_back(Probe(fit_case.name, fit_case.points));
  }
  const std::size_t degenerate_rows = rows.size();

  // Then the real population: every cluster of every P7 golden scan and every P8 scene.
  for (std::size_t i = 0; i < goldens.size(); ++i) {
    if (detector.Detect(goldens[i], &result) != nullptr) continue;
    CollectClusterPointSets(detector, result, "golden" + std::to_string(i),
                            params.min_group_points, &rows);
  }
  for (const auto& scene : detection_scenes::AllScenes()) {
    if (detector.Detect(scene.scan, &result) != nullptr) continue;
    CollectClusterPointSets(detector, result, scene.name, params.min_group_points, &rows);
  }

  std::printf("      %-28s %5s %5s %11s %11s %11s %10s %s\n", "case", "n", "rank", "cond",
              "dCoeff", "dEndpoint", "lineOff", "class");
  for (std::size_t i = 0; i < rows.size(); ++i) {
    // Every named constructed case is printed; the corpus rows are printed only when they have
    // something to say, or the table would be dozens of identical lines.
    const bool interesting = i < degenerate_rows || rows[i].rank < 2 ||
                             rows[i].klass != ProbeClass::kWellPosed ||
                             rows[i].endpoint_delta > 1e-12;
    if (!interesting) continue;
    std::printf("      %-28s %5d %5d %11.3e %11.3e %11.3e %10.2e %s\n", rows[i].name.c_str(),
                rows[i].point_count, rows[i].rank, rows[i].condition, rows[i].coefficient_delta,
                rows[i].endpoint_delta, rows[i].line_offset_ratio, ToString(rows[i].klass));
  }

  // The aggregate, split by the three classes above and additionally by rank, because "does rank
  // deficiency cause divergence" was the question asked and it deserves a direct answer.
  int counts[3] = {0, 0, 0};
  double worst_endpoint[3] = {0.0, 0.0, 0.0};
  double worst_coefficient[3] = {0.0, 0.0, 0.0};
  int deficient = 0;
  double worst_deficient_endpoint = 0.0;
  double worst_well_posed_condition = 0.0;
  for (const ProbeRow& row : rows) {
    const auto index = static_cast<std::size_t>(row.klass);
    ++counts[index];
    worst_endpoint[index] = std::max(worst_endpoint[index], row.endpoint_delta);
    worst_coefficient[index] = std::max(worst_coefficient[index], row.coefficient_delta);
    if (row.rank < 2) {
      ++deficient;
      if (row.klass != ProbeClass::kIllPosedProjection) {
        worst_deficient_endpoint = std::max(worst_deficient_endpoint, row.endpoint_delta);
      }
    }
    if (row.klass == ProbeClass::kWellPosed && std::isfinite(row.condition)) {
      worst_well_posed_condition = std::max(worst_well_posed_condition, row.condition);
    }
  }

  std::printf("\n      %zu fit problems: %d well-posed, %d two-point, %d ill-posed-projection;"
              " %d rank-deficient overall\n",
              rows.size(), counts[0], counts[1], counts[2], deficient);
  std::printf("      worst well-posed condition number %.3e\n", worst_well_posed_condition);
  for (int k = 0; k < 3; ++k) {
    std::printf("      %-15s worst dCoeff %.3e, worst dEndpoint %.3e m\n",
                ToString(static_cast<ProbeClass>(k)), worst_coefficient[static_cast<std::size_t>(k)],
                worst_endpoint[static_cast<std::size_t>(k)]);
  }

  // ---------------------------------------------------------------------------------------
  // THE TOLERANCE POLICY, decided from the table above.
  //
  //   1. WELL-POSED FITS (>= 3 points, fitted line within 1e3 x the data extent of the origin)
  //      agree to <= 1e-9 m on the projected endpoints, INCLUDING the rank-deficient ones - the
  //      port reproduces Armadillo's own truncation rule rather than approximating it, so
  //      rank deficiency turns out not to be a source of divergence at all. This class contains
  //      100% of the fits the pipeline actually performs on the corpus. Section 11's nominal
  //      <= 1e-9 therefore holds unconditionally here, and the Armadillo oracle is a REGRESSION
  //      CONVENIENCE for this class - answer (a) to the open question.
  //
  //      Note what the coefficients do NOT do: on a near-radial cluster they disagree by up to
  //      1e+05 in absolute value while the endpoints still agree to 1e-16, because a near-radial
  //      line has enormous A and B that the projection's 1/(A^2+B^2) divides straight back out.
  //      Comparing coefficients would have produced a spectacular false alarm.
  //
  //   2. TWO-POINT GROUPS are bounded by cond(X) * eps, not by the solver: at cond 1e11 the two
  //      implementations differ by ~4e-6 m and no choice of decomposition changes that. They
  //      cannot occur under the shipped configuration (min_group_points = 5) and are documented
  //      case-by-case per section 11 rather than tolerated silently.
  //
  //   3. ILL-POSED PROJECTIONS are indeterminate in BOTH implementations. Upstream divides the
  //      endpoint projection by D = A^2 + B^2, which vanishes when the fitted line passes through
  //      the sensor; a 1e-16 coefficient agreement then becomes an endpoint disagreement of 1e+17
  //      metres. This is not a pinv-versus-QR effect - it is upstream's projection formula, and it
  //      is reached by a perfectly symmetric closed environment, where the point set sums to zero.
  //      The detector counts the condition (DetectionStats::ill_conditioned_projections) and the
  //      radius cap keeps such a segment from ever becoming a circle.
  //
  // The answer to the architecture doc's open question is therefore (a) WITH a named exception
  // list, not (b): the oracle target is a regression convenience for every fit the pipeline
  // really performs, and the two exception classes are properties of upstream's formulation that
  // no substitution could have avoided.
  // ---------------------------------------------------------------------------------------
  Check(deficient > 0, "the corpus plus the constructed cases do reach rank-deficient fits",
        std::to_string(deficient) + " of " + std::to_string(rows.size()));
  Check(counts[2] > 0, "and they do reach ill-posed projections", std::to_string(counts[2]));
  Check(worst_endpoint[0] <= 1e-9,
        "class 1 (well-posed): endpoints agree to the section-11 nominal 1e-9 m",
        std::to_string(worst_endpoint[0]));
  Check(worst_deficient_endpoint <= 1e-9,
        "and rank deficiency alone causes NO divergence - the truncation rule is shared",
        std::to_string(worst_deficient_endpoint));
  Check(worst_endpoint[1] <= 1e-5,
        "class 2 (two-point groups): bounded by cond(X) * eps, measured within 1e-5 m",
        std::to_string(worst_endpoint[1]));
  Check(worst_coefficient[0] > 1e-3,
        "the coefficients DO diverge where the endpoints do not - comparing them would mislead",
        std::to_string(worst_coefficient[0]));
}

// ---------------------------------------------------------------------------------------
// B  End-to-end parity.
// ---------------------------------------------------------------------------------------
struct ParityResult {
  int compared = 0;
  int indeterminate = 0;  // Scans containing an ill-posed projection; see the policy in section A.
  int count_mismatches = 0;
  double worst_segment_endpoint = 0.0;
  double worst_circle_center = 0.0;
  double worst_circle_radius = 0.0;
  std::string worst_case;
};

void CompareOne(const std::string& name, const ProjectedScan& scan, SegmentCircleDetector& detector,
                const SegmentCircleDetectorParams& params, ParityResult* parity) {
  Detection2DResult result;
  if (detector.Detect(scan, &result) != nullptr) return;

  upstream_reference::Extractor extractor(UpstreamParams(params));
  extractor.Process(ToUpstreamPoints(detector.points()));

  // A scan whose detector reports an ill-posed projection is class 3 of the tolerance policy: the
  // split decisions downstream of that fit are made by comparing 1e16-metre distances against a
  // 0.2-metre threshold, so the two implementations may legitimately produce different SEGMENT
  // COUNTS. Excluded from the parity assertion by that reported condition rather than by name, so
  // a new scene that reaches it is handled automatically instead of silently failing.
  if (result.stats.ill_conditioned_projections > 0) {
    ++parity->indeterminate;
    std::printf("      INDETERMINATE  %-24s %d ill-posed projection(s): excluded from parity\n",
                name.c_str(), result.stats.ill_conditioned_projections);
    return;
  }

  ++parity->compared;

  std::vector<const FittedPrimitive2D*> ported_segments;
  for (const FittedPrimitive2D& primitive : result.primitives) {
    if (primitive.kind == PrimitiveKind::kSegment) ported_segments.push_back(&primitive);
  }

  if (ported_segments.size() != extractor.segments().size() ||
      result.circles.size() != extractor.circles().size()) {
    ++parity->count_mismatches;
    std::printf("      COUNT MISMATCH %-24s segments %zu vs %zu, circles %zu vs %zu\n",
                name.c_str(), ported_segments.size(), extractor.segments().size(),
                result.circles.size(), extractor.circles().size());
    return;
  }

  std::size_t index = 0;
  for (const upstream_reference::Segment& segment : extractor.segments()) {
    const FittedPrimitive2D& ported = *ported_segments[index++];
    const double first = (ported.first_point -
                          Eigen::Vector2d(segment.first_point.x, segment.first_point.y)).norm();
    const double last = (ported.last_point -
                         Eigen::Vector2d(segment.last_point.x, segment.last_point.y)).norm();
    const double worst = std::max(first, last);
    if (worst > parity->worst_segment_endpoint) {
      parity->worst_segment_endpoint = worst;
      parity->worst_case = name;
    }
  }

  index = 0;
  for (const upstream_reference::Circle& circle : extractor.circles()) {
    const CircleObservation& ported = result.circles[index++];
    parity->worst_circle_center =
        std::max(parity->worst_circle_center,
                 (ported.center - Eigen::Vector2d(circle.center.x, circle.center.y)).norm());
    parity->worst_circle_radius =
        std::max(parity->worst_circle_radius, std::abs(ported.radius_fitted_m - circle.radius));
  }
}

void SectionB(const std::vector<ProjectedScan>& goldens) {
  Section("B. end-to-end parity, port in UpstreamQuirks::Exact() mode");

  const SegmentCircleDetectorParams params = ShippedParams();
  SegmentCircleDetector detector(params, UpstreamQuirks::Exact());
  ParityResult parity;

  for (std::size_t i = 0; i < goldens.size(); ++i) {
    CompareOne("golden" + std::to_string(i), goldens[i], detector, params, &parity);
  }
  for (const auto& scene : detection_scenes::AllScenes()) {
    CompareOne(scene.name, scene.scan, detector, params, &parity);
  }

  std::printf("      %d scans compared, %d excluded as indeterminate;"
              " worst segment endpoint delta %.3e m (%s)\n",
              parity.compared, parity.indeterminate, parity.worst_segment_endpoint,
              parity.worst_case.c_str());
  std::printf("      worst circle centre delta %.3e m, worst circle radius delta %.3e m\n",
              parity.worst_circle_center, parity.worst_circle_radius);

  Check(parity.compared >= 20, "the parity corpus is not trivially small",
        std::to_string(parity.compared) + " scans");
  Check(parity.indeterminate <= 2,
        "and almost none of it is excluded - the ill-posed class is a corner, not the corpus",
        std::to_string(parity.indeterminate) + " excluded");
  Check(parity.count_mismatches == 0,
        "the port and upstream produce the same NUMBER of segments and circles on every scan",
        std::to_string(parity.count_mismatches) + " mismatches");
  Check(parity.worst_segment_endpoint <= 1e-9, "segment endpoints agree to 1e-9 m",
        std::to_string(parity.worst_segment_endpoint));
  Check(parity.worst_circle_center <= 1e-9, "circle centres agree to 1e-9 m",
        std::to_string(parity.worst_circle_center));
  Check(parity.worst_circle_radius <= 1e-9, "circle radii agree to 1e-9 m",
        std::to_string(parity.worst_circle_radius));

  // THE NEGATIVE CONTROLS. A comparison that cannot fail is not evidence. Two are needed because
  // the two halves of the comparison are independent: one perturbation that must move the
  // SEGMENTS, and one that must move the CIRCLES. In both, `params` - not the perturbed copy -
  // drives the upstream side, which is what makes the two sides genuinely disagree.
  {
    SegmentCircleDetectorParams altered = params;
    altered.max_group_distance_m = 0.05;  // Tighter grouping: different clusters, different segments.
    SegmentCircleDetector perturbed(altered, UpstreamQuirks::Exact());
    ParityResult control;
    for (const auto& scene : detection_scenes::AllScenes()) {
      CompareOne(scene.name, scene.scan, perturbed, params, &control);
    }
    Check(control.count_mismatches > 0 || control.worst_segment_endpoint > 1e-9,
          "control 1: a changed grouping threshold breaks segment parity",
          std::to_string(control.count_mismatches) + " count mismatches, worst endpoint " +
              std::to_string(control.worst_segment_endpoint));
  }
  {
    SegmentCircleDetectorParams altered = params;
    altered.radius_enlargement_m = params.radius_enlargement_m + 1e-6;
    SegmentCircleDetector perturbed(altered, UpstreamQuirks::Exact());
    ParityResult control;
    for (const auto& scene : detection_scenes::AllScenes()) {
      CompareOne(scene.name, scene.scan, perturbed, params, &control);
    }
    Check(control.worst_circle_radius > 1e-9,
          "control 2: a 1e-6 m change in the enlargement breaks circle parity",
          std::to_string(control.worst_circle_radius));
  }
}

// ---------------------------------------------------------------------------------------
// C  What the two deliberate fixes cost.
// ---------------------------------------------------------------------------------------
void SectionC(const std::vector<ProjectedScan>& goldens) {
  Section("C. corrected mode vs upstream-exact mode - the size of the divergence");

  const SegmentCircleDetectorParams params = ShippedParams();
  SegmentCircleDetector corrected(params);
  SegmentCircleDetector exact(params, UpstreamQuirks::Exact());

  int scans = 0;
  int differing_scans = 0;
  int corrected_circles = 0;
  int exact_circles = 0;
  double worst_circle_delta = 0.0;

  std::vector<ProjectedScan> corpus = goldens;
  for (const auto& scene : detection_scenes::AllScenes()) corpus.push_back(scene.scan);

  Detection2DResult a;
  Detection2DResult b;
  for (const ProjectedScan& scan : corpus) {
    if (corrected.Detect(scan, &a) != nullptr) continue;
    if (exact.Detect(scan, &b) != nullptr) continue;
    ++scans;
    corrected_circles += static_cast<int>(a.circles.size());
    exact_circles += static_cast<int>(b.circles.size());

    bool differs = a.circles.size() != b.circles.size() ||
                   a.primitives.size() != b.primitives.size() ||
                   a.clusters.size() != b.clusters.size();
    const std::size_t common = std::min(a.circles.size(), b.circles.size());
    for (std::size_t i = 0; i < common; ++i) {
      const double delta = (a.circles[i].center - b.circles[i].center).norm();
      if (delta > 1e-12) differs = true;
      worst_circle_delta = std::max(worst_circle_delta, delta);
    }
    if (differs) ++differing_scans;
  }

  std::printf("      %d scans: %d differ between corrected and upstream-exact mode\n", scans,
              differing_scans);
  std::printf("      circles emitted: %d corrected vs %d upstream-exact;"
              " worst matched-centre delta %.3e m\n",
              corrected_circles, exact_circles, worst_circle_delta);

  Check(scans >= 20, "the divergence corpus is not trivially small", std::to_string(scans));
  Check(differing_scans > 0,
        "the two documented upstream defects DO change the output - the fixes are not cosmetic",
        std::to_string(differing_scans) + " of " + std::to_string(scans) + " scans");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <perception-directory>\n", argv[0]);
    return 2;
  }
  g_perception_dir = argv[1];

  std::printf("perception detection oracle test (P8, upstream obstacle_detector + Armadillo %s)\n",
              arma::arma_version::as_string().c_str());

  const std::vector<ProjectedScan> goldens = LoadGoldenScans();
  Check(!goldens.empty(), "the P7 golden corpus is available as P8 input",
        std::to_string(goldens.size()) + " scans");

  SectionA(goldens);
  SectionB(goldens);
  SectionC(goldens);
  return perception_test::Report("perception_detection_oracle_test");
}
