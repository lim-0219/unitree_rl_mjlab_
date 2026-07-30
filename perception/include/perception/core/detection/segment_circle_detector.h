// SegmentCircleDetector - the ported obstacle_extractor detection core.
//
// PORTED FROM, faithful in semantics: obstacle_detector/src/obstacle_extractor.cpp, the
// `groupPoints` -> `detectSegments` -> `mergeSegments` -> `detectCircles` -> `mergeCircles`
// chain (lines 173-403 of the copy audited in this repository), together with the ROS-free
// helpers `utilities/segment.h`, `utilities/circle.h` and `utilities/figure_fitting.h`.
// The pipeline order, every threshold comparison and every literal constant are upstream's.
//
// LICENSE. The algorithms are BSD-3 (Poznan University of Technology, 2017, Mateusz
// Przybyla); see perception/third_party_notices.md. No upstream code is linked into
// production - upstream is ROS1 + Armadillo and its headers are compiled only by the optional
// regression-oracle test target.
//
// -----------------------------------------------------------------------------------------
// WHAT THE PORT DOES NOT REPRODUCE, AND WHY
// -----------------------------------------------------------------------------------------
// Two of upstream's point-counting expressions are defects, not conventions. Both were found
// by transcription rather than by testing, both are reproduced EXACTLY when
// `UpstreamQuirks::enabled` is set - which is what lets the Armadillo oracle be compared
// bit-for-bit instead of "everywhere except the parts that differ" - and both are corrected
// in the shipped default:
//
//   Q1  THE FIRST GROUP GAINS A POINT. `groupPoints` opens its loop with
//       `PointIterator point = input_points_.begin()++`. Post-increment applies to the
//       returned temporary, so the expression evaluates to `begin()` and the loop starts at
//       the FIRST point, not the second - the author plainly meant `++input_points_.begin()`.
//       The first point is therefore compared against itself, the distance is 0, the test
//       passes, and `num_points` is incremented for a point already in the set. Every
//       consequence follows from that one count: the first group's `min_group_points` test is
//       satisfied one point early, and `fitSegment` - which iterates `num_points` points from
//       `begin` - reads one point PAST the group, so the first cluster of every scan is fitted
//       with the first point of the NEXT cluster mixed in. When the whole scan is a single
//       group it reads past `end()` outright, which is undefined behaviour rather than a wrong
//       number. The quirk emulation reproduces the count and the foreign point but CLAMPS the
//       read to the points that exist, because reproducing undefined behaviour is not
//       fidelity.
//
//   Q2  THE SECOND HALF OF A SPLIT LOSES A POINT. `detectSegments` splits by CLONING the
//       dividing point into the point list, so the two halves genuinely share it and together
//       hold `num_points + 1` points. The first half is recorded with `split_index`, which is
//       right. The second half is recorded with `num_points - split_index`, which is one short
//       of the `num_points - split_index + 1` points it actually spans. So `fitSegment` omits
//       the half's final point from the least-squares fit while still projecting onto it as an
//       endpoint, and the omission propagates into every further split of that half.
//
// Neither quirk is exotic and neither is a rounding difference: Q1 mixes a foreign point into
// one fit per scan, Q2 into one fit per split. The corrected mode is the default for the same
// reason the projector's out-of-bounds index was fixed rather than copied (scan_projector.h):
// a port inherits an algorithm, not its defects, and a defect that survives into production
// because it was in the reference is the most expensive kind to find later.
//
// THE POINT LIST IS INDEXED, NOT LINKED. Upstream carries `std::list<Point>::iterator` pairs
// and splits by inserting a cloned point into the list. This port carries index spans into a
// flat vector, and represents the clone by letting the two halves SHARE the divider index.
// That is exactly equivalent - a list insertion invalidates no other iterator, and no other
// cluster's span contains the divider - and it removes the aliasing hazard the Cluster2D
// contract was defined to remove (detection_2d.h).
//
// -----------------------------------------------------------------------------------------
// THE ARMADILLO SUBSTITUTION
// -----------------------------------------------------------------------------------------
// `arma::pinv` is replaced by core/detection/line_fit.h, a streaming Givens QR followed by a
// closed-form 2x2 pseudo-inverse that uses Armadillo's own rank-truncation rule. See that
// header for why this is not an approximation of pinv but a reproduction of it, and
// tests/regression/line_fit_probe.cc for the measurement that settled the tolerance policy.
//
// -----------------------------------------------------------------------------------------
// PER-FRAME STATE ONLY
// -----------------------------------------------------------------------------------------
// Everything the detector holds is scratch, cleared at the top of every Detect() exactly as
// upstream clears its containers (obstacle_extractor.cpp:158-160). The buffers are reused
// across calls, which is the whole of the allocation-free steady state; nothing in them
// survives semantically from one scan to the next. Tracking is a later phase and owns all
// cross-frame memory.
#ifndef PERCEPTION_CORE_DETECTION_SEGMENT_CIRCLE_DETECTOR_H_
#define PERCEPTION_CORE_DETECTION_SEGMENT_CIRCLE_DETECTOR_H_

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "perception/core/detection/line_fit.h"
#include "perception/interfaces/i_detector_2d.h"

namespace perception::core {

// PROVISIONAL measurement-sigma constants. Named here rather than buried in the emission code
// because they are the two numbers in this module that were CHOSEN rather than ported, and the
// tracking phase is what has to replace them with calibrated values.
//
//   kProvisionalResidualFloorM   a fit residual can be legitimately zero (three points exactly
//                                on the fitted line), and a zero sigma tells a Kalman filter
//                                the measurement is perfect. This is the floor that prevents
//                                that, at the order of the projector's own range quantisation.
//   kProvisionalMinDilutionTerm  the 1/(1 - cos alpha) centre dilution diverges as the arc
//                                shortens; this caps it at 100x rather than letting a
//                                single-bin arc report an infinite uncertainty.
inline constexpr double kProvisionalResidualFloorM = 0.005;
inline constexpr double kProvisionalMinDilutionTerm = 0.01;

// How far from the origin the fitted line may sit, as a multiple of the farthest contributing
// point's range, before DetectionStats::ill_conditioned_projections counts the fit. A factor of
// 1000 is far beyond anything a real cluster produces and far below the 1e16 the pathological
// symmetric case reaches, so the counter is neither noisy nor blind.
inline constexpr double kIllConditionedProjectionFactor = 1.0e3;

// The detector's resolved parameters.
//
// WHY THIS IS NOT `DetectionConfig`: the same reason ScanProjectorParams is not
// ProjectionConfig - `perception_core` links `perception_contracts` and nothing else, and that
// link set is what enforces the core-dependency rules. The one-way mapping lives in
// perception/integration/detection_params.h. Field names are upstream's, so the port and its
// oracle can be tuned with the same numbers (architecture doc section 14).
struct SegmentCircleDetectorParams {
  int min_group_points = 5;
  double max_group_distance_m = 0.10;
  double distance_proportion = 0.017453292519943295;
  double max_split_distance_m = 0.20;
  double max_merge_separation_m = 0.20;
  double max_merge_spread_m = 0.20;
  double max_circle_radius_m = 0.60;
  double radius_enlargement_m = 0.25;
  bool circles_from_visibles = true;
  bool use_split_and_merge = true;

  // UPSTREAM default (true). A segment that produced an accepted circle is removed from the
  // segment list, so the two outputs partition the scan instead of describing it twice.
  bool discard_converted_segments = true;

  int max_clusters = 256;
  int max_primitives = 512;
  int max_circles = 256;

  const char* Validate() const;
  bool IsValid() const { return Validate() == nullptr; }
};

// Test-only emulation of the two upstream point-counting defects described in the file
// header. Default-constructed means "corrected", which is the shipped behaviour; nothing on
// the production path ever sets this, and it is deliberately NOT reachable from the YAML.
struct UpstreamQuirks {
  bool enabled = false;

  static UpstreamQuirks Exact() {
    UpstreamQuirks quirks;
    quirks.enabled = true;
    return quirks;
  }
};

class SegmentCircleDetector final : public IDetector2D {
 public:
  explicit SegmentCircleDetector(const SegmentCircleDetectorParams& params,
                                 const UpstreamQuirks& quirks = UpstreamQuirks{});

  const SegmentCircleDetectorParams& params() const { return params_; }
  const UpstreamQuirks& quirks() const { return quirks_; }

  const char* Detect(const ProjectedScan& scan, Detection2DResult* out) override;

  // The ordered point list of the last Detect() - the sequence of Cartesian points built from
  // the occupied bins, which is the index space Cluster2D addresses. Exposed for the
  // regression harness, which has to hand the SAME list to the upstream oracle, and for the
  // visualization adapters, which draw split boundaries in it.
  const std::vector<Eigen::Vector2d>& points() const { return points_; }
  const std::vector<int32_t>& point_bins() const { return point_bins_; }

  // Reserves every scratch buffer to the configured maxima. Called by the constructor; public
  // so a caller that changed nothing can still assert the steady state is warm.
  void Reserve();

 private:
  // A half-open-free, INCLUSIVE index span into `points_`, plus the point count upstream
  // recorded for it - which is the same thing as `last - first + 1` in corrected mode and
  // deliberately is not in quirk mode.
  struct PointSpan {
    int32_t first = 0;
    int32_t last = 0;
    int32_t recorded_count = 0;
    bool front_visible = false;
    bool back_visible = false;
    int32_t cluster_index = -1;
  };

  struct SegmentRecord {
    Eigen::Vector2d first_point{0.0, 0.0};
    Eigen::Vector2d last_point{0.0, 0.0};
    // The contributing spans, in upstream's `Segment::point_sets` order. A merge concatenates
    // them; a split hands each half its own.
    int32_t span_begin = 0;  // Index into spans_.
    int32_t span_count = 0;
    bool degenerate_fit = false;
  };

  void BuildPointList(const ProjectedScan& scan);
  void GroupPoints();
  void DetectSegments(const PointSpan& span);
  void MergeSegments();
  bool CompareSegments(int32_t i, int32_t j, SegmentRecord* merged);
  void DetectCircles();
  void MergeCircles();
  void EmitPrimitives(const ProjectedScan& scan, Detection2DResult* out);

  // fitSegment(point_set) and fitSegment(vector<PointSet>) - one implementation, because
  // upstream's two overloads differ only in how they walk to the points.
  SegmentRecord FitSegment(const PointSpan* spans, int32_t span_count);

  SegmentCircleDetectorParams params_;
  UpstreamQuirks quirks_;

  // Per-frame scratch. Reused across calls; cleared, never carried.
  std::vector<Eigen::Vector2d> points_;
  std::vector<int32_t> point_bins_;
  std::vector<Cluster2D> clusters_;
  std::vector<PointSpan> spans_;  // Flat arena the SegmentRecords index into.
  std::vector<SegmentRecord> segments_;
  std::vector<PointSpan> concat_scratch_;  // Span concatenation during a merge trial.

  struct CircleRecord {
    Eigen::Vector2d center{0.0, 0.0};
    double radius = 0.0;

    // The same radius WITHOUT `radius_enlargement_m`: the bare circumcircle radius at
    // construction, and the merge rule applied to bare radii thereafter. Carried rather than
    // recovered by subtraction because the merge rule compounds the enlargement more than once
    // (see MergeCircles), so `radius - radius_enlargement_m` is only correct for an unmerged
    // circle.
    //
    // IT DECIDES NOTHING. Every merge test, the max_circle_radius_m cap and the emitted
    // `radius_fitted_m` all read `radius`, exactly as before. This field exists solely so
    // `fit_residual_m` can be measured against the circle the points are actually near.
    double radius_unenlarged = 0.0;

    int32_t span_begin = 0;
    int32_t span_count = 0;
  };
  std::vector<CircleRecord> circles_;

  LineFitAccumulator accumulator_;
  DetectionStats* stats_ = nullptr;  // Points into the caller's result during Detect().
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_DETECTION_SEGMENT_CIRCLE_DETECTOR_H_
