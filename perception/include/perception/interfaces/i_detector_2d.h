// IDetector2D - the detection-stage seam.
//
// One scan in, the primitives and circular obstacle hypotheses of that scan out. The
// interface exists so an alternative circle fitter can replace the ported sqrt(3)/3 rule
// without the pipeline learning about it (architecture doc section 4.2: "keep as the default
// fitting backend; alternative fitters can be added behind the same interface later"), which
// is the open question the short-arc bias measurement feeds.
//
// NO STATE ACROSS CALLS. Detect() must depend on nothing but its argument and the detector's
// configuration - upstream clears its per-frame containers at the top of `processPoints`
// (obstacle_extractor.cpp:158-160) and so does every implementation of this. Cross-frame
// memory is the tracker's job, not this stage's, and a detector that quietly remembered the
// previous scan would make the tracker's association statistics meaningless.
#ifndef PERCEPTION_INTERFACES_I_DETECTOR_2D_H_
#define PERCEPTION_INTERFACES_I_DETECTOR_2D_H_

#include <cstdint>
#include <vector>

#include "perception/core/contracts/detection_2d.h"
#include "perception/core/contracts/projected_scan.h"

namespace perception::core {

// Per-scan census. Every number here is a count of a decision the detector made, so the
// section-12 "over/under-segmentation rates reported" gate has a source in the code rather
// than in a test's private bookkeeping.
struct DetectionStats {
  int32_t input_points = 0;  // Occupied bins of the scan - the ordered point list's length.

  int32_t clusters = 0;
  int32_t clusters_below_min_points = 0;  // Groups discarded by `min_group_points`.
  int32_t occluded_boundaries = 0;        // Group breaks judged to be occlusion edges.

  int32_t splits = 0;                  // Split-and-merge divisions performed.
  int32_t segments_before_merge = 0;
  int32_t segment_merges = 0;
  int32_t segments_after_merge = 0;

  // Rank-deficient line fits, i.e. clusters collinear with the sensor. Reported because a
  // rank-1 fit is the one case where the emitted segment is an artefact of the fit model
  // rather than a measurement, and because "how often does that happen" is the question the
  // Armadillo-vs-Eigen tolerance policy turns on.
  int32_t degenerate_fits = 0;

  // Fits that were FULL RANK and still produced a meaningless segment, because the fitted line
  // passes within rounding distance of the sensor origin. Upstream projects the endpoints by
  // dividing by D = A^2 + B^2 (figure_fitting.h:87-101), and D goes to zero exactly when the
  // line passes through the origin - so a coefficient agreement of 1e-16 becomes an endpoint
  // disagreement of any size at all, and the projected endpoints land astronomically far from
  // the data. The condition is REPORTED, not repaired: repairing it would be a third deliberate
  // divergence from the reference, and the radius cap already stops such a segment from ever
  // becoming a circle. A perfectly symmetric closed environment - four equal walls with the
  // sensor at the centre - is the reachable case, because the point set then sums to zero.
  int32_t ill_conditioned_projections = 0;

  // Emitted primitives that failed FittedPrimitive2D::Validate() and were dropped. The only
  // way to reach it is a fit that collapsed to zero length, which the contract forbids and
  // upstream would have published; dropping it is a deliberate divergence and is counted
  // rather than silent.
  int32_t primitives_rejected_invalid = 0;

  int32_t circles_attempted = 0;
  int32_t circles_rejected_visibility = 0;  // Skipped by `circles_from_visibles`.
  int32_t circles_rejected_radius = 0;      // radius >= max_circle_radius.
  int32_t circle_merges = 0;
  int32_t circles_rejected_invalid = 0;     // Failed CircleObservation::Validate().
  int32_t circles = 0;                      // Emitted.

  // Set when a result buffer had to grow past its configured maximum. A cap that truncated
  // silently would read as "nothing more was found"; this makes the opposite claim checkable.
  bool capacity_exceeded = false;

  double wall_time_us = 0.0;
};

struct Detection2DResult {
  std::vector<Cluster2D> clusters;
  std::vector<FittedPrimitive2D> primitives;
  std::vector<CircleObservation> circles;
  DetectionStats stats;

  void Clear() {
    clusters.clear();
    primitives.clear();
    circles.clear();
    stats = DetectionStats{};
  }
};

class IDetector2D {
 public:
  virtual ~IDetector2D() = default;

  // Fills `out` from `scan`. Returns nullptr on success or a STATIC reason string - the
  // non-allocating convention the contracts and the projector already use, because this runs
  // on the perception thread.
  //
  // `out` is caller-owned and is cleared on entry. Passing the same result object back on
  // every call is the intended use: that is what keeps the steady state allocation-free.
  virtual const char* Detect(const ProjectedScan& scan, Detection2DResult* out) = 0;
};

}  // namespace perception::core

#endif  // PERCEPTION_INTERFACES_I_DETECTOR_2D_H_
