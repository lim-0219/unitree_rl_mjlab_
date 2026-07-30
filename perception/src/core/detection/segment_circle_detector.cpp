#include "perception/core/detection/segment_circle_detector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace perception::core {
namespace {

// -----------------------------------------------------------------------------------------
// Upstream's `utilities/point.h` arithmetic, transcribed where its behaviour differs from the
// obvious Eigen equivalent. These are not style choices - each one is a case where upstream
// returns a defined value that Eigen would turn into an inf or a NaN, and the port has to
// take the same branch to stay comparable.
// -----------------------------------------------------------------------------------------

double Cross(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return a.x() * b.y() - a.y() * b.x();
}

// Point::perpendicular(): (-y, x).
Eigen::Vector2d Perpendicular(const Eigen::Vector2d& v) { return Eigen::Vector2d(-v.y(), v.x()); }

// Point::normalized(): the vector unchanged when its length is zero, NOT a NaN.
Eigen::Vector2d NormalizedOrSelf(const Eigen::Vector2d& v) {
  const double length = v.norm();
  return length > 0.0 ? Eigen::Vector2d(v / length) : v;
}

// Point::operator/(Point, double): the ZERO point when the divisor is zero.
Eigen::Vector2d DivideOrZero(const Eigen::Vector2d& v, double divisor) {
  return divisor != 0.0 ? Eigen::Vector2d(v / divisor) : Eigen::Vector2d(0.0, 0.0);
}

}  // namespace

// -----------------------------------------------------------------------------------------
// Upstream's `utilities/segment.h`, on the port's own record type.
// -----------------------------------------------------------------------------------------
namespace {

using SegmentEndpoints = std::pair<Eigen::Vector2d, Eigen::Vector2d>;

// Segment::Segment(p1, p2) - "swap if not counter-clockwise". The condition is on the cross
// product of the two points AS POSITION VECTORS from the sensor, not on the segment direction.
SegmentEndpoints OrderEndpoints(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
  if (Cross(p1, p2) > 0.0) return {p1, p2};
  return {p2, p1};
}

// Segment::projection(p) - onto the INFINITE line. Upstream divides a Point by
// `a.lengthSquared()`, which for a zero-length segment yields the zero point and therefore a
// projection at `first`, not a NaN.
Eigen::Vector2d ProjectOnLine(const Eigen::Vector2d& first, const Eigen::Vector2d& last,
                              const Eigen::Vector2d& p) {
  const Eigen::Vector2d a = last - first;
  const Eigen::Vector2d b = p - first;
  return first + DivideOrZero(a.dot(b) * a, a.squaredNorm());
}

double DistanceToLine(const Eigen::Vector2d& first, const Eigen::Vector2d& last,
                      const Eigen::Vector2d& p) {
  return (p - ProjectOnLine(first, last, p)).norm();
}

// Segment::trueDistanceTo(p) - to the SEGMENT, clamped at both ends. Upstream divides two
// doubles here rather than a Point by a double, so unlike projection() a zero-length segment
// gives 0/0 and the NaN propagates. Reproduced, not repaired: the two functions genuinely
// differ upstream and the merge test is the one that calls this one.
double TrueDistanceTo(const Eigen::Vector2d& first, const Eigen::Vector2d& last,
                      const Eigen::Vector2d& p) {
  const Eigen::Vector2d a = last - first;
  const Eigen::Vector2d b = p - first;
  const Eigen::Vector2d c = p - last;
  const double t = a.dot(b) / a.squaredNorm();
  if (t < 0.0) return b.norm();
  if (t > 1.0) return c.norm();
  return (p - (first + t * a)).norm();
}

}  // namespace

const char* SegmentCircleDetectorParams::Validate() const {
  if (min_group_points < 2) return "min_group_points must be >= 2 (a line fit needs two points)";
  if (!IsFinitePositive(max_group_distance_m)) return "max_group_distance_m must be > 0";
  if (!IsFinitePositive(distance_proportion) || distance_proportion >= 1.0) {
    return "distance_proportion must lie in (0, 1)";
  }
  if (!IsFinitePositive(max_split_distance_m)) return "max_split_distance_m must be > 0";
  if (!IsFinitePositive(max_merge_separation_m)) return "max_merge_separation_m must be > 0";
  if (!IsFinitePositive(max_merge_spread_m)) return "max_merge_spread_m must be > 0";
  if (!IsFinitePositive(max_circle_radius_m)) return "max_circle_radius_m must be > 0";
  if (!IsFiniteNonNegative(radius_enlargement_m)) return "radius_enlargement_m must be >= 0";
  if (!(radius_enlargement_m < max_circle_radius_m)) {
    return "radius_enlargement_m must be < max_circle_radius_m, or every circle is rejected";
  }
  if (max_clusters < 1) return "max_clusters must be >= 1";
  if (max_primitives < max_clusters) return "max_primitives must be >= max_clusters";
  if (max_circles < 1) return "max_circles must be >= 1";
  return nullptr;
}

SegmentCircleDetector::SegmentCircleDetector(const SegmentCircleDetectorParams& params,
                                             const UpstreamQuirks& quirks)
    : params_(params), quirks_(quirks) {
  Reserve();
}

void SegmentCircleDetector::Reserve() {
  clusters_.reserve(static_cast<std::size_t>(params_.max_clusters));
  // Every emitted primitive owns at least one span and a merge concatenates them, so the arena
  // is sized generously rather than exactly - it is the one buffer whose high-water mark is
  // not a configured maximum.
  spans_.reserve(static_cast<std::size_t>(params_.max_primitives) * 4u);
  segments_.reserve(static_cast<std::size_t>(params_.max_primitives));
  circles_.reserve(static_cast<std::size_t>(params_.max_circles));
  concat_scratch_.reserve(64);
}

// -----------------------------------------------------------------------------------------
// Ingestion. Upstream's `scanCallback` unrolls the polar scan by ACCUMULATING
// `phi += angle_increment`; this uses ProjectedScan::AngleAt, which is
// `angle_min + i * increment`. The two differ by accumulated rounding, up to ~1e-13 rad at the
// far end of a 360-bin sweep. The contract already fixes the second form as the bin-centre
// convention (projected_scan.h), and the regression harness hands the SAME point list to both
// cores, so the choice cannot leak into the upstream comparison.
// -----------------------------------------------------------------------------------------
void SegmentCircleDetector::BuildPointList(const ProjectedScan& scan) {
  points_.clear();
  point_bins_.clear();
  const int bins = scan.bins();
  for (int bin = 0; bin < bins; ++bin) {
    if (!scan.HasReturn(bin)) continue;
    const double range = static_cast<double>(scan.ranges[static_cast<std::size_t>(bin)]);
    const double angle = scan.AngleAt(bin);
    points_.emplace_back(range * std::cos(angle), range * std::sin(angle));
    point_bins_.push_back(bin);
  }
}

// -----------------------------------------------------------------------------------------
// groupPoints (obstacle_extractor.cpp:173-214).
// -----------------------------------------------------------------------------------------
void SegmentCircleDetector::GroupPoints() {
  const auto point_count = static_cast<int32_t>(points_.size());
  if (point_count == 0) return;

  const double sin_dp = std::sin(2.0 * params_.distance_proportion);

  PointSpan set;
  set.first = 0;
  set.last = 0;
  set.recorded_count = 1;
  set.front_visible = true;
  set.back_visible = true;

  // Q1: upstream's loop starts at the first point rather than the second, so the first point is
  // compared against itself and counted twice. See the header.
  const int32_t start = quirks_.enabled ? 0 : 1;

  for (int32_t i = start; i < point_count; ++i) {
    const double range = points_[static_cast<std::size_t>(i)].norm();
    const double distance =
        (points_[static_cast<std::size_t>(i)] - points_[static_cast<std::size_t>(set.last)]).norm();

    if (distance < params_.max_group_distance_m + range * params_.distance_proportion) {
      set.last = i;
      ++set.recorded_count;
      continue;
    }

    const double prev_range = points_[static_cast<std::size_t>(set.last)].norm();

    // Heron's formula for the area of the triangle (sensor, previous point, this point), used
    // to recover the sine of the angle between the two beams without an atan2. The product can
    // go slightly negative for a near-degenerate triangle and upstream does not guard it, so
    // sin_d becomes NaN and both comparisons below take their false branch. Reproduced
    // verbatim: clamping the product to zero would take the OTHER branch and silently change
    // the visibility decision on exactly the inputs where it is least obvious.
    const double p = (range + prev_range + distance) / 2.0;
    const double area = std::sqrt(p * (p - range) * (p - prev_range) * (p - distance));
    const double sin_d = 2.0 * area / (range * prev_range);

    // OCCLUSION BOUNDARY: the two beams are adjacent, so the range jump between them is a step
    // in depth rather than a gap in the scan - one of the two sides is an edge the sensor
    // cannot see past. Counted here, on the geometric condition alone, because the boundary is
    // an occlusion in BOTH directions; which SIDE it makes untrustworthy is the separate
    // question the two flags below answer, and upstream answers it by marking the FARTHER side.
    if (std::abs(sin_d) < sin_dp) ++stats_->occluded_boundaries;

    // The closing boundary. Upstream ANDs this into a single `is_visible`; the contract splits
    // it per endpoint, and `front_visible && back_visible` is exactly upstream's flag.
    if (std::abs(sin_d) < sin_dp && range < prev_range) set.back_visible = false;

    DetectSegments(set);

    set.first = i;
    set.last = i;
    set.recorded_count = 1;
    set.front_visible = (std::abs(sin_d) > sin_dp || range < prev_range);
    set.back_visible = true;
  }

  DetectSegments(set);  // The last set is closed by the end of the scan, never by a break.
}

// -----------------------------------------------------------------------------------------
// detectSegments (obstacle_extractor.cpp:216-270). Recursive split-and-merge.
// -----------------------------------------------------------------------------------------
void SegmentCircleDetector::DetectSegments(const PointSpan& span) {
  PointSpan resolved = span;

  // A top-level group is registered as a Cluster2D whatever happens to it afterwards; the
  // subsets a split produces are not clusters, they are pieces of one.
  if (resolved.cluster_index < 0) {
    Cluster2D cluster;
    cluster.first_index = resolved.first;
    cluster.last_index = resolved.last;
    // The TRUE span, never the recorded count: a Cluster2D whose point_count disagreed with its
    // index span would not satisfy its own contract, and in quirk mode the recorded count does.
    cluster.point_count = resolved.last - resolved.first + 1;
    cluster.first_bin = point_bins_[static_cast<std::size_t>(resolved.first)];
    cluster.last_bin = point_bins_[static_cast<std::size_t>(resolved.last)];
    cluster.front_visible = resolved.front_visible;
    cluster.back_visible = resolved.back_visible;
    resolved.cluster_index = static_cast<int32_t>(clusters_.size());
    clusters_.push_back(cluster);
    ++stats_->clusters;
  }

  if (resolved.recorded_count < params_.min_group_points) {
    ++stats_->clusters_below_min_points;
    return;
  }

  // Iterative-end-point-fit seed. Only its LINE is used below, so the endpoint swap in the
  // constructor cannot change the divider search; it is applied anyway because when
  // `use_split_and_merge` is off this same segment is what gets emitted.
  SegmentEndpoints endpoints = OrderEndpoints(points_[static_cast<std::size_t>(resolved.first)],
                                              points_[static_cast<std::size_t>(resolved.last)]);
  SegmentRecord segment;
  segment.first_point = endpoints.first;
  segment.last_point = endpoints.second;

  if (params_.use_split_and_merge) segment = FitSegment(&resolved, 1);

  // Seek the point of division. Upstream's loop runs while `point != point_set.end`, so the
  // LAST point of the span is never a candidate divider, and `point_index` is 1-based.
  double max_distance = 0.0;
  int32_t split_index = 0;
  for (int32_t i = resolved.first; i < resolved.last; ++i) {
    const int32_t point_index = i - resolved.first + 1;
    const double distance =
        DistanceToLine(segment.first_point, segment.last_point, points_[static_cast<std::size_t>(i)]);
    if (distance >= max_distance) {
      const double r = points_[static_cast<std::size_t>(i)].norm();
      if (distance > params_.max_split_distance_m + r * params_.distance_proportion) {
        max_distance = distance;
        split_index = point_index;
      }
    }
  }

  const bool split = max_distance > 0.0 && split_index > params_.min_group_points &&
                     split_index < resolved.recorded_count - params_.min_group_points;

  if (split) {
    ++stats_->splits;

    // Upstream CLONES the dividing point into the list so both halves own a copy. Sharing the
    // index is the same thing without the insertion.
    const int32_t divider = resolved.first + split_index - 1;

    PointSpan first_half = resolved;
    first_half.last = divider;
    first_half.recorded_count = split_index;

    PointSpan second_half = resolved;
    second_half.first = divider;
    // Q2: upstream records `num_points - split_index`, one short of the span it actually owns,
    // because the cloned point is not counted. See the header.
    second_half.recorded_count = quirks_.enabled ? resolved.recorded_count - split_index
                                                 : resolved.last - divider + 1;

    DetectSegments(first_half);
    DetectSegments(second_half);
    return;
  }

  if (!params_.use_split_and_merge) segment = FitSegment(&resolved, 1);

  segment.span_begin = static_cast<int32_t>(spans_.size());
  segment.span_count = 1;
  spans_.push_back(resolved);
  segments_.push_back(segment);
}

// -----------------------------------------------------------------------------------------
// fitSegment (figure_fitting.h:54-164). One implementation for both upstream overloads: they
// differ only in how they walk to the points.
// -----------------------------------------------------------------------------------------
SegmentCircleDetector::SegmentRecord SegmentCircleDetector::FitSegment(const PointSpan* spans,
                                                                       int32_t span_count) {
  accumulator_.Reset();
  double max_range = 0.0;
  const auto available = static_cast<int32_t>(points_.size());
  for (int32_t s = 0; s < span_count; ++s) {
    const PointSpan& span = spans[s];
    for (int32_t k = 0; k < span.recorded_count; ++k) {
      // The clamp exists for the Q1 quirk alone: an over-counted first group would otherwise
      // read past the end of the point list, which is what upstream actually does when the
      // whole scan is one group. Reproducing the extra point is fidelity; reproducing the
      // out-of-bounds read is not.
      const int32_t index = span.first + k;
      if (index >= available) break;
      const Eigen::Vector2d& point = points_[static_cast<std::size_t>(index)];
      accumulator_.Add(point.x(), point.y());
      max_range = std::max(max_range, point.norm());
    }
  }

  const LineFit2D fit = accumulator_.Solve();

  const Eigen::Vector2d& p1 = points_[static_cast<std::size_t>(spans[0].first)];
  const Eigen::Vector2d& p2 = points_[static_cast<std::size_t>(spans[span_count - 1].last)];

  SegmentEndpoints endpoints = OrderEndpoints(p1, p2);
  SegmentRecord segment;
  segment.first_point = endpoints.first;
  segment.last_point = endpoints.second;
  segment.degenerate_fit = fit.IsDegenerate();
  if (fit.IsDegenerate()) ++stats_->degenerate_fits;

  // Project the endpoints onto the fitted line. Note that this OVERWRITES the constructor's
  // counter-clockwise swap with the original (p1, p2) order, so the swap survives only on the
  // `D == 0` branch - a faithful oddity of upstream, not a simplification.
  const double d = fit.D();

  // The line `a*x + b*y = 1` lies at distance 1/sqrt(D) from the origin. When that distance
  // dwarfs the data the line is "at infinity" relative to the points it was fitted to, and the
  // projection below is arithmetic on noise. Counted, never repaired - see the field's comment
  // in i_detector_2d.h.
  if (d > 0.0 && 1.0 / std::sqrt(d) > kIllConditionedProjectionFactor * std::max(max_range, 1e-12)) {
    ++stats_->ill_conditioned_projections;
  }

  if (d > 0.0) {
    const double a = fit.a;
    const double b = fit.b;
    const double c = LineFit2D::kC;
    segment.first_point = Eigen::Vector2d(
        (b * b * p1.x() - a * b * p1.y() - a * c) / d,
        (-a * b * p1.x() + a * a * p1.y() - b * c) / d);
    segment.last_point = Eigen::Vector2d(
        (b * b * p2.x() - a * b * p2.y() - a * c) / d,
        (-a * b * p2.x() + a * a * p2.y() - b * c) / d);
  }
  return segment;
}

// -----------------------------------------------------------------------------------------
// mergeSegments / compareSegments (obstacle_extractor.cpp:272-324).
//
// Upstream drives a std::list with iterator surgery: insert the merged segment before `i`,
// erase `i` and `j`, then step back so the new segment is re-checked against everything.
// The index form below visits the same pairs in the same order, including the re-check.
// -----------------------------------------------------------------------------------------
void SegmentCircleDetector::MergeSegments() {
  for (int32_t i = 0; i < static_cast<int32_t>(segments_.size()); ++i) {
    for (int32_t j = i; j < static_cast<int32_t>(segments_.size()); ++j) {
      SegmentRecord merged;
      if (!CompareSegments(i, j, &merged)) continue;
      segments_[static_cast<std::size_t>(i)] = merged;
      segments_.erase(segments_.begin() + j);
      ++stats_->segment_merges;
      --i;  // The loop's ++i lands back on the merged segment.
      break;
    }
  }
}

bool SegmentCircleDetector::CompareSegments(int32_t i, int32_t j, SegmentRecord* merged) {
  if (i == j) return false;

  const SegmentRecord& s1 = segments_[static_cast<std::size_t>(i)];
  const SegmentRecord& s2 = segments_[static_cast<std::size_t>(j)];

  // "Segments must be provided counter-clockwise" - upstream recurses with the arguments
  // swapped, which also swaps the order the point sets are concatenated in and therefore which
  // endpoints the refit projects onto.
  if (Cross(s1.first_point, s2.first_point) < 0.0) return CompareSegments(j, i, merged);

  const bool near =
      TrueDistanceTo(s1.first_point, s1.last_point, s2.first_point) < params_.max_merge_separation_m ||
      TrueDistanceTo(s1.first_point, s1.last_point, s2.last_point) < params_.max_merge_separation_m ||
      TrueDistanceTo(s2.first_point, s2.last_point, s1.first_point) < params_.max_merge_separation_m ||
      TrueDistanceTo(s2.first_point, s2.last_point, s1.last_point) < params_.max_merge_separation_m;
  if (!near) return false;

  concat_scratch_.clear();
  for (int32_t k = 0; k < s1.span_count; ++k) {
    concat_scratch_.push_back(spans_[static_cast<std::size_t>(s1.span_begin + k)]);
  }
  for (int32_t k = 0; k < s2.span_count; ++k) {
    concat_scratch_.push_back(spans_[static_cast<std::size_t>(s2.span_begin + k)]);
  }

  SegmentRecord fitted =
      FitSegment(concat_scratch_.data(), static_cast<int32_t>(concat_scratch_.size()));

  const bool collinear =
      DistanceToLine(fitted.first_point, fitted.last_point, s1.first_point) < params_.max_merge_spread_m &&
      DistanceToLine(fitted.first_point, fitted.last_point, s1.last_point) < params_.max_merge_spread_m &&
      DistanceToLine(fitted.first_point, fitted.last_point, s2.first_point) < params_.max_merge_spread_m &&
      DistanceToLine(fitted.first_point, fitted.last_point, s2.last_point) < params_.max_merge_spread_m;
  if (!collinear) return false;

  fitted.span_begin = static_cast<int32_t>(spans_.size());
  fitted.span_count = static_cast<int32_t>(concat_scratch_.size());
  spans_.insert(spans_.end(), concat_scratch_.begin(), concat_scratch_.end());
  *merged = fitted;
  return true;
}

// -----------------------------------------------------------------------------------------
// detectCircles (obstacle_extractor.cpp:326-352) - the sqrt(3)/3 circumcircle rule.
// -----------------------------------------------------------------------------------------
void SegmentCircleDetector::DetectCircles() {
  std::size_t kept = 0;
  for (std::size_t s = 0; s < segments_.size(); ++s) {
    const SegmentRecord& segment = segments_[s];

    if (params_.circles_from_visibles) {
      bool visible = true;
      for (int32_t k = 0; k < segment.span_count; ++k) {
        const PointSpan& span = spans_[static_cast<std::size_t>(segment.span_begin + k)];
        if (!span.front_visible || !span.back_visible) {
          visible = false;
          break;
        }
      }
      if (!visible) {
        ++stats_->circles_rejected_visibility;
        segments_[kept++] = segment;
        continue;
      }
    }

    ++stats_->circles_attempted;

    // Circle(const Segment&): the circumcircle of the equilateral triangle standing on the
    // segment. The literal is upstream's TRUNCATED 0.5773502, not sqrt(3)/3 = 0.5773502691...;
    // the seventh decimal is a 6.9e-8 relative difference and it is kept so the two cores
    // produce the same number rather than nearly the same one.
    const Eigen::Vector2d direction = segment.last_point - segment.first_point;
    const double radius_raw = 0.5773502 * direction.norm();
    const Eigen::Vector2d normal = NormalizedOrSelf(Perpendicular(direction));
    const Eigen::Vector2d center = (segment.first_point + segment.last_point - radius_raw * normal) / 2.0;
    const double radius = radius_raw + params_.radius_enlargement_m;

    if (!(radius < params_.max_circle_radius_m)) {
      ++stats_->circles_rejected_radius;
      segments_[kept++] = segment;
      continue;
    }

    CircleRecord circle;
    circle.center = center;
    circle.radius = radius;
    circle.radius_unenlarged = radius_raw;
    circle.span_begin = segment.span_begin;
    circle.span_count = segment.span_count;
    circles_.push_back(circle);

    if (!params_.discard_converted_segments) segments_[kept++] = segment;
  }
  segments_.resize(kept);
}

// -----------------------------------------------------------------------------------------
// mergeCircles / compareCircles (obstacle_extractor.cpp:354-403).
// -----------------------------------------------------------------------------------------
void SegmentCircleDetector::MergeCircles() {
  for (int32_t i = 0; i < static_cast<int32_t>(circles_.size()); ++i) {
    for (int32_t j = i; j < static_cast<int32_t>(circles_.size()); ++j) {
      if (i == j) continue;

      const CircleRecord& c1 = circles_[static_cast<std::size_t>(i)];
      const CircleRecord& c2 = circles_[static_cast<std::size_t>(j)];
      const double separation = (c2.center - c1.center).norm();

      CircleRecord merged;
      bool did_merge = false;

      if (c2.radius - c1.radius >= separation) {
        merged = c2;  // c1 is fully inside c2; upstream keeps c2, point sets and all.
        did_merge = true;
      } else if (c1.radius - c2.radius >= separation) {
        merged = c1;
        did_merge = true;
      } else if (c1.radius + c2.radius >= separation) {
        // Upstream's overlap rule, verbatim including the second enlargement: the merged
        // radius is the distance from the new centre to c1's rim PLUS the larger of the two
        // radii. That is deliberately generous and it is not re-derived here.
        const Eigen::Vector2d center =
            c1.center + DivideOrZero((c2.center - c1.center) * c1.radius, c1.radius + c2.radius);
        double radius = (c1.center - center).norm() + c1.radius;
        radius += std::max(c1.radius, c2.radius);

        if (radius < params_.max_circle_radius_m) {
          merged.center = center;
          merged.radius = radius;
          // The SAME rule on the bare radii. Not `radius - enlargement`: the expression above
          // adds c1.radius and max(c1.radius, c2.radius), so a merged radius carries the
          // enlargement twice and a single subtraction would leave one of them behind. The
          // merge DECISION and the merged centre are still taken on the enlarged radii, exactly
          // as upstream does - only the carried bare radius is computed here.
          merged.radius_unenlarged =
              (c1.center - center).norm() + c1.radius_unenlarged +
              std::max(c1.radius_unenlarged, c2.radius_unenlarged);
          merged.span_begin = static_cast<int32_t>(spans_.size());
          merged.span_count = c1.span_count + c2.span_count;
          for (int32_t k = 0; k < c1.span_count; ++k) {
            spans_.push_back(spans_[static_cast<std::size_t>(c1.span_begin + k)]);
          }
          for (int32_t k = 0; k < c2.span_count; ++k) {
            spans_.push_back(spans_[static_cast<std::size_t>(c2.span_begin + k)]);
          }
          did_merge = true;
        }
      }

      if (!did_merge) continue;

      circles_[static_cast<std::size_t>(i)] = merged;
      circles_.erase(circles_.begin() + j);
      ++stats_->circle_merges;
      --i;
      break;
    }
  }
}

// -----------------------------------------------------------------------------------------
// Contract emission. Everything above works in upstream's types; this is where the port's own
// contracts are filled, including the fields upstream has no equivalent for.
// -----------------------------------------------------------------------------------------
namespace {

// Wrap into (-pi, pi].
double WrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle <= -kPi) angle += 2.0 * kPi;
  return angle;
}

}  // namespace

void SegmentCircleDetector::EmitPrimitives(const ProjectedScan& scan, Detection2DResult* out) {
  out->clusters = clusters_;

  const auto point_of = [this](int32_t index) -> const Eigen::Vector2d& {
    return points_[static_cast<std::size_t>(index)];
  };

  // Segments first, then circles: CircleObservation::primitive_index addresses the kCircle
  // entry, so the circle's own primitive index is `segment_count + circle_ordinal`.
  for (const SegmentRecord& segment : segments_) {
    FittedPrimitive2D primitive;
    primitive.kind = PrimitiveKind::kSegment;
    primitive.first_point = segment.first_point;
    primitive.last_point = segment.last_point;
    primitive.normal = NormalizedOrSelf(Perpendicular(segment.last_point - segment.first_point));

    double residual_sq = 0.0;
    int32_t count = 0;
    bool front_visible = true;
    bool back_visible = true;
    int32_t cluster_index = -1;
    for (int32_t k = 0; k < segment.span_count; ++k) {
      const PointSpan& span = spans_[static_cast<std::size_t>(segment.span_begin + k)];
      if (cluster_index < 0) cluster_index = span.cluster_index;
      front_visible = front_visible && span.front_visible;
      back_visible = back_visible && span.back_visible;
      for (int32_t index = span.first; index <= span.last; ++index) {
        const double distance =
            DistanceToLine(segment.first_point, segment.last_point, point_of(index));
        residual_sq += distance * distance;
        ++count;
      }
    }
    primitive.point_count = count;
    primitive.cluster_index = cluster_index;
    primitive.front_visible = front_visible;
    primitive.back_visible = back_visible;
    primitive.fit_residual_m = count > 0 ? std::sqrt(residual_sq / static_cast<double>(count)) : 0.0;

    if (primitive.IsValid()) {
      out->primitives.push_back(primitive);
    } else {
      ++stats_->primitives_rejected_invalid;
    }
  }

  const auto segment_primitives = static_cast<int32_t>(out->primitives.size());

  for (const CircleRecord& circle : circles_) {
    double residual_sq = 0.0;
    double enclosing = 0.0;
    int32_t count = 0;
    bool front_visible = true;
    bool back_visible = true;
    int32_t cluster_index = -1;
    int32_t min_bin = std::numeric_limits<int32_t>::max();
    int32_t max_bin = std::numeric_limits<int32_t>::min();
    double reference_angle = 0.0;
    double min_offset = 0.0;
    double max_offset = 0.0;
    bool have_reference = false;

    for (int32_t k = 0; k < circle.span_count; ++k) {
      const PointSpan& span = spans_[static_cast<std::size_t>(circle.span_begin + k)];
      if (cluster_index < 0) cluster_index = span.cluster_index;
      front_visible = front_visible && span.front_visible;
      back_visible = back_visible && span.back_visible;
      min_bin = std::min(min_bin, point_bins_[static_cast<std::size_t>(span.first)]);
      max_bin = std::max(max_bin, point_bins_[static_cast<std::size_t>(span.last)]);

      for (int32_t index = span.first; index <= span.last; ++index) {
        const Eigen::Vector2d offset = point_of(index) - circle.center;
        const double distance = offset.norm();
        enclosing = std::max(enclosing, distance);
        // AGAINST THE UN-ENLARGED RADIUS. The contributing points lie on the object's real
        // surface, which the bare circumcircle is an estimate of; `circle.radius` is that
        // estimate plus a deliberate safety pad. Measuring against the padded radius made every
        // residual carry the pad as a constant - on this repository's own corpus the mean
        // residual was 0.3076 m against a 0.2500 m enlargement, 81% of it the constant - so a
        // perfect fit and a poor one reported nearly the same "noise". Corrected here because
        // fit_residual_m's only consumer is the measurement-sigma formula 15 lines below, and a
        // sigma driven by a constant is not a measurement of anything.
        const double error = distance - circle.radius_unenlarged;
        residual_sq += error * error;
        ++count;

        // Angular extent AT THE CIRCLE CENTRE, which is not the sensor-referenced extent - see
        // the two arc definitions in detection_2d.h. Measured as a signed offset from the first
        // contributing point so a span straddling the atan2 branch cut does not read as 2*pi.
        const double angle = std::atan2(offset.y(), offset.x());
        if (!have_reference) {
          reference_angle = angle;
          have_reference = true;
        } else {
          const double delta = WrapAngle(angle - reference_angle);
          min_offset = std::min(min_offset, delta);
          max_offset = std::max(max_offset, delta);
        }
      }
    }

    CircleObservation observation;
    observation.center = circle.center;
    observation.radius_fitted_m = circle.radius;
    observation.radius_enclosing_m = enclosing;
    observation.point_count = count;
    observation.primitive_index = segment_primitives + static_cast<int32_t>(out->circles.size());
    observation.front_visible = front_visible;
    observation.back_visible = back_visible;
    observation.stamp_s = scan.stamp_s;
    observation.frame = scan.frame;
    observation.range_to_center_m = circle.center.norm();
    observation.fit_residual_m =
        count > 0 ? std::sqrt(residual_sq / static_cast<double>(count)) : 0.0;
    observation.arc_extent_rad =
        count > 0 ? static_cast<double>(max_bin - min_bin) * scan.angle_increment_rad : 0.0;
    const double centre_angle = std::min(max_offset - min_offset, 2.0 * kPi);
    observation.arc_length_m = circle.radius * centre_angle;

    // MEASUREMENT SIGMAS - PROVISIONAL, and named as such rather than presented as derived.
    // The scale is the per-point fit residual reduced by the point count; the multiplier is the
    // geometric dilution of a circle centre estimated from an arc of half-angle alpha, which
    // diverges as 1/(1 - cos alpha) as the arc shortens. Both the residual floor and the
    // dilution cap are placeholders: the tracking phase is what calibrates these against
    // measured innovation statistics, and the short-arc bias this phase measures is what tells
    // it whether a bias correction belongs here at all.
    const double sigma_base =
        std::max(observation.fit_residual_m, kProvisionalResidualFloorM) /
        std::sqrt(static_cast<double>(count > 0 ? count : 1));
    const double half_angle = 0.5 * centre_angle;
    const double dilution =
        1.0 / std::max(1.0 - std::cos(half_angle), kProvisionalMinDilutionTerm);
    observation.sigma_center_m = sigma_base * dilution;
    observation.sigma_radius_m = sigma_base * dilution;

    // The circle's own primitive. Emitted alongside the observation rather than instead of it:
    // the observation is what the tracker consumes, the primitive is what the visualization and
    // the dumps address, and CircleObservation::primitive_index is the link between them - so
    // the two have to be pushed together or the index names a segment.
    FittedPrimitive2D primitive;
    primitive.kind = PrimitiveKind::kCircle;
    primitive.center = circle.center;
    primitive.radius_m = circle.radius;
    primitive.fit_residual_m = observation.fit_residual_m;
    primitive.point_count = count;
    primitive.cluster_index = cluster_index;
    primitive.front_visible = front_visible;
    primitive.back_visible = back_visible;

    if (observation.IsValid() && primitive.IsValid()) {
      out->primitives.push_back(primitive);
      out->circles.push_back(observation);
      ++stats_->circles;
    } else {
      if (!primitive.IsValid()) ++stats_->primitives_rejected_invalid;
      ++stats_->circles_rejected_invalid;
    }
  }
}

const char* SegmentCircleDetector::Detect(const ProjectedScan& scan, Detection2DResult* out) {
  if (out == nullptr) return "Detect: the result pointer must not be null";
  if (const char* reason = params_.Validate()) return reason;
  if (const char* reason = scan.Validate()) return reason;

  const auto started = std::chrono::steady_clock::now();

  out->Clear();
  stats_ = &out->stats;

  // Per-frame state only: everything is cleared here, exactly as upstream clears its
  // containers at the top of processPoints().
  clusters_.clear();
  spans_.clear();
  segments_.clear();
  circles_.clear();

  BuildPointList(scan);
  stats_->input_points = static_cast<int32_t>(points_.size());

  GroupPoints();
  stats_->segments_before_merge = static_cast<int32_t>(segments_.size());

  MergeSegments();
  stats_->segments_after_merge = static_cast<int32_t>(segments_.size());

  DetectCircles();
  MergeCircles();
  EmitPrimitives(scan, out);

  stats_->capacity_exceeded =
      clusters_.capacity() > static_cast<std::size_t>(params_.max_clusters) ||
      segments_.capacity() > static_cast<std::size_t>(params_.max_primitives) ||
      circles_.capacity() > static_cast<std::size_t>(params_.max_circles);

  stats_->wall_time_us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - started).count();
  stats_ = nullptr;
  return nullptr;
}

}  // namespace perception::core
