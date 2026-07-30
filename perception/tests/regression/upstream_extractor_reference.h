// The upstream obstacle_extractor detection pipeline, transcribed VERBATIM as a regression
// oracle. TEST-ONLY. Nothing here is ever linked into production.
//
// This is the "upstream ROS-free headers compiled in a test-only target (linking Armadillo)"
// row of the architecture doc's adoption table (section 4.2), realised. The types, the
// arithmetic and the pseudo-inverse are upstream's own headers - obstacle_detector's
// utilities/{point,point_set,segment,circle,figure_fitting}.h, which the audit established are
// ROS-free - and the pipeline below is obstacle_extractor.cpp:158-403 with three, and only
// three, deletions:
//
//   * the ROS message parsing, the tf transform and the publisher (obstacle_extractor.cpp:132
//     -156 and 405-463). The oracle is handed a point list directly, so nothing about the
//     comparison depends on how either side unrolled the scan.
//   * `p_transform_coordinates_` and the x/y limit filtering, which act after detection.
//   * the `static` on `groupPoints`'s `sin_dp`. Upstream computes it once per PROCESS, so a
//     second extractor with a different `distance_proportion` silently uses the first one's
//     value. Reproducing that here would make this harness order-dependent - the second test
//     case in a binary would get the first one's threshold - which is a property of upstream's
//     lifetime model, not of its algorithm. Every call recomputes it.
//
// EVERYTHING ELSE IS CHARACTER-FOR-CHARACTER UPSTREAM, INCLUDING ITS DEFECTS. In particular
// `input_points_.begin()++` is preserved exactly as written (it evaluates to `begin()`, see
// segment_circle_detector.h Q1), as is the `num_points - split_index` undercount of a split's
// second half (Q2). That is the entire point of this file: an oracle that quietly fixed what
// it was oracling would certify the port against something that never shipped.
//
// LICENSE: BSD-3, Poznan University of Technology, 2017, Mateusz Przybyla. See
// perception/third_party_notices.md.
#ifndef PERCEPTION_TESTS_REGRESSION_UPSTREAM_EXTRACTOR_REFERENCE_H_
#define PERCEPTION_TESTS_REGRESSION_UPSTREAM_EXTRACTOR_REFERENCE_H_

#include <cmath>
#include <list>
#include <vector>

#include "obstacle_detector/utilities/circle.h"
#include "obstacle_detector/utilities/figure_fitting.h"
#include "obstacle_detector/utilities/point.h"
#include "obstacle_detector/utilities/point_set.h"
#include "obstacle_detector/utilities/segment.h"

namespace upstream_reference {

using obstacle_detector::Circle;
using obstacle_detector::Point;
using obstacle_detector::PointIterator;
using obstacle_detector::PointSet;
using obstacle_detector::Segment;

// Upstream's parameter block, with upstream's own defaults.
struct Params {
  bool use_split_and_merge = true;
  bool circles_from_visibles = true;
  bool discard_converted_segments = true;

  int min_group_points = 5;

  double distance_proportion = 0.00628;
  double max_group_distance = 0.1;
  double max_split_distance = 0.2;
  double max_merge_separation = 0.2;
  double max_merge_spread = 0.2;
  double max_circle_radius = 0.6;
  double radius_enlargement = 0.25;
};

class Extractor {
 public:
  explicit Extractor(const Params& params) : p_(params) {}

  // The whole pipeline, on a caller-supplied point list. `processPoints`, minus publishing.
  void Process(const std::vector<Point>& points) {
    input_points_.assign(points.begin(), points.end());
    segments_.clear();
    circles_.clear();

    groupPoints();
    mergeSegments();

    detectCircles();
    mergeCircles();
  }

  const std::list<Segment>& segments() const { return segments_; }
  const std::list<Circle>& circles() const { return circles_; }

 private:
  void groupPoints() {
    double sin_dp = sin(2.0 * p_.distance_proportion);

    PointSet point_set;
    point_set.begin = input_points_.begin();
    point_set.end = input_points_.begin();
    point_set.num_points = 1;
    point_set.is_visible = true;

    for (PointIterator point = input_points_.begin()++; point != input_points_.end(); ++point) {
      double range = (*point).length();
      double distance = (*point - *point_set.end).length();

      if (distance < p_.max_group_distance + range * p_.distance_proportion) {
        point_set.end = point;
        point_set.num_points++;
      } else {
        double prev_range = (*point_set.end).length();

        // Heron's equation
        double p = (range + prev_range + distance) / 2.0;
        double S = sqrt(p * (p - range) * (p - prev_range) * (p - distance));
        double sin_d = 2.0 * S / (range * prev_range);  // Sine of angle between beams

        if (std::abs(sin_d) < sin_dp && range < prev_range) point_set.is_visible = false;

        detectSegments(point_set);

        // Begin new point set
        point_set.begin = point;
        point_set.end = point;
        point_set.num_points = 1;
        point_set.is_visible = (std::abs(sin_d) > sin_dp || range < prev_range);
      }
    }

    detectSegments(point_set);  // Check the last point set too!
  }

  void detectSegments(const PointSet& point_set) {
    if (point_set.num_points < p_.min_group_points) return;

    Segment segment(*point_set.begin, *point_set.end);  // Use Iterative End Point Fit

    if (p_.use_split_and_merge) segment = fitSegment(point_set);

    PointIterator set_divider;
    double max_distance = 0.0;
    double distance = 0.0;

    int split_index = 0;  // Natural index of splitting point (counting from 1)
    int point_index = 0;  // Natural index of current point in the set

    // Seek the point of division
    for (PointIterator point = point_set.begin; point != point_set.end; ++point) {
      ++point_index;

      if ((distance = segment.distanceTo(*point)) >= max_distance) {
        double r = (*point).length();

        if (distance > p_.max_split_distance + r * p_.distance_proportion) {
          max_distance = distance;
          set_divider = point;
          split_index = point_index;
        }
      }
    }

    // Split the set only if the sub-groups are not 'small'
    if (max_distance > 0.0 && split_index > p_.min_group_points &&
        split_index < point_set.num_points - p_.min_group_points) {
      set_divider = input_points_.insert(set_divider, *set_divider);  // Clone the dividing point

      PointSet subset1, subset2;
      subset1.begin = point_set.begin;
      subset1.end = set_divider;
      subset1.num_points = split_index;
      subset1.is_visible = point_set.is_visible;

      subset2.begin = ++set_divider;
      subset2.end = point_set.end;
      subset2.num_points = point_set.num_points - split_index;
      subset2.is_visible = point_set.is_visible;

      detectSegments(subset1);
      detectSegments(subset2);
    } else {  // Add the segment
      if (!p_.use_split_and_merge) segment = fitSegment(point_set);

      segments_.push_back(segment);
    }
  }

  void mergeSegments() {
    for (auto i = segments_.begin(); i != segments_.end(); ++i) {
      for (auto j = i; j != segments_.end(); ++j) {
        Segment merged_segment;

        if (compareSegments(*i, *j, merged_segment)) {
          auto temp_itr = segments_.insert(i, merged_segment);
          segments_.erase(i);
          segments_.erase(j);
          i = --temp_itr;  // Check the new segment against others
          break;
        }
      }
    }
  }

  bool compareSegments(const Segment& s1, const Segment& s2, Segment& merged_segment) {
    if (&s1 == &s2) return false;

    // Segments must be provided counter-clockwise
    if (s1.first_point.cross(s2.first_point) < 0.0) return compareSegments(s2, s1, merged_segment);

    if (checkSegmentsProximity(s1, s2)) {
      std::vector<PointSet> point_sets;
      point_sets.insert(point_sets.end(), s1.point_sets.begin(), s1.point_sets.end());
      point_sets.insert(point_sets.end(), s2.point_sets.begin(), s2.point_sets.end());

      Segment segment = fitSegment(point_sets);

      if (checkSegmentsCollinearity(segment, s1, s2)) {
        merged_segment = segment;
        return true;
      }
    }

    return false;
  }

  bool checkSegmentsProximity(const Segment& s1, const Segment& s2) {
    return (s1.trueDistanceTo(s2.first_point) < p_.max_merge_separation ||
            s1.trueDistanceTo(s2.last_point) < p_.max_merge_separation ||
            s2.trueDistanceTo(s1.first_point) < p_.max_merge_separation ||
            s2.trueDistanceTo(s1.last_point) < p_.max_merge_separation);
  }

  bool checkSegmentsCollinearity(const Segment& segment, const Segment& s1, const Segment& s2) {
    return (segment.distanceTo(s1.first_point) < p_.max_merge_spread &&
            segment.distanceTo(s1.last_point) < p_.max_merge_spread &&
            segment.distanceTo(s2.first_point) < p_.max_merge_spread &&
            segment.distanceTo(s2.last_point) < p_.max_merge_spread);
  }

  void detectCircles() {
    for (auto segment = segments_.begin(); segment != segments_.end(); ++segment) {
      if (p_.circles_from_visibles) {
        bool segment_is_visible = true;
        for (const PointSet& ps : segment->point_sets) {
          if (!ps.is_visible) {
            segment_is_visible = false;
            break;
          }
        }
        if (!segment_is_visible) continue;
      }

      Circle circle(*segment);
      circle.radius += p_.radius_enlargement;

      if (circle.radius < p_.max_circle_radius) {
        circles_.push_back(circle);

        if (p_.discard_converted_segments) {
          segment = segments_.erase(segment);
          --segment;
        }
      }
    }
  }

  void mergeCircles() {
    for (auto i = circles_.begin(); i != circles_.end(); ++i) {
      for (auto j = i; j != circles_.end(); ++j) {
        Circle merged_circle;

        if (compareCircles(*i, *j, merged_circle)) {
          auto temp_itr = circles_.insert(i, merged_circle);
          circles_.erase(i);
          circles_.erase(j);
          i = --temp_itr;
          break;
        }
      }
    }
  }

  bool compareCircles(const Circle& c1, const Circle& c2, Circle& merged_circle) {
    if (&c1 == &c2) return false;

    // If circle c1 is fully inside c2 - merge and leave as c2
    if (c2.radius - c1.radius >= (c2.center - c1.center).length()) {
      merged_circle = c2;
      return true;
    }

    // If circle c2 is fully inside c1 - merge and leave as c1
    if (c1.radius - c2.radius >= (c2.center - c1.center).length()) {
      merged_circle = c1;
      return true;
    }

    // If circles intersect and are 'small' - merge
    if (c1.radius + c2.radius >= (c2.center - c1.center).length()) {
      Point center =
          c1.center + (c2.center - c1.center) * c1.radius / (c1.radius + c2.radius);
      double radius = (c1.center - center).length() + c1.radius;

      Circle circle(center, radius);
      circle.radius += std::max(c1.radius, c2.radius);

      if (circle.radius < p_.max_circle_radius) {
        circle.point_sets.insert(circle.point_sets.end(), c1.point_sets.begin(),
                                 c1.point_sets.end());
        circle.point_sets.insert(circle.point_sets.end(), c2.point_sets.begin(),
                                 c2.point_sets.end());
        merged_circle = circle;
        return true;
      }
    }

    return false;
  }

  Params p_;
  std::list<Point> input_points_;
  std::list<Segment> segments_;
  std::list<Circle> circles_;
};

}  // namespace upstream_reference

#endif  // PERCEPTION_TESTS_REGRESSION_UPSTREAM_EXTRACTOR_REFERENCE_H_
