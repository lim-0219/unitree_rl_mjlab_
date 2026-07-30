// Contract-level tests for every type in perception/core/contracts.
//
// Coverage target: 100% of the section-7 contracts, INCLUDING the five that shipped with
// the Mid-360 bring-up slice. Those five were previously exercised only indirectly, by the
// adapter and bring-up suites; here they are constructed and validated directly, which is
// what makes a change to one of them fail at the contract layer rather than three stages
// downstream.
//
// Each contract gets: a valid construction, at least one validation-failure case with the
// specific reason asserted, and - where the contract is copied or compared anywhere in the
// pipeline - a value-semantics round trip.
//
// No MuJoCo, no yaml-cpp, no dpcbf. Eigen and the STL only, which is itself part of what
// this test asserts: it links against perception_contracts alone.

#include <cmath>
#include <limits>

#include "perception/core/contracts/detection_2d.h"
#include "perception/core/contracts/diagnostics.h"
#include "perception/core/contracts/frame_transform_snapshot.h"
#include "perception/core/contracts/frames.h"
#include "perception/core/contracts/labeled_point_cloud.h"
#include "perception/core/contracts/obstacles.h"
#include "perception/core/contracts/perception_frame.h"
#include "perception/core/contracts/point_clouds.h"
#include "perception/core/contracts/projected_scan.h"
#include "perception/core/contracts/ray_pattern.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/core/contracts/tracking_2d.h"
#include "perception/core/contracts/validation.h"

#include "check.h"

using namespace perception::core;
using perception_test::Check;
using perception_test::CheckInvalid;
using perception_test::CheckValid;
using perception_test::Section;

namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

RawTimedPoint MakePoint(float x, float y, float z, double range, int index) {
  RawTimedPoint point;
  point.position = Eigen::Vector3f(x, y, z);
  point.range_m = range;
  point.time_offset_s = -0.01f;
  point.ray_index = index;
  point.geom_id = 7;
  point.body_id = 3;
  return point;
}

// ---------------------------------------------------------------------------------------
// The five contracts that shipped with the bring-up slice.
// ---------------------------------------------------------------------------------------

void TestFrameId() {
  Section("FrameId");
  Check(std::string(ToString(FrameId::kSensor)) == "sensor", "kSensor names itself");
  Check(std::string(ToString(FrameId::kBase)) == "base", "kBase names itself");
  Check(std::string(ToString(FrameId::kGravityAlignedBase)) == "gravity_aligned_base",
        "kGravityAlignedBase names itself");
  Check(std::string(ToString(FrameId::kWorld)) == "world", "kWorld names itself");
  Check(std::string(ToString(FrameId::kUnknown)) == "unknown", "kUnknown names itself");
  // kUnknown must be the zero value: a default-constructed frame tag has to read as
  // "nobody said", not as a real frame.
  Check(static_cast<int>(FrameId::kUnknown) == 0, "kUnknown is the zero value");
}

void TestRayPattern() {
  Section("RayPattern");
  RayPattern pattern;
  pattern.azimuth_rays = 4;
  pattern.elevation_rays = 2;
  pattern.period_s = 0.1;
  for (int i = 0; i < 8; ++i) {
    pattern.directions.emplace_back(1.0, 0.0, 0.0);
    pattern.time_offsets_s.push_back(0.0125 * i);
  }
  Check(pattern.IsConsistent(), "a fully populated pattern is consistent");
  Check(pattern.size() == 8, "size() is the direction count");
  Check(!pattern.empty(), "a populated pattern is not empty");
  Check(pattern.frame == FrameId::kSensor, "directions are sensor-frame by default");

  // Validation failure: the two parallel arrays must stay the same length. A pattern with
  // more directions than timestamps deskews every extra ray to time zero.
  RayPattern mismatched = pattern;
  mismatched.time_offsets_s.pop_back();
  Check(!mismatched.IsConsistent(), "a pattern with mismatched array lengths is rejected");

  // Validation failure: the grid product must equal the ray count.
  RayPattern miscounted = pattern;
  miscounted.azimuth_rays = 3;
  Check(!miscounted.IsConsistent(), "a pattern whose grid product disagrees is rejected");

  RayPattern zero_period = pattern;
  zero_period.period_s = 0.0;
  Check(!zero_period.IsConsistent(), "a pattern with a zero period is rejected");

  Check(RayPattern{}.empty() && !RayPattern{}.IsConsistent(),
        "a default-constructed pattern is empty and inconsistent");

  // Value semantics: the copy owns its own buffers.
  RayPattern copy = pattern;
  copy.directions[0] = Eigen::Vector3d(0.0, 1.0, 0.0);
  Check(pattern.directions[0].x() == 1.0, "copying a RayPattern deep-copies its directions");
}

void TestTimedPointCloud() {
  Section("TimedPointCloud / RawTimedPoint");
  TimedPointCloud cloud;
  cloud.stamp_s = 1.5;
  cloud.sequence = 12;
  cloud.points.push_back(MakePoint(1.0f, 0.0f, 0.0f, 1.0, 0));
  cloud.points.push_back(MakePoint(0.0f, 2.0f, 0.0f, 2.0, 1));

  Check(cloud.size() == 2, "size() is the point count");
  Check(cloud.frame == FrameId::kSensor, "a raw cloud is sensor-frame");
  Check(cloud.motion_compensation == MotionCompensation::kNone,
        "a raw cloud carries no motion compensation");
  Check(RawTimedPoint{}.ray_index == -1 && RawTimedPoint{}.geom_id == -1,
        "an unpopulated point's provenance ids are -1, not 0");

  // Clear() must reset the motion-compensation tag too, or a reused buffer would claim to
  // be deskewed on its next fill.
  TimedPointCloud reused = cloud;
  reused.motion_compensation = MotionCompensation::kDeskewedToStamp;
  reused.Clear();
  Check(reused.empty() && reused.motion_compensation == MotionCompensation::kNone,
        "Clear() resets both the points and the motion-compensation tag");

  // Value semantics.
  TimedPointCloud copy = cloud;
  Check(copy.size() == cloud.size() && copy.stamp_s == cloud.stamp_s &&
            copy.sequence == cloud.sequence,
        "a copied cloud round-trips its stamp, sequence and size");
  copy.points[0].range_m = 99.0;
  Check(cloud.points[0].range_m == 1.0, "copying a cloud deep-copies its points");
}

void TestScanStats() {
  Section("ScanStats");
  ScanStats stats;
  stats.rays_cast = 1000;
  stats.raw_hits = 900;
  stats.no_hit = 100;
  stats.self_rejected = 40;
  stats.range_rejected = 10;
  stats.accepted = 850;
  Check(stats.IsBalanced(), "a census whose outcomes sum to rays_cast is balanced");

  // THE definition guard. self_rejected / rays_cast, not self_rejected / raw_hits. The two
  // differ by enough to change a conclusion (0.040 vs 0.044 here), which is exactly the
  // ambiguity that was resolved once and must not drift back.
  Check(std::abs(stats.SelfHitFraction() - 0.040) < 1e-12,
        "SelfHitFraction denominates by rays_cast");
  Check(std::abs(stats.SelfHitFraction() -
                 static_cast<double>(stats.self_rejected) / stats.raw_hits) > 1e-6,
        "the rays_cast and raw_hits denominators are measurably different");

  ScanStats unbalanced = stats;
  unbalanced.accepted += 1;
  Check(!unbalanced.IsBalanced(), "a census that does not close is rejected");

  Check(ScanStats{}.IsBalanced(), "an all-zero census trivially balances");
  Check(ScanStats{}.SelfHitFraction() == 0.0,
        "SelfHitFraction is 0, not NaN, when no ray was cast");
}

void TestFrameTransformSnapshot() {
  Section("FrameTransformSnapshot");
  FrameTransformSnapshot snapshot;
  CheckInvalid(snapshot, "valid is false", "a default-constructed snapshot is invalid");

  snapshot.stamp_s = 2.0;
  snapshot.world_from_base = Eigen::Isometry3d::Identity();
  snapshot.world_from_base.translation() = Eigen::Vector3d(1.0, 2.0, 0.837);
  snapshot.world_from_base.linear() =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  snapshot.base_from_sensor = Eigen::Isometry3d::Identity();
  snapshot.base_from_sensor.translation() = Eigen::Vector3d(0.0002835, 0.00003, 0.428434);
  snapshot.base_from_sensor.linear() =
      (Eigen::AngleAxisd(0.05112069379091391, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  snapshot.world_from_sensor = snapshot.world_from_base * snapshot.base_from_sensor;
  snapshot.gravity_in_base =
      snapshot.world_from_base.linear().transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
  snapshot.valid = true;
  CheckValid(snapshot, "a composed snapshot validates");

  // THE check worth having: the three transforms are read independently from MuJoCo, so a
  // composition mismatch means the body pose and the site pose came from different states.
  FrameTransformSnapshot inconsistent = snapshot;
  inconsistent.world_from_sensor.translation().x() += 0.01;
  CheckInvalid(inconsistent, "world_from_base * base_from_sensor",
               "an inconsistent transform composition is rejected");

  FrameTransformSnapshot unnormalised = snapshot;
  unnormalised.gravity_in_base *= 2.0;
  CheckInvalid(unnormalised, "unit vector", "a non-unit gravity direction is rejected");

  FrameTransformSnapshot skewed = snapshot;
  skewed.world_from_base.linear() *= 1.5;
  CheckInvalid(skewed, "non-orthonormal", "a scaled rotation is rejected");

  FrameTransformSnapshot nan_stamp = snapshot;
  nan_stamp.stamp_s = kNan;
  CheckInvalid(nan_stamp, "finite", "a NaN stamp is rejected");

  // Value semantics.
  FrameTransformSnapshot copy = snapshot;
  copy.world_from_base.translation().x() = 99.0;
  Check(snapshot.world_from_base.translation().x() == 1.0,
        "copying a snapshot deep-copies its transforms");
}

// ---------------------------------------------------------------------------------------
// Contracts added in the contract-freeze phase.
// ---------------------------------------------------------------------------------------

DeskewedPointCloud MakeDeskewed() {
  DeskewedPointCloud cloud;
  cloud.stamp_s = 3.0;
  cloud.sequence = 4;
  cloud.window_begin_s = -0.1;
  cloud.window_end_s = 0.0;
  cloud.max_correction_m = 0.004;
  cloud.points.push_back(MakePoint(1.0f, 0.0f, -0.8f, 1.28, 0));
  cloud.points.push_back(MakePoint(2.0f, 1.0f, -0.5f, 2.29, 1));
  return cloud;
}

void TestDeskewedPointCloud() {
  Section("DeskewedPointCloud");
  const DeskewedPointCloud cloud = MakeDeskewed();
  CheckValid(cloud, "a deskewed cloud validates");
  Check(cloud.frame == FrameId::kBase, "a deskewed cloud is base-frame");
  Check(cloud.motion_compensation == MotionCompensation::kDeskewedToStamp,
        "a deskewed cloud is tagged as deskewed");

  DeskewedPointCloud wrong_frame = cloud;
  wrong_frame.frame = FrameId::kSensor;
  CheckInvalid(wrong_frame, "must be kBase", "a sensor-frame deskewed cloud is rejected");

  DeskewedPointCloud untagged = cloud;
  untagged.motion_compensation = MotionCompensation::kNone;
  CheckInvalid(untagged, "kDeskewedToStamp",
               "a deskewed cloud claiming no compensation is rejected");

  DeskewedPointCloud reversed = cloud;
  reversed.window_begin_s = 0.5;
  CheckInvalid(reversed, "window_begin_s", "an inverted scan window is rejected");

  DeskewedPointCloud poisoned = cloud;
  poisoned.points[1].position.x() = std::numeric_limits<float>::quiet_NaN();
  CheckInvalid(poisoned, "non-finite position", "a NaN point position is rejected");

  // Reserve()/Clear() are the preallocation contract: capacity survives a clear, so a
  // steady-state frame never reallocates.
  DeskewedPointCloud pooled;
  pooled.Reserve(4096);
  const auto capacity_after_reserve = pooled.points.capacity();
  pooled.points.push_back(MakePoint(0.0f, 0.0f, 0.0f, 0.0, 0));
  pooled.Clear();
  Check(capacity_after_reserve >= 4096 && pooled.points.capacity() == capacity_after_reserve,
        "Clear() keeps the reserved capacity");

  DeskewedPointCloud copy = cloud;
  copy.points[0].position.x() = 42.0f;
  Check(cloud.points[0].position.x() == 1.0f, "copying deep-copies the points");
}

GravityAlignedCloud MakeGravityAligned() {
  GravityAlignedCloud cloud;
  cloud.stamp_s = 3.0;
  cloud.sequence = 4;
  cloud.alignment_from_base =
      Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()).toRotationMatrix();
  cloud.removed_roll_rad = 0.0;
  cloud.removed_pitch_rad = -0.05;
  cloud.residual_tilt_rad = 1e-5;
  cloud.points.push_back(MakePoint(1.0f, 0.0f, -0.8f, 1.28, 0));
  return cloud;
}

void TestGravityAlignedCloud() {
  Section("GravityAlignedCloud");
  const GravityAlignedCloud cloud = MakeGravityAligned();
  CheckValid(cloud, "a gravity-aligned cloud validates");
  Check(cloud.frame == FrameId::kGravityAlignedBase, "the frame tag is kGravityAlignedBase");

  // A silently non-orthonormal alignment scales every range without setting any flag,
  // which is why it is a validation failure rather than a diagnostic.
  GravityAlignedCloud scaled = cloud;
  scaled.alignment_from_base *= 1.01;
  CheckInvalid(scaled, "orthonormal", "a scaled alignment matrix is rejected");

  GravityAlignedCloud reflected = cloud;
  reflected.alignment_from_base = -Eigen::Matrix3d::Identity();
  CheckInvalid(reflected, "reflection", "a reflection is rejected as an alignment");

  GravityAlignedCloud negative_tilt = cloud;
  negative_tilt.residual_tilt_rad = -0.1;
  CheckInvalid(negative_tilt, "residual_tilt_rad", "a negative residual tilt is rejected");

  GravityAlignedCloud wrong_frame = cloud;
  wrong_frame.frame = FrameId::kBase;
  CheckInvalid(wrong_frame, "kGravityAlignedBase", "a base-frame aligned cloud is rejected");

  GravityAlignedCloud copy = cloud;
  copy.alignment_from_base = Eigen::Matrix3d::Identity();
  Check(cloud.alignment_from_base(0, 0) != 1.0 || cloud.alignment_from_base(0, 2) != 0.0,
        "copying does not alias the alignment matrix");
}

LabeledPointCloud MakeLabeled() {
  LabeledPointCloud cloud;
  cloud.stamp_s = 3.0;
  cloud.sequence = 4;
  cloud.points.push_back(MakePoint(1.0f, 0.0f, -0.80f, 1.28, 0));  // ground
  cloud.points.push_back(MakePoint(2.0f, 0.0f, -0.20f, 2.01, 1));  // non-ground
  cloud.points.push_back(MakePoint(0.1f, 0.0f, 0.00f, 0.10, 2));   // self
  cloud.labels = {PointLabel::kGround, PointLabel::kNonGround, PointLabel::kSelf};
  cloud.segmentation.total_points = 3;
  cloud.segmentation.ground_points = 1;
  cloud.segmentation.non_ground_points = 1;
  cloud.segmentation.self_points = 1;
  cloud.segmentation.invalid_points = 0;
  cloud.segmentation.plane.valid = true;
  cloud.segmentation.plane.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  cloud.segmentation.plane.offset_m = 0.837;
  cloud.segmentation.plane.inlier_fraction = 0.97;
  cloud.segmentation.plane.fit_input_points = 100;
  return cloud;
}

void TestLabeledPointCloud() {
  Section("LabeledPointCloud / GroundSegmentationResult / GroundPlane");
  const LabeledPointCloud cloud = MakeLabeled();
  CheckValid(cloud, "a labelled cloud validates");
  Check(cloud.segmentation.IsBalanced(), "the label census closes");

  // Fraction denominators. Both are over TOTAL points, including self and invalid: a spike
  // in self hits must show as a falling ground fraction, not hide behind a renormalisation.
  Check(std::abs(cloud.segmentation.GroundFraction() - 1.0 / 3.0) < 1e-12,
        "GroundFraction denominates by total_points");
  Check(std::abs(cloud.segmentation.NonGroundFraction() - 1.0 / 3.0) < 1e-12,
        "NonGroundFraction denominates by total_points");
  Check(std::abs(cloud.segmentation.GroundFraction() +
                 cloud.segmentation.NonGroundFraction() - 1.0) > 1e-9,
        "the two fractions do not sum to 1 because self/invalid are in the denominator");

  // A stale census is the failure that would silently corrupt every downstream rate, so
  // the labels are recounted rather than trusted.
  LabeledPointCloud stale = cloud;
  stale.labels[0] = PointLabel::kNonGround;
  CheckInvalid(stale, "does not match the actual labels",
               "a census that disagrees with the labels is rejected");

  LabeledPointCloud unparallel = cloud;
  unparallel.labels.pop_back();
  CheckInvalid(unparallel, "parallel", "labels shorter than points is rejected");

  LabeledPointCloud miscounted = cloud;
  miscounted.segmentation.total_points = 4;
  CheckInvalid(miscounted, "total_points", "a total that disagrees with the size is rejected");

  LabeledPointCloud tilted = cloud;
  tilted.segmentation.plane.normal = Eigen::Vector3d(0.0, 0.0, -1.0);
  CheckInvalid(tilted, "point up", "a downward ground normal is rejected");

  LabeledPointCloud unnormalised = cloud;
  unnormalised.segmentation.plane.normal = Eigen::Vector3d(0.0, 0.0, 2.0);
  CheckInvalid(unnormalised, "unit vector", "a non-unit ground normal is rejected");

  LabeledPointCloud out_of_range = cloud;
  out_of_range.segmentation.plane.inlier_fraction = 1.5;
  CheckInvalid(out_of_range, "[0, 1]", "an inlier fraction above 1 is rejected");

  // A z-band-only segmentation has no plane, and that is a legitimate valid result.
  LabeledPointCloud no_plane = cloud;
  no_plane.segmentation.plane = GroundPlane{};
  CheckValid(no_plane, "a z-band result with no fitted plane is still valid");

  LabeledPointCloud copy = cloud;
  copy.labels[1] = PointLabel::kInvalid;
  Check(cloud.labels[1] == PointLabel::kNonGround, "copying deep-copies the labels");

  Check(std::string(ToString(PointLabel::kNonGround)) == "non_ground",
        "PointLabel names itself");
  Check(static_cast<int>(PointLabel::kInvalid) == 0, "kInvalid is the zero label");
}

ProjectedScan MakeScan() {
  ProjectedScan scan;
  scan.range_min_m = 0.1;
  scan.range_max_m = 10.0;
  scan.height_band_min_m = -0.7;
  scan.height_band_max_m = 0.2;
  scan.stamp_s = 3.0;
  scan.sequence = 4;
  scan.scan_time_s = 0.1;
  scan.Resize(360, true);
  scan.ranges[0] = 2.5f;
  scan.ranges[90] = 4.0f;
  scan.bin_point_counts[0] = 3;
  scan.bin_point_counts[90] = 1;
  scan.stats.input_points = 10;
  scan.stats.rejected_height_band = 4;
  scan.stats.rejected_range = 2;
  scan.stats.binned_points = 4;
  scan.stats.occupied_bins = 2;
  return scan;
}

void TestProjectedScan() {
  Section("ProjectedScan / ProjectionStats");
  const ProjectedScan scan = MakeScan();
  CheckValid(scan, "a projected scan validates");
  Check(scan.bins() == 360, "bins() is the range-array length");
  Check(std::abs(scan.angle_increment_rad - 2.0 * kPi / 360.0) < 1e-15,
        "Resize() derives the increment from the bin count");
  Check(std::abs(scan.AngleAt(0) + kPi) < 1e-15, "bin 0 is addressed by its lower edge, -pi");
  Check(scan.HasReturn(0) && !scan.HasReturn(1), "an unfilled bin has no return");
  Check(std::isnan(scan.ranges[1]), "a missing bin is NaN, not +inf and not 0");
  Check(scan.time_increment_s == 0.0, "a post-deskew scan is synchronic");
  Check(scan.stats.IsBalanced(), "the projection census closes");

  // Denominators. Occupancy is over ALL bins; the nearest-return loss is over BINNED
  // points, not input points - a point rejected by the height band was never a candidate.
  Check(std::abs(scan.stats.OccupancyFraction() - 2.0 / 360.0) < 1e-12,
        "OccupancyFraction denominates by total_bins");
  Check(std::abs(scan.stats.NearestReturnLossFraction() - 0.5) < 1e-12,
        "NearestReturnLossFraction denominates by binned_points");

  ProjectedScan too_few = scan;
  too_few.Resize(4, false);
  CheckInvalid(too_few, "at least 8 bins", "a scan with fewer than 8 bins is rejected");

  ProjectedScan bad_increment = scan;
  bad_increment.angle_increment_rad *= 2.0;
  CheckInvalid(bad_increment, "angle_increment_rad",
               "an increment inconsistent with the bin count is rejected");

  ProjectedScan out_of_band = scan;
  out_of_band.ranges[5] = 50.0f;
  CheckInvalid(out_of_band, "outside [range_min_m, range_max_m]",
               "a range beyond the declared maximum is rejected");

  ProjectedScan infinite = scan;
  infinite.ranges[5] = std::numeric_limits<float>::infinity();
  CheckInvalid(infinite, "non-NaN, non-finite",
               "+inf in a bin is rejected (NaN is the internal empty policy)");

  ProjectedScan inverted = scan;
  inverted.range_min_m = 20.0;
  CheckInvalid(inverted, "range_min_m", "range_min >= range_max is rejected");

  ProjectedScan timed = scan;
  timed.time_increment_s = 1e-6;
  CheckInvalid(timed, "time_increment_s", "a non-zero time increment is rejected");

  ProjectedScan mismatched_histogram = scan;
  mismatched_histogram.bin_point_counts.pop_back();
  CheckInvalid(mismatched_histogram, "parallel",
               "a histogram of the wrong length is rejected");

  // The histogram is the one optional buffer: omitting it entirely is valid.
  ProjectedScan no_histogram = scan;
  no_histogram.bin_point_counts.clear();
  CheckValid(no_histogram, "a scan without the optional histogram is valid");

  ProjectedScan copy = scan;
  copy.ranges[0] = 9.0f;
  Check(scan.ranges[0] == 2.5f, "copying deep-copies the range array");
}

void TestCluster2D() {
  Section("Cluster2D");
  Cluster2D cluster;
  cluster.first_index = 10;
  cluster.last_index = 14;
  cluster.point_count = 5;
  cluster.first_bin = 100;
  cluster.last_bin = 104;
  cluster.front_visible = true;
  cluster.back_visible = true;
  CheckValid(cluster, "a cluster validates");
  Check(cluster.is_visible(), "both endpoints visible means fully visible");

  Cluster2D occluded = cluster;
  occluded.back_visible = false;
  Check(!occluded.is_visible(), "one occluded endpoint means not fully visible");
  CheckValid(occluded, "partial visibility is a valid state, not an error");

  Cluster2D miscounted = cluster;
  miscounted.point_count = 4;
  CheckInvalid(miscounted, "index span", "a count that disagrees with the span is rejected");

  Cluster2D inverted = cluster;
  inverted.last_index = 9;
  CheckInvalid(inverted, "last_index", "an inverted span is rejected");

  CheckInvalid(Cluster2D{}, "indices must be >= 0",
               "a default-constructed cluster is invalid");
}

FittedPrimitive2D MakeSegment() {
  FittedPrimitive2D primitive;
  primitive.kind = PrimitiveKind::kSegment;
  primitive.first_point = Eigen::Vector2d(1.0, 0.0);
  primitive.last_point = Eigen::Vector2d(1.0, 0.5);
  primitive.normal = Eigen::Vector2d(1.0, 0.0);  // Perpendicular to a +y segment.
  primitive.fit_residual_m = 0.002;
  primitive.point_count = 7;
  primitive.cluster_index = 0;
  primitive.front_visible = true;
  primitive.back_visible = true;
  return primitive;
}

void TestFittedPrimitive2D() {
  Section("FittedPrimitive2D");
  const FittedPrimitive2D segment = MakeSegment();
  CheckValid(segment, "a fitted segment validates");
  Check(std::abs(segment.Length() - 0.5) < 1e-12, "Length() is the endpoint distance");

  FittedPrimitive2D circle;
  circle.kind = PrimitiveKind::kCircle;
  circle.center = Eigen::Vector2d(2.0, 1.0);
  circle.radius_m = 0.25;
  circle.fit_residual_m = 0.004;
  circle.point_count = 9;
  circle.cluster_index = 1;
  CheckValid(circle, "a fitted circle validates without segment fields");

  // An inherited normal from a previous fit would send every downstream offset the wrong
  // way with no other symptom, so perpendicularity is checked, not assumed.
  FittedPrimitive2D stale_normal = segment;
  stale_normal.normal = Eigen::Vector2d(0.0, 1.0);
  CheckInvalid(stale_normal, "perpendicular", "a non-perpendicular segment normal is rejected");

  FittedPrimitive2D unnormalised = segment;
  unnormalised.normal = Eigen::Vector2d(2.0, 0.0);
  CheckInvalid(unnormalised, "unit vector", "a non-unit segment normal is rejected");

  FittedPrimitive2D degenerate = segment;
  degenerate.last_point = degenerate.first_point;
  CheckInvalid(degenerate, "positive length", "a zero-length segment is rejected");

  FittedPrimitive2D untagged = segment;
  untagged.kind = PrimitiveKind::kNone;
  CheckInvalid(untagged, "kind must be set", "an untagged primitive is rejected");

  FittedPrimitive2D too_few = segment;
  too_few.point_count = 1;
  CheckInvalid(too_few, "point_count", "a one-point fit is rejected");

  FittedPrimitive2D negative_residual = circle;
  negative_residual.fit_residual_m = -1e-9;
  CheckInvalid(negative_residual, "fit_residual_m",
               "a negative RMS residual is rejected (it is an RMS, not a signed error)");

  Check(std::string(ToString(PrimitiveKind::kCircle)) == "circle", "PrimitiveKind names itself");
}

CircleObservation MakeObservation() {
  CircleObservation observation;
  observation.center = Eigen::Vector2d(2.0, 1.0);
  observation.radius_fitted_m = 0.25;
  observation.radius_enclosing_m = 0.27;
  observation.arc_extent_rad = 0.22;
  observation.arc_length_m = 0.39;
  observation.fit_residual_m = 0.004;
  observation.range_to_center_m = 2.236;
  observation.sigma_center_m = 0.02;
  observation.sigma_radius_m = 0.015;
  observation.point_count = 9;
  observation.primitive_index = 0;
  observation.front_visible = true;
  observation.back_visible = true;
  observation.stamp_s = 3.0;
  return observation;
}

void TestCircleObservation() {
  Section("CircleObservation");
  const CircleObservation observation = MakeObservation();
  CheckValid(observation, "a circle observation validates");
  Check(observation.is_fully_visible(), "both endpoints visible means fully visible");

  // The arc extent is SENSOR-referenced and the arc length is CENTRE-referenced; at 2.24 m
  // range on a 0.25 m circle they differ by roughly range/radius, which is why the two are
  // separate fields with separate definitions rather than one convertible number.
  Check(observation.arc_length_m > observation.arc_extent_rad * observation.radius_fitted_m,
        "arc_length_m is centre-referenced, not arc_extent_rad times the radius");

  // The pipeline transforms observations to world before tracking; both frames are legal
  // and nothing else is.
  CircleObservation in_world = observation;
  in_world.frame = FrameId::kWorld;
  CheckValid(in_world, "a world-frame observation validates after the transform");

  CircleObservation in_sensor = observation;
  in_sensor.frame = FrameId::kSensor;
  CheckInvalid(in_sensor, "kGravityAlignedBase or kWorld",
               "a sensor-frame observation is rejected");

  CircleObservation zero_radius = observation;
  zero_radius.radius_fitted_m = 0.0;
  CheckInvalid(zero_radius, "radius_fitted_m", "a zero fitted radius is rejected");

  CircleObservation over_full_turn = observation;
  over_full_turn.arc_extent_rad = 7.0;
  CheckInvalid(over_full_turn, "[0, 2*pi]", "an arc wider than a full turn is rejected");

  CircleObservation negative_sigma = observation;
  negative_sigma.sigma_center_m = -0.01;
  CheckInvalid(negative_sigma, "sigmas", "a negative measurement sigma is rejected");

  CircleObservation copy = observation;
  copy.center.x() = 9.0;
  Check(observation.center.x() == 2.0, "copying does not alias the centre");
}

TrackState2D MakeTrack() {
  TrackState2D track;
  track.id = 17;
  track.center = Eigen::Vector2d(2.0, 1.0);
  track.velocity = Eigen::Vector2d(0.3, -0.1);
  track.radius_m = 0.25;
  track.radius_rate_mps = 0.0;
  track.center_variance = Eigen::Vector2d(0.0004, 0.0004);
  track.velocity_variance = Eigen::Vector2d(0.0025, 0.0025);
  track.radius_variance = 0.0002;
  track.radius_rate_variance = 0.0001;
  track.created_stamp_s = 2.0;
  track.last_update_stamp_s = 3.0;
  track.last_measurement_stamp_s = 3.0;
  track.hits = 9;
  track.misses = 1;
  track.consecutive_misses = 0;
  track.status = TrackStatus::kConfirmed;
  return track;
}

void TestTrackState2D() {
  Section("TrackState2D");
  const TrackState2D track = MakeTrack();
  CheckValid(track, "a confirmed track validates");

  // Age is a DURATION since creation; staleness is time since the last MEASUREMENT. They
  // are deliberately different numbers and the test pins both.
  Check(std::abs(track.age_s() - 1.0) < 1e-12, "age_s() is last_update minus created");
  Check(std::abs(track.staleness_s(3.4) - 0.4) < 1e-12,
        "staleness_s() is now minus the last measurement, not the age");
  Check(track.scans_seen() == 10, "scans_seen() is hits plus misses");
  Check(std::abs(track.HitRatio() - 0.9) < 1e-12, "HitRatio denominates by hits plus misses");

  TrackState2D coasting = track;
  coasting.status = TrackStatus::kCoasting;
  coasting.misses = 3;
  coasting.consecutive_misses = 2;  // Must stay <= misses; a run is part of the total.
  coasting.last_measurement_stamp_s = 2.8;
  CheckValid(coasting, "a coasting track validates");

  TrackState2D impossible_run = coasting;
  impossible_run.consecutive_misses = 9;
  CheckInvalid(impossible_run, "consecutive_misses must be <= misses",
               "a miss run longer than the total miss count is rejected");

  // The lifecycle invariants: coasting means currently unmatched, confirmed means matched.
  // Getting these backwards would publish a coasting track as a fresh measurement.
  TrackState2D coasting_without_miss = track;
  coasting_without_miss.status = TrackStatus::kCoasting;
  CheckInvalid(coasting_without_miss, "consecutive_misses > 0",
               "a coasting track with no consecutive miss is rejected");

  TrackState2D confirmed_while_missing = track;
  confirmed_while_missing.consecutive_misses = 1;
  CheckInvalid(confirmed_while_missing, "consecutive_misses == 0",
               "a confirmed track that is currently missing is rejected");

  TrackState2D negative_variance = track;
  negative_variance.center_variance.x() = -1e-9;
  CheckInvalid(negative_variance, "variances",
               "a negative variance is rejected (it is a broken filter, not big uncertainty)");

  TrackState2D future_measurement = track;
  future_measurement.last_measurement_stamp_s = 4.0;
  CheckInvalid(future_measurement, "last_measurement_stamp_s",
               "a measurement newer than the last update is rejected");

  TrackState2D zero_id = track;
  zero_id.id = 0;
  CheckInvalid(zero_id, "id must be non-zero", "id 0 is reserved and rejected");

  TrackState2D untagged = track;
  untagged.status = TrackStatus::kInvalid;
  CheckInvalid(untagged, "status must be set", "an unset status is rejected");

  Check(std::string(ToString(TrackStatus::kTentative)) == "tentative",
        "TrackStatus names itself");
}

void TestTrackPrediction2D() {
  Section("TrackPrediction2D");
  TrackPrediction2D prediction;
  prediction.id = 17;
  prediction.source_stamp_s = 3.0;
  prediction.stamp_s = 3.1;
  prediction.center = Eigen::Vector2d(2.03, 0.99);
  prediction.velocity = Eigen::Vector2d(0.3, -0.1);
  prediction.radius_m = 0.25;
  prediction.center_variance = Eigen::Vector2d(0.0006, 0.0006);
  prediction.velocity_variance = Eigen::Vector2d(0.0026, 0.0026);
  prediction.radius_variance = 0.0002;
  CheckValid(prediction, "a forward prediction validates");
  Check(std::abs(prediction.dt_s() - 0.1) < 1e-12, "dt_s() is the extrapolation horizon");

  TrackPrediction2D backwards = prediction;
  backwards.stamp_s = 2.9;
  CheckInvalid(backwards, "backwards in time",
               "predicting into the past is rejected");

  TrackPrediction2D zero_horizon = prediction;
  zero_horizon.stamp_s = zero_horizon.source_stamp_s;
  CheckValid(zero_horizon, "a zero-horizon prediction is valid");

  CheckInvalid(TrackPrediction2D{}, "id must be non-zero",
               "a default-constructed prediction is invalid");
}

PerceptionObstacle MakePerceptionObstacle() {
  PerceptionObstacle obstacle;
  obstacle.id = 17;
  obstacle.center = Eigen::Vector2d(2.0, 1.0);
  obstacle.velocity = Eigen::Vector2d(0.3, -0.1);
  obstacle.radius_true_m = 0.25;
  obstacle.center_variance = Eigen::Vector2d(0.0004, 0.0004);
  obstacle.velocity_variance = Eigen::Vector2d(0.0025, 0.0025);
  obstacle.radius_variance = 0.0002;
  obstacle.last_update_stamp_s = 3.0;
  obstacle.track_age_s = 1.0;
  obstacle.confidence = 0.9;
  return obstacle;
}

void TestPerceptionObstacle() {
  Section("PerceptionObstacle");
  const PerceptionObstacle obstacle = MakePerceptionObstacle();
  CheckValid(obstacle, "a perception obstacle validates");

  // Confidence is the track's lifetime hit ratio, a bounded [0, 1] number - not an
  // unbounded score and not a probability of existence.
  const TrackState2D track = MakeTrack();
  Check(std::abs(obstacle.confidence - track.HitRatio()) < 1e-12,
        "confidence is the originating track's HitRatio");

  PerceptionObstacle over_confident = obstacle;
  over_confident.confidence = 1.2;
  CheckInvalid(over_confident, "[0, 1]", "a confidence above 1 is rejected");

  PerceptionObstacle negative_age = obstacle;
  negative_age.track_age_s = -0.1;
  CheckInvalid(negative_age, "track_age_s", "a negative track age is rejected");

  PerceptionObstacle zero_radius = obstacle;
  zero_radius.radius_true_m = 0.0;
  CheckInvalid(zero_radius, "radius_true_m", "a zero radius is rejected");

  PerceptionObstacle wrong_frame = obstacle;
  wrong_frame.frame = FrameId::kGravityAlignedBase;
  CheckInvalid(wrong_frame, "kWorld", "a non-world obstacle is rejected");
}

SafetyObstacle MakeSafetyObstacle() {
  SafetyObstacle obstacle;
  obstacle.id = 17;
  obstacle.center = Eigen::Vector2d(2.0, 1.0);
  obstacle.velocity = Eigen::Vector2d(0.3, -0.1);
  obstacle.radius_true_m = 0.25;
  obstacle.radius_inflated_m = 0.35;
  obstacle.position_inflation_m = 0.04;
  obstacle.stamp_s = 3.0;
  obstacle.age_s = 0.05;
  obstacle.valid = true;
  obstacle.source = ObstacleSource::kEstimated;
  return obstacle;
}

void TestSafetyObstacle() {
  Section("SafetyObstacle");
  const SafetyObstacle obstacle = MakeSafetyObstacle();
  CheckValid(obstacle, "a safety obstacle validates");

  // THE invariant. Under-estimating a radius is the one unrecoverable error in the whole
  // subsystem, so a safety obstacle that shrinks its own estimate cannot be constructed
  // past validation.
  SafetyObstacle shrunk = obstacle;
  shrunk.radius_inflated_m = 0.20;
  CheckInvalid(shrunk, "never shrink",
               "an inflated radius smaller than the true radius is rejected");

  SafetyObstacle equal_radius = obstacle;
  equal_radius.radius_inflated_m = equal_radius.radius_true_m;
  CheckValid(equal_radius, "zero inflation is allowed; only shrinking is not");

  SafetyObstacle unsourced = obstacle;
  unsourced.source = ObstacleSource::kUnknown;
  CheckInvalid(unsourced, "source must be set",
               "an obstacle that will not say where it came from is rejected");

  SafetyObstacle oracle = obstacle;
  oracle.source = ObstacleSource::kOracle;
  CheckValid(oracle, "an oracle-sourced safety obstacle validates");

  SafetyObstacle negative_age = obstacle;
  negative_age.age_s = -0.01;
  CheckInvalid(negative_age, "age_s", "a negative age is rejected");

  Check(std::string(ToString(ObstacleSource::kEstimated)) == "estimated",
        "ObstacleSource names itself");
}

void TestOracleObstacleState() {
  Section("OracleObstacleState");
  OracleObstacleState oracle;
  oracle.id = 3;
  oracle.center = Eigen::Vector2d(2.0, 1.0);
  oracle.velocity = Eigen::Vector2d(0.3, -0.1);
  oracle.radius_m = 0.25;
  oracle.stamp_s = 3.0;
  CheckValid(oracle, "an oracle state validates");

  // Oracle ids come from DynamicObstacleManager and start at 0, unlike track ids where 0
  // is reserved. The two id spaces are deliberately different and this pins that.
  OracleObstacleState first = oracle;
  first.id = 0;
  CheckValid(first, "oracle id 0 is legal (unlike track id 0)");

  CheckInvalid(OracleObstacleState{}, "id must be >= 0",
               "a default-constructed oracle state is invalid");

  OracleObstacleState negative = oracle;
  negative.radius_m = -0.25;
  CheckInvalid(negative, "radius_m", "a negative oracle radius is rejected");
}

void TestStageTimingAndDiagnostics() {
  Section("StageTiming / MatchedPairError / EstimationDiagnostics");
  StageTiming timing;
  timing[PipelineStage::kDeskew] = 120.0;
  timing[PipelineStage::kProjection] = 80.0;
  CheckValid(timing, "a stage timing validates");
  Check(std::abs(timing.SumUs() - 200.0) < 1e-12, "SumUs() adds the per-stage times");
  Check(kPipelineStageCount == 9, "the stage table has nine entries");
  Check(std::string(ToString(PipelineStage::kTracking)) == "tracking",
        "PipelineStage names itself");

  StageTiming negative = timing;
  negative[PipelineStage::kSafety] = -1.0;
  CheckInvalid(negative, "finite and >= 0", "a negative stage time is rejected");

  timing.Clear();
  Check(timing.SumUs() == 0.0, "Clear() zeroes every stage");

  EstimationDiagnostics diagnostics;
  diagnostics.sequence = 4;
  diagnostics.stamp_s = 3.0;
  diagnostics.scan.rays_cast = 100;
  diagnostics.scan.raw_hits = 90;
  diagnostics.scan.no_hit = 10;
  diagnostics.scan.self_rejected = 5;
  diagnostics.scan.range_rejected = 5;
  diagnostics.scan.accepted = 80;
  diagnostics.segmentation.total_points = 80;
  diagnostics.segmentation.ground_points = 50;
  diagnostics.segmentation.non_ground_points = 30;
  diagnostics.projection.input_points = 30;
  diagnostics.projection.binned_points = 30;
  diagnostics.projection.occupied_bins = 20;
  diagnostics.projection.total_bins = 360;
  CheckValid(diagnostics, "a comparison-off diagnostics record validates");

  // With comparison off, the comparison fields must be EMPTY rather than zero-filled, so
  // an unrun comparison cannot be read as a perfect score.
  EstimationDiagnostics leaked = diagnostics;
  leaked.oracle_count = 3;
  CheckInvalid(leaked, "comparison fields must be empty",
               "comparison counts without comparison_valid are rejected");

  EstimationDiagnostics compared = diagnostics;
  compared.comparison_valid = true;
  compared.oracle_count = 3;
  compared.unmatched_oracle = 1;
  compared.unmatched_estimated = 1;
  MatchedPairError pair;
  pair.track_id = 17;
  pair.oracle_id = 2;
  pair.center_error_m = 0.03;
  pair.velocity_error_mps = 0.05;
  pair.radius_error_m = -0.01;  // Signed on purpose: negative means under-estimated.
  pair.stamp_s = 3.0;
  compared.matched.push_back(pair);
  compared.matched.push_back(pair);
  compared.matched.back().track_id = 18;
  compared.matched.back().oracle_id = 1;
  CheckValid(compared, "a comparison record whose association census closes validates");

  // Recall denominates by oracle count, precision by estimated count. Same numerator,
  // different denominators - the pair a single "match rate" float would have conflated.
  Check(std::abs(compared.DetectionRecall() - 2.0 / 3.0) < 1e-12,
        "DetectionRecall denominates by oracle_count");
  Check(std::abs(compared.DetectionPrecision() - 2.0 / 3.0) < 1e-12,
        "DetectionPrecision denominates by the estimated count");
  Check(compared.DetectionRecall() != 1.0, "an unmatched oracle obstacle lowers recall");

  EstimationDiagnostics broken_census = compared;
  broken_census.unmatched_oracle = 5;
  CheckInvalid(broken_census, "must equal oracle_count",
               "an association census that does not close is rejected");

  CheckInvalid(MatchedPairError{}, "track_id", "a default-constructed pair error is invalid");

  MatchedPairError signed_error = pair;
  signed_error.center_error_m = -0.01;
  CheckInvalid(signed_error, "center_error_m",
               "a negative centre error is rejected (it is a distance)");

  MatchedPairError under = pair;
  under.radius_error_m = -0.5;
  CheckValid(under, "a negative radius error is legal: the sign carries the safety meaning");

  EstimationDiagnostics reset = compared;
  reset.Clear();
  Check(!reset.comparison_valid && reset.matched.empty() && reset.sequence == 0,
        "Clear() returns the record to its default state");
}

PerceptionFrame MakeFrame() {
  PerceptionFrame frame;
  frame.sequence = 4;
  frame.stamp_s = 3.0;
  frame.publish_stamp_s = 3.02;

  FrameTransformSnapshot transform;
  transform.stamp_s = 3.0;
  transform.world_from_base = Eigen::Isometry3d::Identity();
  transform.base_from_sensor = Eigen::Isometry3d::Identity();
  transform.world_from_sensor = Eigen::Isometry3d::Identity();
  transform.gravity_in_base = Eigen::Vector3d(0.0, 0.0, -1.0);
  transform.valid = true;
  frame.transform = transform;

  frame.obstacles.push_back(MakePerceptionObstacle());
  frame.safety_obstacles.push_back(MakeSafetyObstacle());
  frame.tracks.push_back(MakeTrack());
  frame.observations.push_back(MakeObservation());

  frame.diagnostics.sequence = 4;
  frame.diagnostics.stamp_s = 3.0;
  frame.diagnostics.pipeline_latency_s = 0.02;
  frame.valid = true;
  return frame;
}

void TestPerceptionFrame() {
  Section("PerceptionFrame");
  const PerceptionFrame frame = MakeFrame();
  CheckValid(frame, "a valid frame validates");
  Check(std::abs(frame.latency_s() - 0.02) < 1e-12, "latency_s() is publish minus scan stamp");

  // A frame must not be simultaneously valid and carrying a reason it is not.
  PerceptionFrame contradictory = frame;
  contradictory.invalid_reason = FrameInvalidReason::kStale;
  CheckInvalid(contradictory, "kNone", "a valid frame with an invalid reason is rejected");

  PerceptionFrame silent = frame;
  silent.valid = false;
  CheckInvalid(silent, "must say why", "an invalid frame with no reason is rejected");

  PerceptionFrame dropped = frame;
  dropped.valid = false;
  dropped.invalid_reason = FrameInvalidReason::kNoTransformSnapshot;
  CheckValid(dropped, "an invalid frame that states its reason validates");
  Check(std::string(ToString(FrameInvalidReason::kNoTransformSnapshot)) ==
            "no_transform_snapshot",
        "FrameInvalidReason names itself");

  // A valid frame's world coordinates were all produced through its transform, so an
  // unusable transform silently invalidates every obstacle in it.
  PerceptionFrame no_pose = frame;
  no_pose.transform.valid = false;
  CheckInvalid(no_pose, "valid is false", "a valid frame with an unusable transform is rejected");

  PerceptionFrame time_travel = frame;
  time_travel.publish_stamp_s = 2.9;
  CheckInvalid(time_travel, "publish_stamp_s", "publishing before the scan is rejected");

  // Provenance: a SafetyObstacle with no parent PerceptionObstacle is either a duplicate
  // or a fabrication, and the DPCBF adapter has no way to notice either.
  PerceptionFrame orphan = frame;
  orphan.safety_obstacles[0].id = 999;
  CheckInvalid(orphan, "no matching obstacle id",
               "a safety obstacle with no parent obstacle is rejected");

  // Element validation runs through the bundle.
  PerceptionFrame bad_element = frame;
  bad_element.safety_obstacles[0].radius_inflated_m = 0.01;
  CheckInvalid(bad_element, "never shrink",
               "an invalid element invalidates the whole frame");

  // Retained-stage flags gate the expensive checks: an unretained scan is empty, not wrong.
  PerceptionFrame unretained = frame;
  CheckValid(unretained, "an empty unretained scan does not fail validation");
  PerceptionFrame retained_but_empty = frame;
  retained_but_empty.retained.projected_scan = true;
  CheckInvalid(retained_but_empty, "at least 8 bins",
               "a scan claimed as retained is validated for real");

  // Preallocation.
  PerceptionFrame pooled;
  PerceptionFrameCapacities capacities;
  capacities.max_points = 11520;
  capacities.max_bins = 360;
  capacities.max_clusters = 256;
  capacities.max_primitives = 512;
  capacities.max_observations = 256;
  capacities.max_tracks = 64;
  capacities.max_obstacles = 32;
  capacities.max_matched_pairs = 32;
  pooled.Reserve(capacities);
  Check(pooled.raw_cloud.points.capacity() >= 11520 && pooled.tracks.capacity() >= 64 &&
            pooled.safety_obstacles.capacity() >= 32,
        "Reserve() preallocates every buffer from one capacity struct");

  // Value semantics: the double buffer copies frames, so a copy must not alias.
  PerceptionFrame copy = frame;
  copy.safety_obstacles[0].center.x() = 99.0;
  Check(frame.safety_obstacles[0].center.x() == 2.0, "copying a frame deep-copies its vectors");
}

}  // namespace

int main() {
  std::printf("perception contract tests\n");

  TestFrameId();
  TestRayPattern();
  TestTimedPointCloud();
  TestScanStats();
  TestFrameTransformSnapshot();

  TestDeskewedPointCloud();
  TestGravityAlignedCloud();
  TestLabeledPointCloud();
  TestProjectedScan();
  TestCluster2D();
  TestFittedPrimitive2D();
  TestCircleObservation();
  TestTrackState2D();
  TestTrackPrediction2D();
  TestPerceptionObstacle();
  TestSafetyObstacle();
  TestOracleObstacleState();
  TestStageTimingAndDiagnostics();
  TestPerceptionFrame();

  return perception_test::Report("perception_contracts_test");
}
