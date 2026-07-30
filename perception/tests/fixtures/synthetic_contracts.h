// Hand-authored fixtures for every dumpable contract.
//
// WHY SYNTHETIC. Sixteen of the twenty-four record types have no producing stage yet - the
// pipeline phases that will make CircleObservations, TrackState2Ds and SafetyObstacles are all
// later than this one. The dump format nonetheless has to be correct for them NOW, because it
// is the format their own regression fixtures will be written in. So each is built by hand
// here, at values chosen to be hostile to a sloppy codec rather than pretty:
//
//   * -0.0, which compares equal to 0.0 but has a different bit pattern, so an
//     epsilon-tolerant round-trip passes while a bit-exact one fails
//   * the smallest denormal (5e-324) and the largest finite double, which a decimal
//     round-trip at fewer than 17 digits loses
//   * 1/3 and 0.1, which have no exact decimal form
//   * float fields fed values that are NOT exactly representable in float, to catch a codec
//     that silently widens them to double and back
//
// Every fixture also satisfies its contract's own Validate(), so the round-trip test can
// assert validity before and after and catch a codec that produces a readable but
// contract-violating record. The deliberately non-finite cases (NaN in an empty ProjectedScan
// bin, infinities) are exercised separately by the codec-level pathological-float test, since
// most contracts legitimately reject them.
#ifndef PERCEPTION_TESTS_FIXTURES_SYNTHETIC_CONTRACTS_H_
#define PERCEPTION_TESTS_FIXTURES_SYNTHETIC_CONTRACTS_H_

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "perception/core/contracts/detection_2d.h"
#include "perception/core/contracts/diagnostics.h"
#include "perception/core/contracts/frame_transform_snapshot.h"
#include "perception/core/contracts/labeled_point_cloud.h"
#include "perception/core/contracts/obstacles.h"
#include "perception/core/contracts/perception_frame.h"
#include "perception/core/contracts/point_clouds.h"
#include "perception/core/contracts/projected_scan.h"
#include "perception/core/contracts/ray_pattern.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/core/contracts/tracking_2d.h"

namespace perception_fixtures {

using namespace perception::core;  // NOLINT - test-local convenience.

// The awkward-value palette. Named so a failing assertion says which hazard it caught.
inline constexpr double kNegativeZero = -0.0;
inline constexpr double kSmallestDenormal = 5e-324;
inline constexpr double kLargestFinite = 1.7976931348623157e308;
inline double OneThird() { return 1.0 / 3.0; }

// A float value that is not exactly representable in float, so a codec that round-trips it
// through double and truncates differently is caught.
inline float AwkwardFloat(int variant) {
  return static_cast<float>(0.1 + 0.017 * static_cast<double>(variant));
}

inline double Perturb(double base, int variant) {
  return base + OneThird() * static_cast<double>(variant) * 1e-7;
}

inline RawTimedPoint MakeRawTimedPoint(int variant) {
  RawTimedPoint point;
  point.position = Eigen::Vector3f(AwkwardFloat(variant), -AwkwardFloat(variant + 1),
                                   variant == 0 ? -0.0f : AwkwardFloat(variant + 2));
  point.range_m = Perturb(3.14159265358979, variant);
  point.time_offset_s = AwkwardFloat(variant + 3);
  point.ray_index = 1000 + variant;
  point.geom_id = 7 + variant;
  point.body_id = 3 + variant;
  return point;
}

inline RayPattern MakeRayPattern(int variant) {
  RayPattern pattern;
  const int azimuth = 4;
  const int elevation = 3;
  pattern.azimuth_rays = azimuth;
  pattern.elevation_rays = elevation;
  pattern.period_s = 0.1;
  pattern.frame = FrameId::kSensor;
  for (int index = 0; index < azimuth * elevation; ++index) {
    // Unit directions, built from an angle so the components are irrational-ish rather than
    // round numbers a lossy codec could accidentally preserve.
    const double angle = OneThird() * static_cast<double>(index + variant);
    const Eigen::Vector3d direction(std::cos(angle), std::sin(angle), 0.0);
    pattern.directions.push_back(direction.normalized());
    pattern.time_offsets_s.push_back(Perturb(0.001 * index, variant));
  }
  return pattern;
}

inline TimedPointCloud MakeTimedPointCloud(int variant) {
  TimedPointCloud cloud;
  cloud.stamp_s = Perturb(1.2345678901234567, variant);
  cloud.sequence = 100 + static_cast<uint64_t>(variant);
  cloud.frame = FrameId::kSensor;
  cloud.motion_compensation = MotionCompensation::kNone;
  for (int index = 0; index < 3 + variant; ++index) {
    cloud.points.push_back(MakeRawTimedPoint(index + variant));
  }
  return cloud;
}

inline ScanStats MakeScanStats(int variant) {
  ScanStats stats;
  stats.rays_cast = 11520;
  stats.accepted = 8000 + variant;
  stats.self_rejected = 478;
  stats.range_rejected = 12;
  // Balanced by construction: the residual goes to no_hit, so IsBalanced() holds for every
  // variant. A fixture whose census did not balance would be testing the wrong thing.
  stats.no_hit = stats.rays_cast - stats.accepted - stats.self_rejected - stats.range_rejected;
  stats.raw_hits = stats.rays_cast - stats.no_hit;
  stats.aperture_transmitted = 9000;
  stats.aperture_diagnostic_valid = (variant % 2) == 0;
  stats.scan_wall_time_us = Perturb(4321.5, variant);
  return stats;
}

inline FrameTransformSnapshot MakeFrameTransformSnapshot(int variant) {
  FrameTransformSnapshot snapshot;
  snapshot.stamp_s = Perturb(2.5, variant);

  // A genuine rotation, not an axis-aligned one: an orthonormality bug survives identity.
  const double angle = 0.37 + 0.11 * static_cast<double>(variant);
  const Eigen::AngleAxisd yaw(angle, Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd pitch(0.05112069379091391, Eigen::Vector3d::UnitY());

  snapshot.world_from_base = Eigen::Isometry3d::Identity();
  snapshot.world_from_base.linear() = (yaw * pitch).toRotationMatrix();
  snapshot.world_from_base.translation() = Eigen::Vector3d(0.1, kNegativeZero, 0.837);

  snapshot.base_from_sensor = Eigen::Isometry3d::Identity();
  snapshot.base_from_sensor.linear() =
      Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  snapshot.base_from_sensor.translation() =
      Eigen::Vector3d(0.0002835, 0.00003, 0.428434);

  // Composed, not independently invented: the contract asserts the composition holds.
  snapshot.world_from_sensor = snapshot.world_from_base * snapshot.base_from_sensor;

  snapshot.gravity_in_base =
      (snapshot.world_from_base.linear().transpose() * Eigen::Vector3d(0.0, 0.0, -1.0))
          .normalized();
  snapshot.valid = true;
  return snapshot;
}

inline DeskewedPointCloud MakeDeskewedPointCloud(int variant) {
  DeskewedPointCloud cloud;
  cloud.stamp_s = Perturb(3.5, variant);
  cloud.sequence = 200 + static_cast<uint64_t>(variant);
  cloud.frame = FrameId::kBase;
  cloud.motion_compensation = MotionCompensation::kDeskewedToStamp;
  cloud.window_begin_s = -0.1;
  cloud.window_end_s = kNegativeZero;
  cloud.max_correction_m = kSmallestDenormal;
  for (int index = 0; index < 2 + variant; ++index) {
    cloud.points.push_back(MakeRawTimedPoint(index));
  }
  return cloud;
}

inline GravityAlignedCloud MakeGravityAlignedCloud(int variant) {
  GravityAlignedCloud cloud;
  cloud.stamp_s = Perturb(4.5, variant);
  cloud.sequence = 300 + static_cast<uint64_t>(variant);
  cloud.frame = FrameId::kGravityAlignedBase;
  cloud.motion_compensation = MotionCompensation::kDeskewedToStamp;
  cloud.alignment_from_base =
      Eigen::AngleAxisd(0.02 + 0.01 * static_cast<double>(variant), Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  cloud.removed_roll_rad = -0.0123456789;
  cloud.removed_pitch_rad = 0.05112069379091391;
  cloud.residual_tilt_rad = kNegativeZero;
  for (int index = 0; index < 2 + variant; ++index) {
    cloud.points.push_back(MakeRawTimedPoint(index + 1));
  }
  return cloud;
}

inline GroundPlane MakeGroundPlane(int variant) {
  GroundPlane plane;
  plane.normal =
      Eigen::Vector3d(0.01 * static_cast<double>(variant), 0.02, 1.0).normalized();
  plane.offset_m = 0.837;
  plane.inlier_fraction = OneThird();
  plane.fit_input_points = 4096 + variant;
  plane.valid = true;
  return plane;
}

inline GroundSegmentationResult MakeGroundSegmentationResult(int variant) {
  GroundSegmentationResult result;
  result.ground_points = 5000;
  result.non_ground_points = 3000 + variant;
  result.self_points = 400;
  result.invalid_points = 100;
  result.total_points =
      result.ground_points + result.non_ground_points + result.self_points +
      result.invalid_points;
  result.plane = MakeGroundPlane(variant);
  result.wall_time_us = Perturb(1234.5, variant);
  return result;
}

inline LabeledPointCloud MakeLabeledPointCloud(int variant) {
  LabeledPointCloud cloud;
  cloud.stamp_s = Perturb(5.5, variant);
  cloud.sequence = 400 + static_cast<uint64_t>(variant);
  cloud.frame = FrameId::kGravityAlignedBase;
  cloud.motion_compensation = MotionCompensation::kDeskewedToStamp;

  // One point of each label, so the census the contract cross-checks is non-trivial.
  const PointLabel labels[] = {PointLabel::kGround, PointLabel::kNonGround, PointLabel::kSelf,
                              PointLabel::kInvalid};
  for (int index = 0; index < 4; ++index) {
    cloud.points.push_back(MakeRawTimedPoint(index + variant));
    cloud.labels.push_back(labels[index]);
  }
  cloud.segmentation = GroundSegmentationResult{};
  cloud.segmentation.total_points = 4;
  cloud.segmentation.ground_points = 1;
  cloud.segmentation.non_ground_points = 1;
  cloud.segmentation.self_points = 1;
  cloud.segmentation.invalid_points = 1;
  cloud.segmentation.plane = MakeGroundPlane(variant);
  cloud.segmentation.wall_time_us = Perturb(99.5, variant);
  return cloud;
}

inline ProjectionStats MakeProjectionStats(int variant) {
  ProjectionStats stats;
  stats.rejected_non_finite = 3;
  stats.rejected_height_band = 700;
  stats.rejected_range = 40;
  stats.rejected_angle = 0;
  stats.binned_points = 2000 + variant;
  stats.input_points = stats.rejected_non_finite + stats.rejected_height_band +
                       stats.rejected_range + stats.rejected_angle + stats.binned_points;
  stats.total_bins = 16;
  stats.occupied_bins = 9;
  stats.wall_time_us = Perturb(210.25, variant);
  return stats;
}

inline ProjectedScan MakeProjectedScan(int variant) {
  ProjectedScan scan;
  scan.angle_min_rad = -kPi;
  scan.angle_max_rad = kPi;
  scan.range_min_m = 0.10;
  scan.range_max_m = 10.0;
  scan.height_band_min_m = -0.70;
  scan.height_band_max_m = 0.20;
  scan.stamp_s = Perturb(6.5, variant);
  scan.sequence = 500 + static_cast<uint64_t>(variant);
  scan.frame = FrameId::kGravityAlignedBase;
  scan.motion_compensation = MotionCompensation::kDeskewedToStamp;
  scan.time_increment_s = 0.0;
  scan.scan_time_s = 0.1;
  scan.Resize(16, /*with_histogram=*/true);

  // Nine occupied bins, seven left as NaN. The NaNs are the point: an empty bin IS a NaN in
  // this contract, so a codec that cannot carry one destroys the never-measured/measured
  // distinction. Ranges stay inside the declared bounds, which Validate() enforces.
  for (int bin = 0; bin < 9; ++bin) {
    scan.ranges[static_cast<std::size_t>(bin)] =
        static_cast<float>(0.5 + 0.125 * static_cast<double>(bin + variant));
    scan.bin_point_counts[static_cast<std::size_t>(bin)] = bin + 1;
  }
  scan.stats = MakeProjectionStats(variant);
  scan.stats.total_bins = 16;
  scan.stats.occupied_bins = 9;
  return scan;
}

inline Cluster2D MakeCluster2D(int variant) {
  Cluster2D cluster;
  cluster.first_index = 10 + variant;
  cluster.last_index = 20 + variant;
  cluster.point_count = cluster.last_index - cluster.first_index + 1;
  cluster.first_bin = 30 + variant;
  cluster.last_bin = 41 + variant;
  cluster.front_visible = true;
  cluster.back_visible = (variant % 2) == 0;
  return cluster;
}

inline FittedPrimitive2D MakeFittedPrimitive2D(int variant) {
  FittedPrimitive2D primitive;
  primitive.point_count = 7 + variant;
  primitive.cluster_index = variant;
  primitive.fit_residual_m = kSmallestDenormal;
  primitive.front_visible = true;
  primitive.back_visible = false;

  if (variant % 2 == 0) {
    primitive.kind = PrimitiveKind::kSegment;
    primitive.first_point = Eigen::Vector2d(1.0, OneThird());
    primitive.last_point = Eigen::Vector2d(2.0, OneThird() + 0.5);
    const Eigen::Vector2d along = (primitive.last_point - primitive.first_point).normalized();
    // Exact left-hand normal of THIS segment; the contract rejects an inherited one.
    primitive.normal = Eigen::Vector2d(-along.y(), along.x());
  } else {
    primitive.kind = PrimitiveKind::kCircle;
    primitive.center = Eigen::Vector2d(OneThird(), kNegativeZero);
    primitive.radius_m = 0.25;
  }
  return primitive;
}

inline CircleObservation MakeCircleObservation(int variant) {
  CircleObservation observation;
  observation.center = Eigen::Vector2d(1.5 + OneThird(), kNegativeZero);
  observation.radius_fitted_m = 0.2309401076758503;  // sqrt(3)/3 * 0.4, the ported rule.
  observation.radius_enclosing_m = 0.26;
  observation.arc_extent_rad = 0.4363323129985824;
  observation.arc_length_m = 0.1;
  observation.fit_residual_m = 0.004;
  observation.range_to_center_m = 2.5;
  observation.sigma_center_m = kSmallestDenormal;
  observation.sigma_radius_m = kNegativeZero;
  observation.point_count = 9 + variant;
  observation.primitive_index = variant;
  observation.front_visible = true;
  observation.back_visible = true;
  observation.stamp_s = Perturb(7.5, variant);
  observation.frame = variant % 2 == 0 ? FrameId::kGravityAlignedBase : FrameId::kWorld;
  return observation;
}

inline TrackState2D MakeTrackState2D(int variant) {
  TrackState2D track;
  track.id = 1 + static_cast<uint32_t>(variant);
  track.center = Eigen::Vector2d(2.0, OneThird());
  track.velocity = Eigen::Vector2d(0.4, kNegativeZero);
  track.radius_m = 0.3;
  track.radius_rate_mps = -0.001;
  track.center_variance = Eigen::Vector2d(0.01, kSmallestDenormal);
  track.velocity_variance = Eigen::Vector2d(0.05, 0.05);
  track.radius_variance = 0.002;
  track.radius_rate_variance = kNegativeZero;
  track.created_stamp_s = 1.0;
  track.last_measurement_stamp_s = 2.0;
  track.last_update_stamp_s = 2.0;
  track.hits = 12;
  track.misses = 3;
  // kConfirmed requires consecutive_misses == 0; kCoasting requires it > 0. Alternating the
  // status across variants exercises both branches of that invariant.
  if (variant % 2 == 0) {
    track.status = TrackStatus::kConfirmed;
    track.consecutive_misses = 0;
  } else {
    track.status = TrackStatus::kCoasting;
    track.consecutive_misses = 2;
  }
  track.frame = FrameId::kWorld;
  return track;
}

inline TrackPrediction2D MakeTrackPrediction2D(int variant) {
  TrackPrediction2D prediction;
  prediction.id = 1 + static_cast<uint32_t>(variant);
  prediction.source_stamp_s = 2.0;
  prediction.stamp_s = 2.0 + 0.1 * static_cast<double>(variant);
  prediction.center = Eigen::Vector2d(2.1, OneThird());
  prediction.velocity = Eigen::Vector2d(0.4, kNegativeZero);
  prediction.radius_m = 0.3;
  prediction.center_variance = Eigen::Vector2d(0.02, 0.02);
  prediction.velocity_variance = Eigen::Vector2d(0.06, 0.06);
  prediction.radius_variance = kSmallestDenormal;
  prediction.frame = FrameId::kWorld;
  return prediction;
}

inline PerceptionObstacle MakePerceptionObstacle(int variant) {
  PerceptionObstacle obstacle;
  obstacle.id = 1 + static_cast<uint32_t>(variant);
  obstacle.center = Eigen::Vector2d(2.0, OneThird());
  obstacle.velocity = Eigen::Vector2d(0.4, kNegativeZero);
  obstacle.radius_true_m = 0.3;
  obstacle.center_variance = Eigen::Vector2d(0.01, 0.01);
  obstacle.velocity_variance = Eigen::Vector2d(0.05, 0.05);
  obstacle.radius_variance = 0.002;
  obstacle.last_update_stamp_s = Perturb(8.5, variant);
  obstacle.track_age_s = 1.5;
  obstacle.confidence = 0.8;
  obstacle.frame = FrameId::kWorld;
  return obstacle;
}

inline SafetyObstacle MakeSafetyObstacle(int variant) {
  SafetyObstacle obstacle;
  obstacle.id = 1 + static_cast<uint32_t>(variant);
  obstacle.center = Eigen::Vector2d(2.0, OneThird());
  obstacle.velocity = Eigen::Vector2d(0.4, kNegativeZero);
  obstacle.radius_true_m = 0.3;
  // Strictly >= radius_true_m: the one invariant of this contract that safety depends on.
  obstacle.radius_inflated_m = 0.42;
  obstacle.position_inflation_m = 0.02;
  obstacle.stamp_s = Perturb(9.5, variant);
  obstacle.age_s = kNegativeZero;
  obstacle.valid = true;
  obstacle.source = variant % 2 == 0 ? ObstacleSource::kEstimated : ObstacleSource::kOracle;
  obstacle.frame = FrameId::kWorld;
  return obstacle;
}

inline OracleObstacleState MakeOracleObstacleState(int variant) {
  OracleObstacleState state;
  state.id = variant;
  state.center = Eigen::Vector2d(-1.25, OneThird());
  state.velocity = Eigen::Vector2d(kNegativeZero, 0.8);
  state.radius_m = 0.2;
  state.stamp_s = Perturb(10.5, variant);
  state.frame = FrameId::kWorld;
  return state;
}

inline StageTiming MakeStageTiming(int variant) {
  StageTiming timing;
  for (std::size_t index = 0; index < kPipelineStageCount; ++index) {
    timing.wall_time_us[index] =
        Perturb(10.0 * static_cast<double>(index + 1), variant);
  }
  timing.wall_time_us[0] = kNegativeZero;
  timing.wall_time_us[1] = kSmallestDenormal;
  timing.wall_time_us[2] = kLargestFinite;
  return timing;
}

inline MatchedPairError MakeMatchedPairError(int variant) {
  MatchedPairError error;
  error.track_id = 1 + static_cast<uint32_t>(variant);
  error.oracle_id = variant;
  error.center_error_m = 0.031;
  error.velocity_error_mps = kNegativeZero;
  // Signed on purpose: a negative radius error means the estimate is SMALLER than truth,
  // which is the direction that matters for safety, so the sign has to survive the codec.
  error.radius_error_m = -0.015;
  error.stamp_s = Perturb(11.5, variant);
  return error;
}

inline EstimationDiagnostics MakeEstimationDiagnostics(int variant) {
  EstimationDiagnostics diagnostics;
  diagnostics.sequence = 600 + static_cast<uint64_t>(variant);
  diagnostics.stamp_s = Perturb(12.5, variant);
  diagnostics.scan = MakeScanStats(variant);
  diagnostics.segmentation = MakeGroundSegmentationResult(variant);
  diagnostics.projection = MakeProjectionStats(variant);
  diagnostics.clusters_found = 12;
  diagnostics.primitives_fitted = 15;
  diagnostics.circles_detected = 4;
  diagnostics.circles_rejected_radius = 1;
  diagnostics.circles_rejected_visibility = 2;
  diagnostics.tracks_tentative = 2;
  diagnostics.tracks_confirmed = 3;
  diagnostics.tracks_coasting = 1;
  diagnostics.tracks_created = 1;
  diagnostics.tracks_deleted = 0;
  diagnostics.id_switches = 0;
  diagnostics.obstacles_published = 3;
  diagnostics.obstacles_dropped_stale = 1;
  diagnostics.obstacles_dropped_unconfirmed = 2;
  diagnostics.obstacles_dropped_radius_floor = 0;
  diagnostics.timing = MakeStageTiming(variant);
  diagnostics.pipeline_latency_s = 0.135;

  // comparison_valid true means the matched/unmatched fields must close against
  // oracle_count; false means they must all be empty. Both are exercised across variants.
  if (variant % 2 == 0) {
    diagnostics.comparison_valid = true;
    diagnostics.matched.push_back(MakeMatchedPairError(0));
    diagnostics.matched.push_back(MakeMatchedPairError(1));
    diagnostics.unmatched_oracle = 1;
    diagnostics.oracle_count = 3;
    diagnostics.unmatched_estimated = 2;
  } else {
    diagnostics.comparison_valid = false;
  }
  return diagnostics;
}

inline PerceptionFrame MakePerceptionFrame(int variant) {
  PerceptionFrame frame;
  frame.sequence = 700 + static_cast<uint64_t>(variant);
  frame.stamp_s = Perturb(13.5, variant);
  frame.publish_stamp_s = frame.stamp_s + 0.037;
  frame.transform = MakeFrameTransformSnapshot(variant);

  frame.raw_cloud = MakeTimedPointCloud(variant);
  frame.deskewed_cloud = MakeDeskewedPointCloud(variant);
  frame.gravity_aligned_cloud = MakeGravityAlignedCloud(variant);
  frame.labeled_cloud = MakeLabeledPointCloud(variant);
  frame.projected_scan = MakeProjectedScan(variant);

  frame.clusters.push_back(MakeCluster2D(0));
  frame.clusters.push_back(MakeCluster2D(1));
  frame.primitives.push_back(MakeFittedPrimitive2D(0));
  frame.primitives.push_back(MakeFittedPrimitive2D(1));
  // Observations must be in kWorld by the time they are in a frame the tracker consumed.
  CircleObservation observation = MakeCircleObservation(0);
  observation.frame = FrameId::kWorld;
  frame.observations.push_back(observation);
  frame.tracks.push_back(MakeTrackState2D(0));
  frame.tracks.push_back(MakeTrackState2D(1));

  // Every SafetyObstacle must trace to a PerceptionObstacle of the same id - the contract
  // rejects an orphan, so the fixture builds the pair rather than two independent lists.
  frame.obstacles.push_back(MakePerceptionObstacle(0));
  frame.obstacles.push_back(MakePerceptionObstacle(1));
  frame.safety_obstacles.push_back(MakeSafetyObstacle(0));

  frame.diagnostics = MakeEstimationDiagnostics(variant);
  frame.timing = MakeStageTiming(variant);

  // All nine retained, so the frame's retained-gated sub-validations all actually run.
  frame.retained.raw_cloud = true;
  frame.retained.deskewed_cloud = true;
  frame.retained.gravity_aligned_cloud = true;
  frame.retained.labeled_cloud = true;
  frame.retained.projected_scan = true;
  frame.retained.clusters = true;
  frame.retained.primitives = true;
  frame.retained.observations = true;
  frame.retained.tracks = true;

  frame.valid = true;
  frame.invalid_reason = FrameInvalidReason::kNone;
  return frame;
}

}  // namespace perception_fixtures

#endif  // PERCEPTION_TESTS_FIXTURES_SYNTHETIC_CONTRACTS_H_
