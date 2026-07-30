// THE serialization mechanism: one field list per contract, reused by every codec.
//
// WHY IT IS SHAPED LIKE THIS. There are twenty-four dumpable record types and three codecs
// (binary write, binary read, JSON write, JSON read - four). Writing a bespoke writer and a
// bespoke reader per type would be ~48 hand-maintained functions that must agree field for
// field, and the first time one of them drifted the symptom would be a silently
// misinterpreted dump, not a compile error. Instead each contract declares its fields
// exactly ONCE, in a `VisitFields` overload below, and every codec is a visitor over those
// declarations. Write and read cannot disagree because they walk the same list.
//
// Adding a field to a contract is therefore a one-line change here plus a dump-schema
// version bump in dump_format.h. Forgetting the field list entirely is a compile error at
// the first codec instantiation, not a runtime hole.
//
// VISITOR CONTRACT. A codec must provide:
//
//   template <class T> void Prim(const char* name, T& value);
//        arithmetic (bool, integral, float, double) and std::string
//   template <class E> void Enm(const char* name, E& value);
//        any enum; travels as uint32 regardless of its declared underlying type
//   void Eig(const char* name, X& value);
//        Eigen::Vector2d / Vector3d / Vector3f / Vector4d / Matrix3d / Isometry3d
//   template <class S> void Sub(const char* name, S& value);
//        a nested contract that has its own VisitFields
//   template <class T> void Arr(const char* name, std::vector<T>& value);
//        a repeated field; T may be primitive, enum, Eigen or contract
//   template <std::size_t N> void FixedArr(const char* name, std::array<double, N>& value);
//        a fixed-length numeric array (StageTiming)
//
// CONST-CORRECTNESS. The write codecs take a non-const reference too, and simply do not
// modify it. A separate const overload set would double this file for no benefit; the
// writers wrap their argument with a const_cast at exactly one place each, which is
// documented there.
#ifndef PERCEPTION_CORE_DIAGNOSTICS_CONTRACT_FIELDS_H_
#define PERCEPTION_CORE_DIAGNOSTICS_CONTRACT_FIELDS_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

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

namespace perception::core::diagnostics {

// ---------------------------------------------------------------------------------------
// Element-kind traits. `Arr` needs to know which of the four visitor entry points to use
// for its element type, and it cannot ask the visitor - overload resolution inside a
// template would pick the wrong one silently for enums (which are convertible to int).
// ---------------------------------------------------------------------------------------
template <class T>
struct IsContractStruct : std::false_type {};

template <class T>
struct IsEigenSerializable : std::false_type {};

template <> struct IsEigenSerializable<Eigen::Vector2d>   : std::true_type {};
template <> struct IsEigenSerializable<Eigen::Vector3d>   : std::true_type {};
template <> struct IsEigenSerializable<Eigen::Vector3f>   : std::true_type {};
template <> struct IsEigenSerializable<Eigen::Vector4d>   : std::true_type {};
template <> struct IsEigenSerializable<Eigen::Matrix3d>   : std::true_type {};
template <> struct IsEigenSerializable<Eigen::Isometry3d> : std::true_type {};

#define PERCEPTION_DECLARE_CONTRACT_STRUCT(Type) \
  template <> struct IsContractStruct<Type> : std::true_type {}

// ---------------------------------------------------------------------------------------
// The field lists. Order IS the byte order; changing it changes the format.
// ---------------------------------------------------------------------------------------

template <class V>
void VisitFields(V& v, RayPattern& x) {
  v.Arr("directions", x.directions);
  v.Arr("time_offsets_s", x.time_offsets_s);
  v.Prim("period_s", x.period_s);
  v.Prim("azimuth_rays", x.azimuth_rays);
  v.Prim("elevation_rays", x.elevation_rays);
  v.Enm("frame", x.frame);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(RayPattern);

template <class V>
void VisitFields(V& v, RawTimedPoint& x) {
  v.Eig("position", x.position);
  v.Prim("range_m", x.range_m);
  v.Prim("time_offset_s", x.time_offset_s);
  v.Prim("ray_index", x.ray_index);
  v.Prim("geom_id", x.geom_id);
  v.Prim("body_id", x.body_id);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(RawTimedPoint);

template <class V>
void VisitFields(V& v, TimedPointCloud& x) {
  v.Arr("points", x.points);
  v.Prim("stamp_s", x.stamp_s);
  v.Prim("sequence", x.sequence);
  v.Enm("frame", x.frame);
  v.Enm("motion_compensation", x.motion_compensation);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(TimedPointCloud);

template <class V>
void VisitFields(V& v, ScanStats& x) {
  v.Prim("rays_cast", x.rays_cast);
  v.Prim("raw_hits", x.raw_hits);
  v.Prim("no_hit", x.no_hit);
  v.Prim("self_rejected", x.self_rejected);
  v.Prim("range_rejected", x.range_rejected);
  v.Prim("accepted", x.accepted);
  v.Prim("aperture_transmitted", x.aperture_transmitted);
  v.Prim("aperture_diagnostic_valid", x.aperture_diagnostic_valid);
  v.Prim("scan_wall_time_us", x.scan_wall_time_us);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(ScanStats);

template <class V>
void VisitFields(V& v, FrameTransformSnapshot& x) {
  v.Prim("stamp_s", x.stamp_s);
  v.Eig("world_from_base", x.world_from_base);
  v.Eig("world_from_sensor", x.world_from_sensor);
  v.Eig("base_from_sensor", x.base_from_sensor);
  v.Eig("gravity_in_base", x.gravity_in_base);
  v.Prim("valid", x.valid);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(FrameTransformSnapshot);

template <class V>
void VisitFields(V& v, DeskewedPointCloud& x) {
  v.Arr("points", x.points);
  v.Prim("stamp_s", x.stamp_s);
  v.Prim("sequence", x.sequence);
  v.Enm("frame", x.frame);
  v.Enm("motion_compensation", x.motion_compensation);
  v.Prim("window_begin_s", x.window_begin_s);
  v.Prim("window_end_s", x.window_end_s);
  v.Prim("max_correction_m", x.max_correction_m);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(DeskewedPointCloud);

template <class V>
void VisitFields(V& v, GravityAlignedCloud& x) {
  v.Arr("points", x.points);
  v.Prim("stamp_s", x.stamp_s);
  v.Prim("sequence", x.sequence);
  v.Enm("frame", x.frame);
  v.Enm("motion_compensation", x.motion_compensation);
  v.Eig("alignment_from_base", x.alignment_from_base);
  v.Prim("removed_roll_rad", x.removed_roll_rad);
  v.Prim("removed_pitch_rad", x.removed_pitch_rad);
  v.Prim("residual_tilt_rad", x.residual_tilt_rad);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(GravityAlignedCloud);

template <class V>
void VisitFields(V& v, GroundPlane& x) {
  v.Eig("normal", x.normal);
  v.Prim("offset_m", x.offset_m);
  v.Prim("inlier_fraction", x.inlier_fraction);
  v.Prim("fit_input_points", x.fit_input_points);
  v.Prim("valid", x.valid);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(GroundPlane);

template <class V>
void VisitFields(V& v, GroundSegmentationResult& x) {
  v.Prim("total_points", x.total_points);
  v.Prim("ground_points", x.ground_points);
  v.Prim("non_ground_points", x.non_ground_points);
  v.Prim("self_points", x.self_points);
  v.Prim("invalid_points", x.invalid_points);
  v.Sub("plane", x.plane);
  v.Prim("wall_time_us", x.wall_time_us);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(GroundSegmentationResult);

template <class V>
void VisitFields(V& v, LabeledPointCloud& x) {
  v.Arr("points", x.points);
  v.Arr("labels", x.labels);
  v.Prim("stamp_s", x.stamp_s);
  v.Prim("sequence", x.sequence);
  v.Enm("frame", x.frame);
  v.Enm("motion_compensation", x.motion_compensation);
  v.Sub("segmentation", x.segmentation);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(LabeledPointCloud);

template <class V>
void VisitFields(V& v, ProjectionStats& x) {
  v.Prim("input_points", x.input_points);
  v.Prim("rejected_non_finite", x.rejected_non_finite);
  v.Prim("rejected_height_band", x.rejected_height_band);
  v.Prim("rejected_range", x.rejected_range);
  v.Prim("rejected_angle", x.rejected_angle);
  v.Prim("binned_points", x.binned_points);
  v.Prim("occupied_bins", x.occupied_bins);
  v.Prim("total_bins", x.total_bins);
  v.Prim("wall_time_us", x.wall_time_us);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(ProjectionStats);

template <class V>
void VisitFields(V& v, ProjectedScan& x) {
  v.Arr("ranges", x.ranges);
  v.Arr("bin_point_counts", x.bin_point_counts);
  v.Prim("angle_min_rad", x.angle_min_rad);
  v.Prim("angle_max_rad", x.angle_max_rad);
  v.Prim("angle_increment_rad", x.angle_increment_rad);
  v.Prim("time_increment_s", x.time_increment_s);
  v.Prim("scan_time_s", x.scan_time_s);
  v.Prim("range_min_m", x.range_min_m);
  v.Prim("range_max_m", x.range_max_m);
  v.Prim("height_band_min_m", x.height_band_min_m);
  v.Prim("height_band_max_m", x.height_band_max_m);
  v.Prim("stamp_s", x.stamp_s);
  v.Prim("sequence", x.sequence);
  v.Enm("frame", x.frame);
  v.Enm("motion_compensation", x.motion_compensation);
  v.Sub("stats", x.stats);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(ProjectedScan);

template <class V>
void VisitFields(V& v, Cluster2D& x) {
  v.Prim("first_index", x.first_index);
  v.Prim("last_index", x.last_index);
  v.Prim("point_count", x.point_count);
  v.Prim("first_bin", x.first_bin);
  v.Prim("last_bin", x.last_bin);
  v.Prim("front_visible", x.front_visible);
  v.Prim("back_visible", x.back_visible);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(Cluster2D);

template <class V>
void VisitFields(V& v, FittedPrimitive2D& x) {
  v.Enm("kind", x.kind);
  v.Eig("first_point", x.first_point);
  v.Eig("last_point", x.last_point);
  v.Eig("normal", x.normal);
  v.Eig("center", x.center);
  v.Prim("radius_m", x.radius_m);
  v.Prim("fit_residual_m", x.fit_residual_m);
  v.Prim("point_count", x.point_count);
  v.Prim("cluster_index", x.cluster_index);
  v.Prim("front_visible", x.front_visible);
  v.Prim("back_visible", x.back_visible);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(FittedPrimitive2D);

template <class V>
void VisitFields(V& v, CircleObservation& x) {
  v.Eig("center", x.center);
  v.Prim("radius_fitted_m", x.radius_fitted_m);
  v.Prim("radius_enclosing_m", x.radius_enclosing_m);
  v.Prim("arc_extent_rad", x.arc_extent_rad);
  v.Prim("arc_length_m", x.arc_length_m);
  v.Prim("fit_residual_m", x.fit_residual_m);
  v.Prim("range_to_center_m", x.range_to_center_m);
  v.Prim("sigma_center_m", x.sigma_center_m);
  v.Prim("sigma_radius_m", x.sigma_radius_m);
  v.Prim("point_count", x.point_count);
  v.Prim("primitive_index", x.primitive_index);
  v.Prim("front_visible", x.front_visible);
  v.Prim("back_visible", x.back_visible);
  v.Prim("stamp_s", x.stamp_s);
  v.Enm("frame", x.frame);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(CircleObservation);

template <class V>
void VisitFields(V& v, TrackState2D& x) {
  v.Prim("id", x.id);
  v.Eig("center", x.center);
  v.Eig("velocity", x.velocity);
  v.Prim("radius_m", x.radius_m);
  v.Prim("radius_rate_mps", x.radius_rate_mps);
  v.Eig("center_variance", x.center_variance);
  v.Eig("velocity_variance", x.velocity_variance);
  v.Prim("radius_variance", x.radius_variance);
  v.Prim("radius_rate_variance", x.radius_rate_variance);
  v.Prim("created_stamp_s", x.created_stamp_s);
  v.Prim("last_update_stamp_s", x.last_update_stamp_s);
  v.Prim("last_measurement_stamp_s", x.last_measurement_stamp_s);
  v.Prim("hits", x.hits);
  v.Prim("misses", x.misses);
  v.Prim("consecutive_misses", x.consecutive_misses);
  v.Enm("status", x.status);
  v.Enm("frame", x.frame);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(TrackState2D);

template <class V>
void VisitFields(V& v, TrackPrediction2D& x) {
  v.Prim("id", x.id);
  v.Prim("stamp_s", x.stamp_s);
  v.Prim("source_stamp_s", x.source_stamp_s);
  v.Eig("center", x.center);
  v.Eig("velocity", x.velocity);
  v.Prim("radius_m", x.radius_m);
  v.Eig("center_variance", x.center_variance);
  v.Eig("velocity_variance", x.velocity_variance);
  v.Prim("radius_variance", x.radius_variance);
  v.Enm("frame", x.frame);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(TrackPrediction2D);

template <class V>
void VisitFields(V& v, PerceptionObstacle& x) {
  v.Prim("id", x.id);
  v.Eig("center", x.center);
  v.Eig("velocity", x.velocity);
  v.Prim("radius_true_m", x.radius_true_m);
  v.Eig("center_variance", x.center_variance);
  v.Eig("velocity_variance", x.velocity_variance);
  v.Prim("radius_variance", x.radius_variance);
  v.Prim("last_update_stamp_s", x.last_update_stamp_s);
  v.Prim("track_age_s", x.track_age_s);
  v.Prim("confidence", x.confidence);
  v.Enm("frame", x.frame);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(PerceptionObstacle);

template <class V>
void VisitFields(V& v, SafetyObstacle& x) {
  v.Prim("id", x.id);
  v.Eig("center", x.center);
  v.Eig("velocity", x.velocity);
  v.Prim("radius_true_m", x.radius_true_m);
  v.Prim("radius_inflated_m", x.radius_inflated_m);
  v.Prim("position_inflation_m", x.position_inflation_m);
  v.Prim("stamp_s", x.stamp_s);
  v.Prim("age_s", x.age_s);
  v.Prim("valid", x.valid);
  v.Enm("source", x.source);
  v.Enm("frame", x.frame);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(SafetyObstacle);

template <class V>
void VisitFields(V& v, OracleObstacleState& x) {
  v.Prim("id", x.id);
  v.Eig("center", x.center);
  v.Eig("velocity", x.velocity);
  v.Prim("radius_m", x.radius_m);
  v.Prim("stamp_s", x.stamp_s);
  v.Enm("frame", x.frame);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(OracleObstacleState);

template <class V>
void VisitFields(V& v, StageTiming& x) {
  // Fixed length by construction: the array is indexed by PipelineStage, so a dump whose
  // length disagreed with kPipelineStageCount would be a stage-enum change, i.e. a schema
  // change, not a data variation. The codecs enforce the length rather than trusting it.
  v.FixedArr("wall_time_us", x.wall_time_us);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(StageTiming);

template <class V>
void VisitFields(V& v, MatchedPairError& x) {
  v.Prim("track_id", x.track_id);
  v.Prim("oracle_id", x.oracle_id);
  v.Prim("center_error_m", x.center_error_m);
  v.Prim("velocity_error_mps", x.velocity_error_mps);
  v.Prim("radius_error_m", x.radius_error_m);
  v.Prim("stamp_s", x.stamp_s);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(MatchedPairError);

template <class V>
void VisitFields(V& v, EstimationDiagnostics& x) {
  v.Prim("sequence", x.sequence);
  v.Prim("stamp_s", x.stamp_s);
  v.Sub("scan", x.scan);
  v.Sub("segmentation", x.segmentation);
  v.Sub("projection", x.projection);
  v.Prim("clusters_found", x.clusters_found);
  v.Prim("primitives_fitted", x.primitives_fitted);
  v.Prim("circles_detected", x.circles_detected);
  v.Prim("circles_rejected_radius", x.circles_rejected_radius);
  v.Prim("circles_rejected_visibility", x.circles_rejected_visibility);
  v.Prim("tracks_tentative", x.tracks_tentative);
  v.Prim("tracks_confirmed", x.tracks_confirmed);
  v.Prim("tracks_coasting", x.tracks_coasting);
  v.Prim("tracks_created", x.tracks_created);
  v.Prim("tracks_deleted", x.tracks_deleted);
  v.Prim("id_switches", x.id_switches);
  v.Prim("obstacles_published", x.obstacles_published);
  v.Prim("obstacles_dropped_stale", x.obstacles_dropped_stale);
  v.Prim("obstacles_dropped_unconfirmed", x.obstacles_dropped_unconfirmed);
  v.Prim("obstacles_dropped_radius_floor", x.obstacles_dropped_radius_floor);
  v.Prim("comparison_valid", x.comparison_valid);
  v.Arr("matched", x.matched);
  v.Prim("oracle_count", x.oracle_count);
  v.Prim("unmatched_oracle", x.unmatched_oracle);
  v.Prim("unmatched_estimated", x.unmatched_estimated);
  v.Sub("timing", x.timing);
  v.Prim("pipeline_latency_s", x.pipeline_latency_s);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(EstimationDiagnostics);

template <class V>
void VisitFields(V& v, RetainedStages& x) {
  v.Prim("raw_cloud", x.raw_cloud);
  v.Prim("deskewed_cloud", x.deskewed_cloud);
  v.Prim("gravity_aligned_cloud", x.gravity_aligned_cloud);
  v.Prim("labeled_cloud", x.labeled_cloud);
  v.Prim("projected_scan", x.projected_scan);
  v.Prim("clusters", x.clusters);
  v.Prim("primitives", x.primitives);
  v.Prim("observations", x.observations);
  v.Prim("tracks", x.tracks);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(RetainedStages);

template <class V>
void VisitFields(V& v, PerceptionFrame& x) {
  v.Prim("sequence", x.sequence);
  v.Prim("stamp_s", x.stamp_s);
  v.Prim("publish_stamp_s", x.publish_stamp_s);
  v.Sub("transform", x.transform);
  // The intermediate stage outputs. Each is written unconditionally; when the runner did
  // not retain one it is simply empty, and `retained` below is what says which of those
  // emptinesses was a decision rather than a result.
  v.Sub("raw_cloud", x.raw_cloud);
  v.Sub("deskewed_cloud", x.deskewed_cloud);
  v.Sub("gravity_aligned_cloud", x.gravity_aligned_cloud);
  v.Sub("labeled_cloud", x.labeled_cloud);
  v.Sub("projected_scan", x.projected_scan);
  v.Arr("clusters", x.clusters);
  v.Arr("primitives", x.primitives);
  v.Arr("observations", x.observations);
  v.Arr("tracks", x.tracks);
  v.Arr("obstacles", x.obstacles);
  v.Arr("safety_obstacles", x.safety_obstacles);
  v.Sub("diagnostics", x.diagnostics);
  v.Sub("timing", x.timing);
  v.Sub("retained", x.retained);
  v.Prim("valid", x.valid);
  v.Enm("invalid_reason", x.invalid_reason);
}
PERCEPTION_DECLARE_CONTRACT_STRUCT(PerceptionFrame);

// ---------------------------------------------------------------------------------------
// Shared array-element dispatch, so all four codecs agree on how to treat an element type.
// A codec implements Arr() by calling this with a lambda-free element functor.
// ---------------------------------------------------------------------------------------
template <class V, class T>
void VisitArrayElement(V& v, const char* name, T& element) {
  if constexpr (IsContractStruct<T>::value) {
    v.Sub(name, element);
  } else if constexpr (IsEigenSerializable<T>::value) {
    v.Eig(name, element);
  } else if constexpr (std::is_enum_v<T>) {
    v.Enm(name, element);
  } else {
    v.Prim(name, element);
  }
}

}  // namespace perception::core::diagnostics

#endif  // PERCEPTION_CORE_DIAGNOSTICS_CONTRACT_FIELDS_H_
