#include "perception/integration/perception_config.h"

#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace perception::integration {
namespace {

[[noreturn]] void Fail(const std::string& message) {
  throw std::runtime_error("perception config: " + message);
}

// Rejecting unknown keys is the whole point of a schema, so every map we descend into is
// checked against its allowed key set.
void RejectUnknownKeys(const YAML::Node& node, const std::string& where,
                       const std::set<std::string>& allowed) {
  if (!node || !node.IsMap()) {
    return;
  }
  for (const auto& entry : node) {
    const auto key = entry.first.as<std::string>();
    if (allowed.count(key) == 0) {
      std::ostringstream message;
      message << "unknown key '" << key << "' in '" << where << "'. Allowed:";
      for (const auto& name : allowed) {
        message << ' ' << name;
      }
      Fail(message.str());
    }
  }
}

Eigen::Vector3d ReadVector3(const YAML::Node& node, const std::string& where) {
  if (!node.IsSequence() || node.size() != 3) {
    Fail(where + " must be a sequence of exactly 3 numbers");
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

Eigen::Vector4d ReadVector4(const YAML::Node& node, const std::string& where) {
  if (!node.IsSequence() || node.size() != 4) {
    Fail(where + " must be a sequence of exactly 4 numbers");
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>(),
          node[3].as<double>()};
}

Eigen::Vector2d ReadVector2(const YAML::Node& node, const std::string& where) {
  if (!node.IsSequence() || node.size() != 2) {
    Fail(where + " must be a sequence of exactly 2 numbers");
  }
  return {node[0].as<double>(), node[1].as<double>()};
}

void LoadExtrinsic(const YAML::Node& node, LidarExtrinsicConfig& out) {
  RejectUnknownKeys(node, "perception.lidar.extrinsic",
                    {"parent_body", "site_name", "translation_xyz_m", "rotation_rpy_rad"});
  if (node["parent_body"]) out.parent_body = node["parent_body"].as<std::string>();
  if (node["site_name"]) out.site_name = node["site_name"].as<std::string>();
  if (node["translation_xyz_m"]) {
    out.translation_xyz_m =
        ReadVector3(node["translation_xyz_m"], "perception.lidar.extrinsic.translation_xyz_m");
  }
  if (node["rotation_rpy_rad"]) {
    out.rotation_rpy_rad =
        ReadVector3(node["rotation_rpy_rad"], "perception.lidar.extrinsic.rotation_rpy_rad");
  }
}

void LoadSensor(const YAML::Node& node, LidarSensorModelConfig& out) {
  RejectUnknownKeys(node, "perception.lidar.sensor",
                    {"vertical_fov_deg", "horizontal_fov_deg", "azimuth_rays",
                     "elevation_rays", "min_range_m", "max_range_m", "scan_rate_hz",
                     "range_noise_std_m", "apply_range_noise", "seed",
                     "angular_noise_deg_1sigma", "body_size_mm"});
  if (node["vertical_fov_deg"]) {
    const auto fov = ReadVector2(node["vertical_fov_deg"], "perception.lidar.sensor.vertical_fov_deg");
    out.vertical_fov_deg_min = fov[0];
    out.vertical_fov_deg_max = fov[1];
  }
  if (node["horizontal_fov_deg"]) out.horizontal_fov_deg = node["horizontal_fov_deg"].as<double>();
  if (node["azimuth_rays"]) out.azimuth_rays = node["azimuth_rays"].as<int>();
  if (node["elevation_rays"]) out.elevation_rays = node["elevation_rays"].as<int>();
  if (node["min_range_m"]) out.min_range_m = node["min_range_m"].as<double>();
  if (node["max_range_m"]) out.max_range_m = node["max_range_m"].as<double>();
  if (node["scan_rate_hz"]) out.scan_rate_hz = node["scan_rate_hz"].as<double>();
  if (node["range_noise_std_m"]) out.range_noise_std_m = node["range_noise_std_m"].as<double>();
  if (node["apply_range_noise"]) out.apply_range_noise = node["apply_range_noise"].as<bool>();
  if (node["seed"]) out.seed = node["seed"].as<uint32_t>();
  if (node["angular_noise_deg_1sigma"]) {
    out.angular_noise_deg_1sigma = node["angular_noise_deg_1sigma"].as<double>();
  }
  if (node["body_size_mm"]) {
    out.body_size_mm = ReadVector3(node["body_size_mm"], "perception.lidar.sensor.body_size_mm");
  }
}

// Reads a string and checks it against a closed set. A mode or backend name that does not
// exist has to fail loudly: silently falling back to a default would run the wrong
// pipeline while the config file says otherwise.
std::string ReadEnum(const YAML::Node& node, const std::string& where,
                     const std::vector<std::string>& allowed) {
  const auto value = node.as<std::string>();
  for (const auto& candidate : allowed) {
    if (value == candidate) return value;
  }
  std::ostringstream message;
  message << where << " has unsupported value '" << value << "'. Allowed:";
  for (const auto& candidate : allowed) {
    message << ' ' << candidate;
  }
  Fail(message.str());
}

void LoadFrames(const YAML::Node& node, FramesConfig& out) {
  RejectUnknownKeys(node, "perception.frames",
                    {"base_body", "world_frame_name", "snapshot_ring_capacity"});
  if (node["base_body"]) out.base_body = node["base_body"].as<std::string>();
  if (node["world_frame_name"]) out.world_frame_name = node["world_frame_name"].as<std::string>();
  if (node["snapshot_ring_capacity"]) {
    out.snapshot_ring_capacity = node["snapshot_ring_capacity"].as<int>();
  }
}

void LoadMotionCompensation(const YAML::Node& node, MotionCompensationConfig& out) {
  RejectUnknownKeys(node, "perception.motion_compensation",
                    {"deskew_enabled", "gravity_align_enabled", "interpolation",
                     "max_snapshot_gap_s", "max_extrapolation_s", "require_full_window"});
  if (node["deskew_enabled"]) out.deskew_enabled = node["deskew_enabled"].as<bool>();
  if (node["gravity_align_enabled"]) {
    out.gravity_align_enabled = node["gravity_align_enabled"].as<bool>();
  }
  if (node["interpolation"]) {
    out.interpolation = ReadEnum(node["interpolation"],
                                 "perception.motion_compensation.interpolation",
                                 {"nearest", "linear"});
  }
  if (node["max_snapshot_gap_s"]) out.max_snapshot_gap_s = node["max_snapshot_gap_s"].as<double>();
  if (node["max_extrapolation_s"]) {
    out.max_extrapolation_s = node["max_extrapolation_s"].as<double>();
  }
  if (node["require_full_window"]) {
    out.require_full_window = node["require_full_window"].as<bool>();
  }
}

void LoadCoreSelfFilter(const YAML::Node& node, CoreSelfFilterConfig& out) {
  RejectUnknownKeys(node, "perception.self_filter", {"enabled", "radius_m", "z_min_m", "z_max_m"});
  if (node["enabled"]) out.enabled = node["enabled"].as<bool>();
  if (node["radius_m"]) out.radius_m = node["radius_m"].as<double>();
  if (node["z_min_m"]) out.z_min_m = node["z_min_m"].as<double>();
  if (node["z_max_m"]) out.z_max_m = node["z_max_m"].as<double>();
}

void LoadSegmentation(const YAML::Node& node, SegmentationConfig& out) {
  RejectUnknownKeys(node, "perception.segmentation", {"backend", "zband"});
  if (node["backend"]) {
    out.backend = ReadEnum(node["backend"], "perception.segmentation.backend",
                           {"zband", "travel"});
  }
  const YAML::Node zband = node["zband"];
  if (zband) {
    RejectUnknownKeys(zband, "perception.segmentation.zband",
                      {"max_ground_height_m", "band_tolerance_m", "refine_with_plane",
                       "plane_ransac_distance_m", "plane_ransac_iterations", "max_slope_deg"});
    if (zband["max_ground_height_m"]) {
      out.zband.max_ground_height_m = zband["max_ground_height_m"].as<double>();
    }
    if (zband["band_tolerance_m"]) out.zband.band_tolerance_m = zband["band_tolerance_m"].as<double>();
    if (zband["refine_with_plane"]) {
      out.zband.refine_with_plane = zband["refine_with_plane"].as<bool>();
    }
    if (zband["plane_ransac_distance_m"]) {
      out.zband.plane_ransac_distance_m = zband["plane_ransac_distance_m"].as<double>();
    }
    if (zband["plane_ransac_iterations"]) {
      out.zband.plane_ransac_iterations = zband["plane_ransac_iterations"].as<int>();
    }
    if (zband["max_slope_deg"]) out.zband.max_slope_deg = zband["max_slope_deg"].as<double>();
  }
}

void LoadProjection(const YAML::Node& node, ProjectionConfig& out) {
  RejectUnknownKeys(node, "perception.projection",
                    {"bins", "range_min_m", "range_max_m", "height_band_min_m",
                     "height_band_max_m", "collect_bin_histogram"});
  if (node["bins"]) out.bins = node["bins"].as<int>();
  if (node["range_min_m"]) out.range_min_m = node["range_min_m"].as<double>();
  if (node["range_max_m"]) out.range_max_m = node["range_max_m"].as<double>();
  if (node["height_band_min_m"]) out.height_band_min_m = node["height_band_min_m"].as<double>();
  if (node["height_band_max_m"]) out.height_band_max_m = node["height_band_max_m"].as<double>();
  if (node["collect_bin_histogram"]) {
    out.collect_bin_histogram = node["collect_bin_histogram"].as<bool>();
  }
}

void LoadDetection(const YAML::Node& node, DetectionConfig& out) {
  RejectUnknownKeys(node, "perception.detection",
                    {"min_group_points", "max_group_distance_m", "distance_proportion",
                     "max_split_distance_m", "max_merge_separation_m", "max_merge_spread_m",
                     "max_circle_radius_m", "radius_enlargement_m", "circles_from_visibles",
                     "use_split_and_merge", "discard_converted_segments", "max_clusters",
                     "max_primitives", "max_circles"});
  if (node["min_group_points"]) out.min_group_points = node["min_group_points"].as<int>();
  if (node["max_group_distance_m"]) {
    out.max_group_distance_m = node["max_group_distance_m"].as<double>();
  }
  if (node["distance_proportion"]) {
    out.distance_proportion = node["distance_proportion"].as<double>();
  }
  if (node["max_split_distance_m"]) {
    out.max_split_distance_m = node["max_split_distance_m"].as<double>();
  }
  if (node["max_merge_separation_m"]) {
    out.max_merge_separation_m = node["max_merge_separation_m"].as<double>();
  }
  if (node["max_merge_spread_m"]) out.max_merge_spread_m = node["max_merge_spread_m"].as<double>();
  if (node["max_circle_radius_m"]) {
    out.max_circle_radius_m = node["max_circle_radius_m"].as<double>();
  }
  if (node["radius_enlargement_m"]) {
    out.radius_enlargement_m = node["radius_enlargement_m"].as<double>();
  }
  if (node["circles_from_visibles"]) {
    out.circles_from_visibles = node["circles_from_visibles"].as<bool>();
  }
  if (node["use_split_and_merge"]) {
    out.use_split_and_merge = node["use_split_and_merge"].as<bool>();
  }
  if (node["discard_converted_segments"]) {
    out.discard_converted_segments = node["discard_converted_segments"].as<bool>();
  }
  if (node["max_clusters"]) out.max_clusters = node["max_clusters"].as<int>();
  if (node["max_primitives"]) out.max_primitives = node["max_primitives"].as<int>();
  if (node["max_circles"]) out.max_circles = node["max_circles"].as<int>();
}

void LoadTracking(const YAML::Node& node, TrackingConfig& out) {
  RejectUnknownKeys(node, "perception.tracking",
                    {"process_variance", "process_rate_variance", "measurement_variance",
                     "min_correspondence_cost_m", "association_radius_weight",
                     "measurement_sigma_scale", "measurement_sigma_floor_m",
                     "initial_rate_variance", "confirm_hits", "delete_misses",
                     "max_coast_s", "enable_fusion", "enable_fission", "max_tracks"});
  if (node["process_variance"]) out.process_variance = node["process_variance"].as<double>();
  if (node["process_rate_variance"]) {
    out.process_rate_variance = node["process_rate_variance"].as<double>();
  }
  if (node["measurement_variance"]) {
    out.measurement_variance = node["measurement_variance"].as<double>();
  }
  if (node["min_correspondence_cost_m"]) {
    out.min_correspondence_cost_m = node["min_correspondence_cost_m"].as<double>();
  }
  if (node["association_radius_weight"]) {
    out.association_radius_weight = node["association_radius_weight"].as<double>();
  }
  if (node["measurement_sigma_scale"]) {
    out.measurement_sigma_scale = node["measurement_sigma_scale"].as<double>();
  }
  if (node["measurement_sigma_floor_m"]) {
    out.measurement_sigma_floor_m = node["measurement_sigma_floor_m"].as<double>();
  }
  if (node["initial_rate_variance"]) {
    out.initial_rate_variance = node["initial_rate_variance"].as<double>();
  }
  if (node["confirm_hits"]) out.confirm_hits = node["confirm_hits"].as<int>();
  if (node["delete_misses"]) out.delete_misses = node["delete_misses"].as<int>();
  if (node["max_coast_s"]) out.max_coast_s = node["max_coast_s"].as<double>();
  if (node["enable_fusion"]) out.enable_fusion = node["enable_fusion"].as<bool>();
  if (node["enable_fission"]) out.enable_fission = node["enable_fission"].as<bool>();
  if (node["max_tracks"]) out.max_tracks = node["max_tracks"].as<int>();
}

void LoadSafety(const YAML::Node& node, SafetyConfig& out) {
  RejectUnknownKeys(node, "perception.safety",
                    {"max_age_s", "min_track_age_s", "min_track_hits",
                     "radius_inflation_k_sigma", "radius_inflation_fixed_m",
                     "latency_inflation_s", "use_enclosing_radius", "min_radius_m",
                     "max_radius_m", "max_speed_mps", "max_obstacles"});
  if (node["max_age_s"]) out.max_age_s = node["max_age_s"].as<double>();
  if (node["min_track_age_s"]) out.min_track_age_s = node["min_track_age_s"].as<double>();
  if (node["min_track_hits"]) out.min_track_hits = node["min_track_hits"].as<int>();
  if (node["radius_inflation_k_sigma"]) {
    out.radius_inflation_k_sigma = node["radius_inflation_k_sigma"].as<double>();
  }
  if (node["radius_inflation_fixed_m"]) {
    out.radius_inflation_fixed_m = node["radius_inflation_fixed_m"].as<double>();
  }
  if (node["latency_inflation_s"]) {
    out.latency_inflation_s = node["latency_inflation_s"].as<double>();
  }
  if (node["use_enclosing_radius"]) {
    out.use_enclosing_radius = node["use_enclosing_radius"].as<bool>();
  }
  if (node["min_radius_m"]) out.min_radius_m = node["min_radius_m"].as<double>();
  if (node["max_radius_m"]) out.max_radius_m = node["max_radius_m"].as<double>();
  if (node["max_speed_mps"]) out.max_speed_mps = node["max_speed_mps"].as<double>();
  if (node["max_obstacles"]) out.max_obstacles = node["max_obstacles"].as<int>();
}

void LoadDpcbfAdapter(const YAML::Node& node, DpcbfAdapterConfig& out) {
  RejectUnknownKeys(node, "perception.dpcbf_adapter",
                    {"preserve_track_ids", "id_offset", "max_obstacles", "drop_invalid"});
  if (node["preserve_track_ids"]) {
    out.preserve_track_ids = node["preserve_track_ids"].as<bool>();
  }
  if (node["id_offset"]) out.id_offset = node["id_offset"].as<int>();
  if (node["max_obstacles"]) out.max_obstacles = node["max_obstacles"].as<int>();
  if (node["drop_invalid"]) out.drop_invalid = node["drop_invalid"].as<bool>();
}

void LoadMode(const YAML::Node& node, ModeConfig& out) {
  RejectUnknownKeys(node, "perception.mode",
                    {"name", "fallback_max_age_s", "fallback_on_invalid_frame",
                     "heartbeat_timeout_s", "scenario_allowlist"});
  if (node["name"]) {
    out.name = ReadEnum(node["name"], "perception.mode.name",
                        {"oracle", "shadow", "compare", "estimated_fallback", "estimated"});
  }
  if (node["fallback_max_age_s"]) {
    out.fallback_max_age_s = node["fallback_max_age_s"].as<double>();
  }
  if (node["fallback_on_invalid_frame"]) {
    out.fallback_on_invalid_frame = node["fallback_on_invalid_frame"].as<bool>();
  }
  if (node["heartbeat_timeout_s"]) {
    out.heartbeat_timeout_s = node["heartbeat_timeout_s"].as<double>();
  }
  if (node["scenario_allowlist"]) {
    const YAML::Node list = node["scenario_allowlist"];
    if (!list.IsSequence()) Fail("perception.mode.scenario_allowlist must be a sequence");
    out.scenario_allowlist.clear();
    for (const auto& entry : list) {
      out.scenario_allowlist.push_back(entry.as<std::string>());
    }
  }
}

void LoadRos2(const YAML::Node& node, Ros2Config& out) {
  RejectUnknownKeys(node, "perception.ros2",
                    {"enabled", "topic_namespace", "publish_rate_hz", "diagnostics_rate_hz",
                     "publish_clouds", "publish_scan", "publish_markers", "publish_tf",
                     "use_sim_time", "debug_level"});
  if (node["enabled"]) out.enabled = node["enabled"].as<bool>();
  if (node["topic_namespace"]) out.topic_namespace = node["topic_namespace"].as<std::string>();
  if (node["publish_rate_hz"]) out.publish_rate_hz = node["publish_rate_hz"].as<double>();
  if (node["diagnostics_rate_hz"]) {
    out.diagnostics_rate_hz = node["diagnostics_rate_hz"].as<double>();
  }
  if (node["publish_clouds"]) out.publish_clouds = node["publish_clouds"].as<bool>();
  if (node["publish_scan"]) out.publish_scan = node["publish_scan"].as<bool>();
  if (node["publish_markers"]) out.publish_markers = node["publish_markers"].as<bool>();
  if (node["publish_tf"]) out.publish_tf = node["publish_tf"].as<bool>();
  if (node["use_sim_time"]) out.use_sim_time = node["use_sim_time"].as<bool>();
  if (node["debug_level"]) out.debug_level = node["debug_level"].as<int>();
}

void LoadDumps(const YAML::Node& node, DumpsConfig& out) {
  RejectUnknownKeys(node, "perception.dumps",
                    {"enabled", "directory", "format", "retain_stage_clouds", "max_frames",
                     "decimation"});
  if (node["enabled"]) out.enabled = node["enabled"].as<bool>();
  if (node["directory"]) out.directory = node["directory"].as<std::string>();
  if (node["format"]) {
    out.format = ReadEnum(node["format"], "perception.dumps.format",
                          {"jsonl", "binary", "both"});
  }
  if (node["retain_stage_clouds"]) {
    out.retain_stage_clouds = node["retain_stage_clouds"].as<bool>();
  }
  if (node["max_frames"]) out.max_frames = node["max_frames"].as<int>();
  if (node["decimation"]) out.decimation = node["decimation"].as<int>();
}

void LoadMetrics(const YAML::Node& node, MetricsConfig& out) {
  RejectUnknownKeys(node, "perception.metrics",
                    {"enabled", "manifest_path", "compare_against_oracle",
                     "association_max_distance_m"});
  if (node["enabled"]) out.enabled = node["enabled"].as<bool>();
  if (node["manifest_path"]) out.manifest_path = node["manifest_path"].as<std::string>();
  if (node["compare_against_oracle"]) {
    out.compare_against_oracle = node["compare_against_oracle"].as<bool>();
  }
  if (node["association_max_distance_m"]) {
    out.association_max_distance_m = node["association_max_distance_m"].as<double>();
  }
}

void LoadProfiling(const YAML::Node& node, ProfilingConfig& out) {
  RejectUnknownKeys(node, "perception.profiling",
                    {"enabled", "per_stage_timing", "allocation_counters", "report_period_s"});
  if (node["enabled"]) out.enabled = node["enabled"].as<bool>();
  if (node["per_stage_timing"]) out.per_stage_timing = node["per_stage_timing"].as<bool>();
  if (node["allocation_counters"]) {
    out.allocation_counters = node["allocation_counters"].as<bool>();
  }
  if (node["report_period_s"]) out.report_period_s = node["report_period_s"].as<double>();
}

// The one section that does NOT reject unknown keys outright - it rejects them unless the
// flag is set, and captures them verbatim when it is, so an experimental run still records
// exactly what was enabled.
void LoadExperimental(const YAML::Node& node, ExperimentalConfig& out) {
  if (node["allow_experimental"]) {
    out.allow_experimental = node["allow_experimental"].as<bool>();
  }
  if (!node.IsMap()) return;
  for (const auto& entry : node) {
    const auto key = entry.first.as<std::string>();
    if (key == "allow_experimental") continue;
    if (!out.allow_experimental) {
      Fail("experimental option '" + key +
           "' is present but perception.experimental.allow_experimental is false");
    }
    out.options[key] = YAML::Dump(entry.second);
  }
}

void LoadVisualization(const YAML::Node& node, LidarVisualizationConfig& out) {
  RejectUnknownKeys(node, "perception.visualization",
                    {"enabled", "viewer_overlay", "draw_points", "draw_rays",
                     "draw_sensor_frame", "point_stride", "ray_stride", "point_size_m",
                     "ray_width_px", "refresh_hz", "point_rgba", "ray_rgba"});
  if (node["enabled"]) out.enabled = node["enabled"].as<bool>();
  if (node["viewer_overlay"]) out.viewer_overlay = node["viewer_overlay"].as<bool>();
  if (node["draw_points"]) out.draw_points = node["draw_points"].as<bool>();
  if (node["draw_rays"]) out.draw_rays = node["draw_rays"].as<bool>();
  if (node["draw_sensor_frame"]) out.draw_sensor_frame = node["draw_sensor_frame"].as<bool>();
  if (node["point_stride"]) out.point_stride = node["point_stride"].as<int>();
  if (node["ray_stride"]) out.ray_stride = node["ray_stride"].as<int>();
  if (node["point_size_m"]) out.point_size_m = node["point_size_m"].as<double>();
  if (node["ray_width_px"]) out.ray_width_px = node["ray_width_px"].as<double>();
  if (node["refresh_hz"]) out.refresh_hz = node["refresh_hz"].as<double>();
  if (node["point_rgba"]) {
    out.point_rgba = ReadVector4(node["point_rgba"], "perception.visualization.point_rgba");
  }
  if (node["ray_rgba"]) {
    out.ray_rgba = ReadVector4(node["ray_rgba"], "perception.visualization.ray_rgba");
  }
}

}  // namespace

void PerceptionConfig::Validate() const {
  if (schema_version != 1) {
    Fail("unsupported schema_version " + std::to_string(schema_version) + " (expected 1)");
  }

  const auto& sensor = lidar.sensor;
  if (sensor.vertical_fov_deg_min >= sensor.vertical_fov_deg_max) {
    Fail("sensor.vertical_fov_deg must be [min, max] with min < max");
  }
  if (sensor.vertical_fov_deg_min < -90.0 || sensor.vertical_fov_deg_max > 90.0) {
    Fail("sensor.vertical_fov_deg must lie within [-90, 90]");
  }
  if (sensor.horizontal_fov_deg <= 0.0 || sensor.horizontal_fov_deg > 360.0) {
    Fail("sensor.horizontal_fov_deg must lie in (0, 360]");
  }
  if (sensor.azimuth_rays < 8) Fail("sensor.azimuth_rays must be >= 8");
  if (sensor.elevation_rays < 2) Fail("sensor.elevation_rays must be >= 2");
  if (sensor.min_range_m < 0.0) Fail("sensor.min_range_m must be >= 0");
  if (!(sensor.min_range_m < sensor.max_range_m)) {
    Fail("sensor.min_range_m must be < sensor.max_range_m");
  }
  if (sensor.scan_rate_hz <= 0.0) Fail("sensor.scan_rate_hz must be > 0");
  if (sensor.range_noise_std_m < 0.0) Fail("sensor.range_noise_std_m must be >= 0");

  // The rotation must be a rotation: rpy is unconstrained in magnitude, but a
  // non-finite entry would silently poison every ray.
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(lidar.extrinsic.translation_xyz_m[i]) ||
        !std::isfinite(lidar.extrinsic.rotation_rpy_rad[i])) {
      Fail("extrinsic translation/rotation must be finite");
    }
  }
  if (lidar.extrinsic.parent_body.empty()) Fail("extrinsic.parent_body must not be empty");
  if (lidar.extrinsic.site_name.empty()) Fail("extrinsic.site_name must not be empty");

  if (lidar.aperture.transparent_geom_group >= 6) {
    Fail("aperture.transparent_geom_group must be < 6 (mjNGROUP); use -1 to disable");
  }
  if (lidar.self_filter.robot_root_body.empty()) {
    Fail("self_filter.robot_root_body must not be empty");
  }

  if (visualization.point_stride < 1) Fail("visualization.point_stride must be >= 1");
  if (visualization.ray_stride < 1) Fail("visualization.ray_stride must be >= 1");
  if (visualization.refresh_hz <= 0.0) Fail("visualization.refresh_hz must be > 0");

  // ---------------------------------------------------------------------------------
  // Sections frozen in the contract/config phase. Within-section checks first, then the
  // cross-section ones, which are the reason this all lives in a single function.
  // ---------------------------------------------------------------------------------
  const double frame_period_s = SensorFramePeriodSeconds();

  // frames
  if (frames.base_body.empty()) Fail("frames.base_body must not be empty");
  if (frames.world_frame_name.empty()) Fail("frames.world_frame_name must not be empty");
  if (frames.snapshot_ring_capacity < 2) {
    Fail("frames.snapshot_ring_capacity must be >= 2 (deskew interpolates between two)");
  }

  // motion_compensation
  if (motion_compensation.interpolation != "nearest" &&
      motion_compensation.interpolation != "linear") {
    Fail("motion_compensation.interpolation must be 'nearest' or 'linear'");
  }
  if (!(motion_compensation.max_snapshot_gap_s > 0.0)) {
    Fail("motion_compensation.max_snapshot_gap_s must be > 0");
  }
  if (motion_compensation.max_extrapolation_s < 0.0) {
    Fail("motion_compensation.max_extrapolation_s must be >= 0");
  }

  // self_filter (core stage)
  if (!(self_filter.radius_m > 0.0)) Fail("self_filter.radius_m must be > 0");
  if (!(self_filter.z_min_m < self_filter.z_max_m)) {
    Fail("self_filter.z_min_m must be < self_filter.z_max_m");
  }

  // segmentation
  if (segmentation.backend != "zband" && segmentation.backend != "travel") {
    Fail("segmentation.backend must be 'zband' or 'travel'");
  }
  if (!(segmentation.zband.band_tolerance_m >= 0.0)) {
    Fail("segmentation.zband.band_tolerance_m must be >= 0");
  }
  if (!(segmentation.zband.plane_ransac_distance_m > 0.0)) {
    Fail("segmentation.zband.plane_ransac_distance_m must be > 0");
  }
  if (segmentation.zband.plane_ransac_iterations < 1) {
    Fail("segmentation.zband.plane_ransac_iterations must be >= 1");
  }
  if (!(segmentation.zband.max_slope_deg > 0.0) || segmentation.zband.max_slope_deg >= 90.0) {
    Fail("segmentation.zband.max_slope_deg must lie in (0, 90)");
  }

  // projection
  if (projection.bins < 8) Fail("projection.bins must be >= 8");
  if (projection.bins > 36000) {
    Fail("projection.bins must be <= 36000 (0.01 deg); larger is a preallocation hazard");
  }
  if (projection.range_min_m < 0.0) Fail("projection.range_min_m must be >= 0");
  if (!(projection.range_min_m < projection.range_max_m)) {
    Fail("projection.range_min_m must be < projection.range_max_m");
  }
  if (!(projection.height_band_min_m < projection.height_band_max_m)) {
    Fail("projection.height_band_min_m must be < projection.height_band_max_m");
  }

  // detection
  if (detection.min_group_points < 2) {
    Fail("detection.min_group_points must be >= 2 (a line fit needs two points)");
  }
  if (!(detection.max_group_distance_m > 0.0)) {
    Fail("detection.max_group_distance_m must be > 0");
  }
  if (!(detection.distance_proportion > 0.0) || detection.distance_proportion >= 1.0) {
    Fail("detection.distance_proportion must lie in (0, 1)");
  }
  if (!(detection.max_split_distance_m > 0.0)) Fail("detection.max_split_distance_m must be > 0");
  if (!(detection.max_merge_separation_m > 0.0)) {
    Fail("detection.max_merge_separation_m must be > 0");
  }
  if (!(detection.max_merge_spread_m > 0.0)) Fail("detection.max_merge_spread_m must be > 0");
  if (!(detection.max_circle_radius_m > 0.0)) Fail("detection.max_circle_radius_m must be > 0");
  if (detection.radius_enlargement_m < 0.0) {
    Fail("detection.radius_enlargement_m must be >= 0");
  }
  // The extractor enlarges a fitted radius and only THEN compares against the cap, so an
  // enlargement at or above the cap rejects every circle it ever fits.
  if (!(detection.radius_enlargement_m < detection.max_circle_radius_m)) {
    Fail("detection.radius_enlargement_m must be < detection.max_circle_radius_m, "
         "or every fitted circle is rejected by construction");
  }
  if (detection.max_clusters < 1) Fail("detection.max_clusters must be >= 1");
  if (detection.max_primitives < detection.max_clusters) {
    Fail("detection.max_primitives must be >= detection.max_clusters "
         "(split-and-merge can only increase the count)");
  }
  if (detection.max_circles < 1) Fail("detection.max_circles must be >= 1");

  // tracking
  if (!(tracking.process_variance > 0.0)) Fail("tracking.process_variance must be > 0");
  if (!(tracking.process_rate_variance > 0.0)) {
    Fail("tracking.process_rate_variance must be > 0");
  }
  if (!(tracking.measurement_variance > 0.0)) Fail("tracking.measurement_variance must be > 0");
  if (!(tracking.min_correspondence_cost_m > 0.0)) {
    Fail("tracking.min_correspondence_cost_m must be > 0");
  }
  // A zero weight would delete the radius channel from the association cost, which is a
  // different rule rather than a down-weighted one; a weight above 1 would give the biased
  // channel more say than upstream gave it, which is the opposite of what the bias measurement
  // calls for.
  if (!(tracking.association_radius_weight > 0.0) || tracking.association_radius_weight > 1.0) {
    Fail("tracking.association_radius_weight must lie in (0, 1]");
  }
  if (!(tracking.measurement_sigma_scale > 0.0)) {
    Fail("tracking.measurement_sigma_scale must be > 0");
  }
  // The floor is what stops a zero measurement sigma driving the Kalman gain to 1 and the
  // covariance to 0, so it may not itself be zero.
  if (!(tracking.measurement_sigma_floor_m > 0.0)) {
    Fail("tracking.measurement_sigma_floor_m must be > 0");
  }
  if (!(tracking.initial_rate_variance > 0.0)) {
    Fail("tracking.initial_rate_variance must be > 0");
  }
  if (tracking.confirm_hits < 1) Fail("tracking.confirm_hits must be >= 1");
  if (tracking.delete_misses < 1) Fail("tracking.delete_misses must be >= 1");
  if (!(tracking.max_coast_s > 0.0)) Fail("tracking.max_coast_s must be > 0");
  if (tracking.max_tracks < 1) Fail("tracking.max_tracks must be >= 1");

  // safety
  if (!(safety.max_age_s > 0.0)) Fail("safety.max_age_s must be > 0");
  if (safety.min_track_age_s < 0.0) Fail("safety.min_track_age_s must be >= 0");
  if (safety.min_track_hits < 1) Fail("safety.min_track_hits must be >= 1");
  if (safety.radius_inflation_k_sigma < 0.0) {
    Fail("safety.radius_inflation_k_sigma must be >= 0");
  }
  if (safety.radius_inflation_fixed_m < 0.0) {
    Fail("safety.radius_inflation_fixed_m must be >= 0");
  }
  if (safety.latency_inflation_s < 0.0) Fail("safety.latency_inflation_s must be >= 0");
  if (!(safety.min_radius_m > 0.0)) Fail("safety.min_radius_m must be > 0");
  if (!(safety.min_radius_m <= safety.max_radius_m)) {
    Fail("safety.min_radius_m must be <= safety.max_radius_m");
  }
  if (!(safety.max_speed_mps > 0.0)) Fail("safety.max_speed_mps must be > 0");
  if (safety.max_obstacles < 1) Fail("safety.max_obstacles must be >= 1");

  // dpcbf_adapter
  if (dpcbf_adapter.max_obstacles < 1) Fail("dpcbf_adapter.max_obstacles must be >= 1");
  if (dpcbf_adapter.id_offset < 0) Fail("dpcbf_adapter.id_offset must be >= 0");

  // mode
  if (mode.name != "oracle" && mode.name != "shadow" && mode.name != "compare" &&
      mode.name != "estimated_fallback" && mode.name != "estimated") {
    Fail("mode.name must be one of oracle|shadow|compare|estimated_fallback|estimated");
  }
  if (!(mode.fallback_max_age_s > 0.0)) Fail("mode.fallback_max_age_s must be > 0");
  if (!(mode.heartbeat_timeout_s > 0.0)) Fail("mode.heartbeat_timeout_s must be > 0");

  // ros2
  if (!(ros2.publish_rate_hz > 0.0)) Fail("ros2.publish_rate_hz must be > 0");
  if (!(ros2.diagnostics_rate_hz > 0.0)) Fail("ros2.diagnostics_rate_hz must be > 0");
  if (ros2.debug_level < 0) Fail("ros2.debug_level must be >= 0");
  if (ros2.topic_namespace.empty()) Fail("ros2.topic_namespace must not be empty");

  // dumps / metrics / profiling
  if (dumps.format != "jsonl" && dumps.format != "binary" && dumps.format != "both") {
    Fail("dumps.format must be 'jsonl', 'binary' or 'both'");
  }
  if (dumps.enabled && dumps.directory.empty()) {
    Fail("dumps.directory must not be empty when dumps.enabled is true");
  }
  if (dumps.max_frames < 0) Fail("dumps.max_frames must be >= 0 (0 means unlimited)");
  if (dumps.decimation < 1) Fail("dumps.decimation must be >= 1");
  if (metrics.enabled && metrics.manifest_path.empty()) {
    Fail("metrics.manifest_path must not be empty when metrics.enabled is true");
  }
  if (!(metrics.association_max_distance_m > 0.0)) {
    Fail("metrics.association_max_distance_m must be > 0");
  }
  if (!(profiling.report_period_s > 0.0)) Fail("profiling.report_period_s must be > 0");

  // ---------------------------------------------------------------------------------
  // CROSS-SECTION constraints. Each one is a coupling that no single section owns and
  // that would otherwise only be discovered as a silent behavioural bug.
  // ---------------------------------------------------------------------------------

  // A frame older than one sensor period arrives stale by definition, so the safety gate
  // would reject every frame the pipeline ever produced.
  if (!(safety.max_age_s >= frame_period_s)) {
    Fail("safety.max_age_s must be >= the sensor frame period (1 / sensor.scan_rate_hz)");
  }

  // The selector must not fall back before the safety stage has had the chance to declare
  // a frame stale, or safety's own staleness gate can never fire.
  if (!(mode.fallback_max_age_s >= safety.max_age_s)) {
    Fail("mode.fallback_max_age_s must be >= safety.max_age_s");
  }

  // The projector cannot see closer than the sensor's blind zone or further than its
  // cutoff; a wider projection window is a claim the sensor cannot back.
  if (projection.range_min_m < lidar.sensor.min_range_m) {
    Fail("projection.range_min_m must be >= sensor.min_range_m");
  }
  if (projection.range_max_m > lidar.sensor.max_range_m) {
    Fail("projection.range_max_m must be <= sensor.max_range_m");
  }

  // The height band and the ground label are a deliberate double gate (risk R6). A band
  // that reaches BELOW the ground threshold re-admits exactly the points segmentation
  // just rejected.
  if (projection.height_band_min_m < segmentation.zband.max_ground_height_m) {
    Fail("projection.height_band_min_m must be >= segmentation.zband.max_ground_height_m");
  }

  // A self-filter cylinder wider than the scan makes every point a self hit.
  if (!(self_filter.radius_m < projection.range_max_m)) {
    Fail("self_filter.radius_m must be < projection.range_max_m");
  }

  // Deskew has to interpolate across the whole scan window, so consecutive pose snapshots
  // must be closer together than one scan period.
  if (!(motion_compensation.max_snapshot_gap_s <= frame_period_s)) {
    Fail("motion_compensation.max_snapshot_gap_s must be <= the sensor frame period");
  }

  // The buffer chain: nothing downstream may be sized larger than what feeds it.
  if (safety.max_obstacles > tracking.max_tracks) {
    Fail("safety.max_obstacles must be <= tracking.max_tracks");
  }
  if (dpcbf_adapter.max_obstacles > safety.max_obstacles) {
    Fail("dpcbf_adapter.max_obstacles must be <= safety.max_obstacles");
  }
  if (detection.max_circles > detection.max_primitives) {
    Fail("detection.max_circles must be <= detection.max_primitives");
  }

  // The safety stage may only inflate, never clip, a radius the detector can legitimately
  // produce; a ceiling below the detector's cap would silently shrink large obstacles.
  if (!(safety.max_radius_m >= detection.max_circle_radius_m)) {
    Fail("safety.max_radius_m must be >= detection.max_circle_radius_m");
  }

  // ============================================================================
  // THE SHORT-ARC BIAS BUDGET - a cross-STAGE safety constraint, added at P10.
  // ============================================================================
  // P8 measured the DE-ENLARGED fitted radius (`radius_fitted_m - radius_enlargement_m`,
  // upstream's `true_radius`) to be biased SMALL on short arcs by up to 0.161 m, and P9
  // measured the Kalman filter carrying that bias through to 0.169 m at the safety stage's
  // input. Under-estimating a radius is the one unrecoverable error in the subsystem.
  //
  // Only two configured terms cover that bias. `detection.radius_enlargement_m` adds a flat
  // margin to every fitted radius before it ever reaches the tracker, and
  // `safety.radius_inflation_fixed_m` adds one after. The stochastic term
  // `radius_inflation_k_sigma * sigma_r` does NOT: a systematic bias is invisible to the
  // filter's own covariance, and the P10 sweep measured containment falling to 93.9% - well
  // under the 99.9% target - on a corpus where the enlargement is absent and only k_sigma is
  // left to cover it.
  //
  // The shipped configuration satisfies this with 0.25 + 0.05 = 0.30 m against a 0.20 m
  // requirement. What the constraint exists to stop is somebody zeroing the ENLARGEMENT - a
  // detection-side knob whose safety consequence is entirely at the other end of the pipeline
  // - and silently losing containment. Turning it down is still allowed; turning it down
  // without raising the safety-side term is not.
  //
  // 0.20 m is P9's 0.169 m worst case rounded up to the nearest 0.05 m, and it is validated by
  // the P10 sweep rather than asserted: at the shipped k_sigma the enlargement-free corpus
  // reaches 99.9% containment once the two terms sum to 0.20 m.
  constexpr double kMeasuredShortArcRadiusBiasM = 0.20;
  if (!(detection.radius_enlargement_m + safety.radius_inflation_fixed_m >=
        kMeasuredShortArcRadiusBiasM)) {
    Fail("detection.radius_enlargement_m + safety.radius_inflation_fixed_m must be >= 0.20 m, "
         "the measured worst-case short-arc radius under-estimate (P8 finding 2, P9 through "
         "the filter); no other configured term covers a systematic bias");
  }

  // Confirming a track takes confirm_hits scans, so a min_track_age shorter than that
  // gate is dead configuration - it can never be the binding constraint.
  if (safety.min_track_hits < tracking.confirm_hits) {
    Fail("safety.min_track_hits must be >= tracking.confirm_hits "
         "(a track cannot reach safety before the tracker confirms it)");
  }

  // Coasting longer than the safety stage will tolerate produces tracks that exist only
  // to be discarded.
  if (!(tracking.max_coast_s >= frame_period_s)) {
    Fail("tracking.max_coast_s must be >= the sensor frame period");
  }

  // ROS2 publishes the frame, so it cannot claim to publish faster than frames arrive.
  if (ros2.publish_rate_hz > lidar.sensor.scan_rate_hz) {
    Fail("ros2.publish_rate_hz must be <= sensor.scan_rate_hz (it republishes frames)");
  }
}

PerceptionConfig PerceptionConfig::LoadFromYaml(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    Fail("configuration file was not found: " + path.string());
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path.string());
  } catch (const std::exception& error) {
    Fail("failed to parse " + path.string() + ": " + error.what());
  }

  const YAML::Node perception = root["perception"];
  if (!perception || !perception.IsMap()) {
    Fail(path.string() + " has no 'perception:' map");
  }
  RejectUnknownKeys(perception, "perception",
                    {"schema_version", "enabled", "seed", "lidar", "visualization", "frames",
                     "motion_compensation", "self_filter", "segmentation", "projection",
                     "detection", "tracking", "safety", "dpcbf_adapter", "mode", "ros2",
                     "dumps", "metrics", "profiling", "experimental"});

  PerceptionConfig config;
  if (perception["schema_version"]) {
    config.schema_version = perception["schema_version"].as<int>();
  }
  if (perception["enabled"]) config.enabled = perception["enabled"].as<bool>();

  // The master seed is read BEFORE the lidar section so it can supply the raycaster's
  // default; the two are reconciled after the lidar section has had its say.
  const bool master_seed_present = static_cast<bool>(perception["seed"]);
  if (master_seed_present) config.seed = perception["seed"].as<uint32_t>();

  const YAML::Node lidar = perception["lidar"];
  if (lidar) {
    RejectUnknownKeys(lidar, "perception.lidar",
                      {"extrinsic", "sensor", "aperture", "raycast", "self_filter"});
    if (lidar["extrinsic"]) LoadExtrinsic(lidar["extrinsic"], config.lidar.extrinsic);
    if (lidar["sensor"]) LoadSensor(lidar["sensor"], config.lidar.sensor);
    if (lidar["aperture"]) {
      RejectUnknownKeys(lidar["aperture"], "perception.lidar.aperture",
                        {"transparent_geom_group"});
      if (lidar["aperture"]["transparent_geom_group"]) {
        config.lidar.aperture.transparent_geom_group =
            lidar["aperture"]["transparent_geom_group"].as<int>();
      }
    }
    if (lidar["raycast"]) {
      RejectUnknownKeys(lidar["raycast"], "perception.lidar.raycast", {"exact"});
      if (lidar["raycast"]["exact"]) {
        config.lidar.raycast.exact = lidar["raycast"]["exact"].as<bool>();
      }
    }
    if (lidar["self_filter"]) {
      RejectUnknownKeys(lidar["self_filter"], "perception.lidar.self_filter",
                        {"robot_root_body", "reject_self_hits"});
      if (lidar["self_filter"]["robot_root_body"]) {
        config.lidar.self_filter.robot_root_body =
            lidar["self_filter"]["robot_root_body"].as<std::string>();
      }
      if (lidar["self_filter"]["reject_self_hits"]) {
        config.lidar.self_filter.reject_self_hits =
            lidar["self_filter"]["reject_self_hits"].as<bool>();
      }
    }
  }

  if (perception["visualization"]) {
    LoadVisualization(perception["visualization"], config.visualization);
  }

  // ONE seed, or a hard error. Two seeds that silently disagree would make a "same seed,
  // same dumps" determinism claim untrue in a way no test would notice.
  const bool sensor_seed_present =
      static_cast<bool>(lidar) && static_cast<bool>(lidar["sensor"]) &&
      static_cast<bool>(lidar["sensor"]["seed"]);
  if (master_seed_present && sensor_seed_present) {
    if (config.seed != config.lidar.sensor.seed) {
      Fail("perception.seed (" + std::to_string(config.seed) +
           ") and perception.lidar.sensor.seed (" + std::to_string(config.lidar.sensor.seed) +
           ") disagree; set one or make them equal");
    }
  } else if (master_seed_present) {
    config.lidar.sensor.seed = config.seed;
  } else if (sensor_seed_present) {
    config.seed = config.lidar.sensor.seed;
  }

  if (perception["frames"]) LoadFrames(perception["frames"], config.frames);
  if (perception["motion_compensation"]) {
    LoadMotionCompensation(perception["motion_compensation"], config.motion_compensation);
  }
  if (perception["self_filter"]) LoadCoreSelfFilter(perception["self_filter"], config.self_filter);
  if (perception["segmentation"]) LoadSegmentation(perception["segmentation"], config.segmentation);
  if (perception["projection"]) LoadProjection(perception["projection"], config.projection);
  if (perception["detection"]) LoadDetection(perception["detection"], config.detection);
  if (perception["tracking"]) LoadTracking(perception["tracking"], config.tracking);
  if (perception["safety"]) LoadSafety(perception["safety"], config.safety);
  if (perception["dpcbf_adapter"]) {
    LoadDpcbfAdapter(perception["dpcbf_adapter"], config.dpcbf_adapter);
  }
  if (perception["mode"]) LoadMode(perception["mode"], config.mode);
  if (perception["ros2"]) LoadRos2(perception["ros2"], config.ros2);
  if (perception["dumps"]) LoadDumps(perception["dumps"], config.dumps);
  if (perception["metrics"]) LoadMetrics(perception["metrics"], config.metrics);
  if (perception["profiling"]) LoadProfiling(perception["profiling"], config.profiling);
  if (perception["experimental"]) {
    LoadExperimental(perception["experimental"], config.experimental);
  }

  // DERIVED DEFAULT. Upstream's grouping threshold grows as
  // max_group_distance + range * distance_proportion, where distance_proportion is the
  // per-beam arc length at unit range. That is a property of the SCAN, so when the YAML
  // does not pin it, it follows the configured bin count rather than a stale literal.
  const bool distance_proportion_present =
      static_cast<bool>(perception["detection"]) &&
      static_cast<bool>(perception["detection"]["distance_proportion"]);
  if (!distance_proportion_present && config.projection.bins > 0) {
    constexpr double kTwoPi = 6.283185307179586;
    config.detection.distance_proportion = kTwoPi / config.projection.bins;
  }

  config.Validate();
  return config;
}

}  // namespace perception::integration
