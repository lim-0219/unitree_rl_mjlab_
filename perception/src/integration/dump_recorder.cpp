#include "perception/integration/dump_recorder.h"

#include <cstdio>
#include <ctime>
#include <fstream>

#include "perception/core/diagnostics/config_hash.h"
#include "perception/generated/perception_build_info.h"

namespace perception::integration {
namespace {

using core::diagnostics::DumpFormat;
using core::diagnostics::RecordType;

// One key per line, in a fixed order. Doubles at 17 significant digits so the text is a
// lossless rendering of the resolved tree rather than a rounded view of it - otherwise two
// configs that differ below the printed precision would hash identically.
class ConfigTextBuilder {
 public:
  void Add(const char* key, const std::string& value) { text_ += std::string(key) + "=" + value + "\n"; }
  void Add(const char* key, bool value) { Add(key, value ? std::string("true") : std::string("false")); }
  void Add(const char* key, int value) { Add(key, std::to_string(value)); }
  void Add(const char* key, uint32_t value) { Add(key, std::to_string(value)); }

  void Add(const char* key, double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
    Add(key, std::string(buffer));
  }

  void Add(const char* key, const Eigen::Vector3d& value) { AddIndexed(key, value.data(), 3); }
  void Add(const char* key, const Eigen::Vector4d& value) { AddIndexed(key, value.data(), 4); }

  const std::string& text() const { return text_; }

 private:
  void AddIndexed(const char* key, const double* data, int count) {
    for (int index = 0; index < count; ++index) {
      Add((std::string(key) + "[" + std::to_string(index) + "]").c_str(), data[index]);
    }
  }

  std::string text_;
};

std::string IsoUtcNow() {
  const std::time_t now = std::time(nullptr);
  std::tm utc{};
#if defined(_WIN32)
  gmtime_s(&utc, &now);
#else
  gmtime_r(&now, &utc);
#endif
  char buffer[32];
  if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &utc) == 0) {
    return "unknown";
  }
  return buffer;
}

}  // namespace

std::string ResolvedConfigText(const PerceptionConfig& config) {
  ConfigTextBuilder out;

  // NOTE on scope: this covers the ENTIRE resolved tree, including the dumps/metrics/
  // profiling sections. Turning dumps on therefore changes the config hash. That is
  // deliberate - the hash identifies a run's configuration, not a subset of it that someone
  // judged to be behaviour-affecting, and such a judgement would itself need versioning.
  out.Add("schema_version", config.schema_version);
  out.Add("enabled", config.enabled);
  out.Add("seed", config.seed);

  out.Add("lidar.extrinsic.parent_body", config.lidar.extrinsic.parent_body);
  out.Add("lidar.extrinsic.site_name", config.lidar.extrinsic.site_name);
  out.Add("lidar.extrinsic.translation_xyz_m", config.lidar.extrinsic.translation_xyz_m);
  out.Add("lidar.extrinsic.rotation_rpy_rad", config.lidar.extrinsic.rotation_rpy_rad);

  out.Add("lidar.sensor.vertical_fov_deg_min", config.lidar.sensor.vertical_fov_deg_min);
  out.Add("lidar.sensor.vertical_fov_deg_max", config.lidar.sensor.vertical_fov_deg_max);
  out.Add("lidar.sensor.horizontal_fov_deg", config.lidar.sensor.horizontal_fov_deg);
  out.Add("lidar.sensor.azimuth_rays", config.lidar.sensor.azimuth_rays);
  out.Add("lidar.sensor.elevation_rays", config.lidar.sensor.elevation_rays);
  out.Add("lidar.sensor.min_range_m", config.lidar.sensor.min_range_m);
  out.Add("lidar.sensor.scan_rate_hz", config.lidar.sensor.scan_rate_hz);
  out.Add("lidar.sensor.max_range_m", config.lidar.sensor.max_range_m);
  out.Add("lidar.sensor.range_noise_std_m", config.lidar.sensor.range_noise_std_m);
  out.Add("lidar.sensor.apply_range_noise", config.lidar.sensor.apply_range_noise);
  out.Add("lidar.sensor.seed", config.lidar.sensor.seed);
  out.Add("lidar.sensor.angular_noise_deg_1sigma", config.lidar.sensor.angular_noise_deg_1sigma);
  out.Add("lidar.sensor.body_size_mm", config.lidar.sensor.body_size_mm);

  out.Add("lidar.aperture.transparent_geom_group", config.lidar.aperture.transparent_geom_group);
  out.Add("lidar.raycast.exact", config.lidar.raycast.exact);
  out.Add("lidar.self_filter.robot_root_body", config.lidar.self_filter.robot_root_body);
  out.Add("lidar.self_filter.reject_self_hits", config.lidar.self_filter.reject_self_hits);

  out.Add("visualization.enabled", config.visualization.enabled);
  out.Add("visualization.viewer_overlay", config.visualization.viewer_overlay);
  out.Add("visualization.draw_points", config.visualization.draw_points);
  out.Add("visualization.draw_rays", config.visualization.draw_rays);
  out.Add("visualization.draw_sensor_frame", config.visualization.draw_sensor_frame);
  out.Add("visualization.point_stride", config.visualization.point_stride);
  out.Add("visualization.ray_stride", config.visualization.ray_stride);
  out.Add("visualization.point_size_m", config.visualization.point_size_m);
  out.Add("visualization.ray_width_px", config.visualization.ray_width_px);
  out.Add("visualization.refresh_hz", config.visualization.refresh_hz);
  out.Add("visualization.point_rgba", config.visualization.point_rgba);
  out.Add("visualization.ray_rgba", config.visualization.ray_rgba);

  out.Add("frames.base_body", config.frames.base_body);
  out.Add("frames.world_frame_name", config.frames.world_frame_name);
  out.Add("frames.snapshot_ring_capacity", config.frames.snapshot_ring_capacity);

  out.Add("motion_compensation.deskew_enabled", config.motion_compensation.deskew_enabled);
  out.Add("motion_compensation.gravity_align_enabled",
          config.motion_compensation.gravity_align_enabled);
  out.Add("motion_compensation.interpolation", config.motion_compensation.interpolation);
  out.Add("motion_compensation.max_snapshot_gap_s", config.motion_compensation.max_snapshot_gap_s);
  out.Add("motion_compensation.max_extrapolation_s",
          config.motion_compensation.max_extrapolation_s);
  out.Add("motion_compensation.require_full_window",
          config.motion_compensation.require_full_window);

  out.Add("self_filter.enabled", config.self_filter.enabled);
  out.Add("self_filter.radius_m", config.self_filter.radius_m);
  out.Add("self_filter.z_min_m", config.self_filter.z_min_m);
  out.Add("self_filter.z_max_m", config.self_filter.z_max_m);

  out.Add("segmentation.backend", config.segmentation.backend);
  out.Add("segmentation.zband.max_ground_height_m", config.segmentation.zband.max_ground_height_m);
  out.Add("segmentation.zband.band_tolerance_m", config.segmentation.zband.band_tolerance_m);
  out.Add("segmentation.zband.refine_with_plane", config.segmentation.zband.refine_with_plane);
  out.Add("segmentation.zband.plane_ransac_distance_m",
          config.segmentation.zband.plane_ransac_distance_m);
  out.Add("segmentation.zband.plane_ransac_iterations",
          config.segmentation.zband.plane_ransac_iterations);
  out.Add("segmentation.zband.max_slope_deg", config.segmentation.zband.max_slope_deg);

  out.Add("projection.bins", config.projection.bins);
  out.Add("projection.range_min_m", config.projection.range_min_m);
  out.Add("projection.range_max_m", config.projection.range_max_m);
  out.Add("projection.height_band_min_m", config.projection.height_band_min_m);
  out.Add("projection.height_band_max_m", config.projection.height_band_max_m);
  out.Add("projection.collect_bin_histogram", config.projection.collect_bin_histogram);

  out.Add("detection.min_group_points", config.detection.min_group_points);
  out.Add("detection.max_group_distance_m", config.detection.max_group_distance_m);
  out.Add("detection.distance_proportion", config.detection.distance_proportion);
  out.Add("detection.max_split_distance_m", config.detection.max_split_distance_m);
  out.Add("detection.max_merge_separation_m", config.detection.max_merge_separation_m);
  out.Add("detection.max_merge_spread_m", config.detection.max_merge_spread_m);
  out.Add("detection.max_circle_radius_m", config.detection.max_circle_radius_m);
  out.Add("detection.radius_enlargement_m", config.detection.radius_enlargement_m);
  out.Add("detection.circles_from_visibles", config.detection.circles_from_visibles);
  out.Add("detection.use_split_and_merge", config.detection.use_split_and_merge);
  out.Add("detection.max_clusters", config.detection.max_clusters);
  out.Add("detection.max_primitives", config.detection.max_primitives);
  out.Add("detection.max_circles", config.detection.max_circles);

  out.Add("tracking.process_variance", config.tracking.process_variance);
  out.Add("tracking.process_rate_variance", config.tracking.process_rate_variance);
  out.Add("tracking.measurement_variance", config.tracking.measurement_variance);
  out.Add("tracking.min_correspondence_cost_m", config.tracking.min_correspondence_cost_m);
  out.Add("tracking.association_radius_weight", config.tracking.association_radius_weight);
  out.Add("tracking.measurement_sigma_scale", config.tracking.measurement_sigma_scale);
  out.Add("tracking.measurement_sigma_floor_m", config.tracking.measurement_sigma_floor_m);
  out.Add("tracking.initial_rate_variance", config.tracking.initial_rate_variance);
  out.Add("tracking.confirm_hits", config.tracking.confirm_hits);
  out.Add("tracking.delete_misses", config.tracking.delete_misses);
  out.Add("tracking.max_coast_s", config.tracking.max_coast_s);
  out.Add("tracking.enable_fusion", config.tracking.enable_fusion);
  out.Add("tracking.enable_fission", config.tracking.enable_fission);
  out.Add("tracking.max_tracks", config.tracking.max_tracks);

  out.Add("safety.max_age_s", config.safety.max_age_s);
  out.Add("safety.min_track_age_s", config.safety.min_track_age_s);
  out.Add("safety.min_track_hits", config.safety.min_track_hits);
  out.Add("safety.radius_inflation_k_sigma", config.safety.radius_inflation_k_sigma);
  out.Add("safety.radius_inflation_fixed_m", config.safety.radius_inflation_fixed_m);
  out.Add("safety.latency_inflation_s", config.safety.latency_inflation_s);
  out.Add("safety.use_enclosing_radius", config.safety.use_enclosing_radius);
  out.Add("safety.min_radius_m", config.safety.min_radius_m);
  out.Add("safety.max_radius_m", config.safety.max_radius_m);
  out.Add("safety.max_speed_mps", config.safety.max_speed_mps);
  out.Add("safety.max_obstacles", config.safety.max_obstacles);

  out.Add("dpcbf_adapter.preserve_track_ids", config.dpcbf_adapter.preserve_track_ids);
  out.Add("dpcbf_adapter.id_offset", config.dpcbf_adapter.id_offset);
  out.Add("dpcbf_adapter.max_obstacles", config.dpcbf_adapter.max_obstacles);
  out.Add("dpcbf_adapter.drop_invalid", config.dpcbf_adapter.drop_invalid);

  out.Add("mode.name", config.mode.name);
  out.Add("mode.fallback_max_age_s", config.mode.fallback_max_age_s);
  out.Add("mode.fallback_on_invalid_frame", config.mode.fallback_on_invalid_frame);
  out.Add("mode.heartbeat_timeout_s", config.mode.heartbeat_timeout_s);
  out.Add("mode.scenario_allowlist.count",
          static_cast<int>(config.mode.scenario_allowlist.size()));
  for (std::size_t index = 0; index < config.mode.scenario_allowlist.size(); ++index) {
    out.Add(("mode.scenario_allowlist[" + std::to_string(index) + "]").c_str(),
            config.mode.scenario_allowlist[index]);
  }

  out.Add("ros2.enabled", config.ros2.enabled);
  out.Add("ros2.topic_namespace", config.ros2.topic_namespace);
  out.Add("ros2.publish_rate_hz", config.ros2.publish_rate_hz);
  out.Add("ros2.diagnostics_rate_hz", config.ros2.diagnostics_rate_hz);
  out.Add("ros2.publish_clouds", config.ros2.publish_clouds);
  out.Add("ros2.publish_scan", config.ros2.publish_scan);
  out.Add("ros2.publish_markers", config.ros2.publish_markers);
  out.Add("ros2.publish_tf", config.ros2.publish_tf);
  out.Add("ros2.use_sim_time", config.ros2.use_sim_time);
  out.Add("ros2.debug_level", config.ros2.debug_level);

  out.Add("dumps.enabled", config.dumps.enabled);
  out.Add("dumps.directory", config.dumps.directory);
  out.Add("dumps.format", config.dumps.format);
  out.Add("dumps.retain_stage_clouds", config.dumps.retain_stage_clouds);
  out.Add("dumps.max_frames", config.dumps.max_frames);
  out.Add("dumps.decimation", config.dumps.decimation);

  out.Add("metrics.enabled", config.metrics.enabled);
  out.Add("metrics.manifest_path", config.metrics.manifest_path);
  out.Add("metrics.compare_against_oracle", config.metrics.compare_against_oracle);
  out.Add("metrics.association_max_distance_m", config.metrics.association_max_distance_m);

  out.Add("profiling.enabled", config.profiling.enabled);
  out.Add("profiling.per_stage_timing", config.profiling.per_stage_timing);
  out.Add("profiling.allocation_counters", config.profiling.allocation_counters);
  out.Add("profiling.report_period_s", config.profiling.report_period_s);

  out.Add("experimental.allow_experimental", config.experimental.allow_experimental);
  // std::map iterates in key order, so this is deterministic without an explicit sort.
  out.Add("experimental.options.count", static_cast<int>(config.experimental.options.size()));
  for (const auto& [key, value] : config.experimental.options) {
    out.Add(("experimental.options." + key).c_str(), value);
  }

  return out.text();
}

core::diagnostics::RunProvenance MakeRunProvenance(const PerceptionConfig& config,
                                                   const std::string& config_source,
                                                   const std::string& producer) {
  core::diagnostics::RunProvenance provenance;
  provenance.dump_schema_version = core::diagnostics::kDumpSchemaVersion;
  // The OTHER version. Both are recorded because they drift independently.
  provenance.config_schema_version = config.schema_version;
  provenance.config_hash = core::diagnostics::HashConfigText(ResolvedConfigText(config));
  provenance.config_source = config_source.empty() ? "<built-in defaults>" : config_source;
  provenance.git_sha = PERCEPTION_GIT_SHA;
  provenance.git_describe = PERCEPTION_GIT_DESCRIBE;
  provenance.git_dirty = PERCEPTION_GIT_DIRTY;
  provenance.created_utc = IsoUtcNow();
  provenance.seed = config.seed;
  provenance.producer = producer.empty() ? "unknown" : producer;
  return provenance;
}

DumpRecorder::~DumpRecorder() {
  // Deliberately no Finish() here. A dump directory with no manifest is how an interrupted
  // run announces itself; writing one from a destructor would erase that signal.
}

bool DumpRecorder::Configure(const PerceptionConfig& config, const std::string& config_source,
                             const std::string& producer,
                             const std::filesystem::path& directory_override,
                             std::string& error) {
  enabled_ = config.dumps.enabled;
  dumps_format_ = config.dumps.format;
  retain_stage_clouds_ = config.dumps.retain_stage_clouds;
  decimation_ = config.dumps.decimation;
  max_frames_ = config.dumps.max_frames;
  frames_seen_ = 0;
  frames_recorded_ = 0;
  finished_ = false;
  sinks_.clear();

  // The loader already validates these, but Configure is also reachable from a
  // programmatically-built config (tests, benches) that never went through the loader.
  if (decimation_ < 1) {
    error = "dumps.decimation must be >= 1";
    return false;
  }
  if (max_frames_ < 0) {
    error = "dumps.max_frames must be >= 0";
    return false;
  }

  want_binary_ = dumps_format_ == "binary" || dumps_format_ == "both";
  want_jsonl_ = dumps_format_ == "jsonl" || dumps_format_ == "both";
  if (enabled_ && !want_binary_ && !want_jsonl_) {
    error = "dumps.format must be 'jsonl', 'binary' or 'both' (got '" + dumps_format_ + "')";
    return false;
  }

  directory_ = directory_override.empty() ? std::filesystem::path(config.dumps.directory)
                                          : directory_override;
  if (enabled_ && directory_.empty()) {
    error = "dumps.directory must not be empty when dumps.enabled is true";
    return false;
  }

  resolved_config_text_ = ResolvedConfigText(config);
  provenance_ = MakeRunProvenance(config, config_source, producer);

  // Default retention for whatever the producer does not override: the bring-up slice's raw
  // cloud is governed by retain_stage_clouds; every later stage is absent because it does not
  // exist yet, which is precisely "never recorded" rather than "produced nothing".
  retained_ = core::RetainedStages{};
  retained_.raw_cloud = retain_stage_clouds_;
  retained_mask_ = core::diagnostics::PackRetainedStages(retained_);
  return true;
}

void DumpRecorder::set_retained_stages(const core::RetainedStages& stages) {
  retained_ = stages;
  retained_mask_ = core::diagnostics::PackRetainedStages(stages);
}

bool DumpRecorder::ShouldRecordFrame() {
  if (!enabled_) return false;
  const uint64_t index = frames_seen_;
  ++frames_seen_;
  if (max_frames_ > 0 && frames_recorded_ >= static_cast<uint64_t>(max_frames_)) return false;
  if (index % static_cast<uint64_t>(decimation_) != 0) return false;
  ++frames_recorded_;
  return true;
}

DumpRecorder::Sink* DumpRecorder::SinkFor(RecordType type, std::string& error) {
  for (const std::unique_ptr<Sink>& sink : sinks_) {
    if (sink->type == type) return sink.get();
  }
  if (finished_) {
    error = "DumpRecorder: cannot open a new dump file after Finish()";
    return nullptr;
  }

  auto sink = std::make_unique<Sink>();
  sink->type = type;
  const std::string stem = core::diagnostics::ToString(type);

  if (want_binary_) {
    sink->binary = std::make_unique<core::diagnostics::DumpWriter>();
    const std::filesystem::path path =
        directory_ / (stem + core::diagnostics::DumpFormatExtension(DumpFormat::kBinary));
    if (!sink->binary->Open(path, DumpFormat::kBinary, type, provenance_, retained_mask_,
                            error)) {
      return nullptr;
    }
  }
  if (want_jsonl_) {
    sink->jsonl = std::make_unique<core::diagnostics::DumpWriter>();
    const std::filesystem::path path =
        directory_ / (stem + core::diagnostics::DumpFormatExtension(DumpFormat::kJsonl));
    if (!sink->jsonl->Open(path, DumpFormat::kJsonl, type, provenance_, retained_mask_,
                           error)) {
      return nullptr;
    }
  }

  sinks_.push_back(std::move(sink));
  return sinks_.back().get();
}

bool DumpRecorder::RecordBringUpScan(const core::TimedPointCloud& cloud,
                                     const core::ScanStats& stats,
                                     const core::FrameTransformSnapshot& snapshot,
                                     std::string& error) {
  if (!enabled_) return true;

  // The census and the pose are cheap and always written - they are the diagnostics-shaped
  // records the headless channel exists for. The cloud is the expensive one and is gated.
  const uint64_t sequence = cloud.sequence;
  if (!Emit(stats, sequence, cloud.stamp_s, error)) return false;
  if (!Emit(snapshot, sequence, snapshot.stamp_s, error)) return false;
  if (retain_stage_clouds_) {
    if (!Emit(cloud, sequence, cloud.stamp_s, error)) return false;
  }
  return true;
}

bool DumpRecorder::Finish(std::string& error) {
  if (!enabled_ || finished_) return true;

  core::diagnostics::RunManifest manifest;
  manifest.provenance = provenance_;
  manifest.retained_stages_mask = retained_mask_;
  manifest.dumps_format = dumps_format_;
  manifest.retain_stage_clouds = retain_stage_clouds_;
  manifest.decimation = decimation_;
  manifest.max_frames = max_frames_;
  manifest.frames_seen = frames_seen_;
  manifest.frames_recorded = frames_recorded_;
  manifest.resolved_config_path = core::diagnostics::kResolvedConfigFileName;

  for (const std::unique_ptr<Sink>& sink : sinks_) {
    core::diagnostics::DumpWriter* writers[2] = {sink->binary.get(), sink->jsonl.get()};
    for (core::diagnostics::DumpWriter* writer : writers) {
      if (writer == nullptr) continue;
      if (!writer->Close(error)) return false;
      core::diagnostics::DumpFileEntry entry;
      entry.record_type = writer->record_type();
      entry.path = writer->path().filename().string();
      entry.format = writer->format();
      entry.record_count = writer->footer().record_count;
      entry.sequence_first = writer->footer().sequence_first;
      entry.sequence_last = writer->footer().sequence_last;
      entry.stamp_first_s = writer->footer().stamp_first_s;
      entry.stamp_last_s = writer->footer().stamp_last_s;
      manifest.files.push_back(entry);
    }
  }

  std::error_code filesystem_error;
  std::filesystem::create_directories(directory_, filesystem_error);
  if (filesystem_error) {
    error = "could not create the dump directory: " + filesystem_error.message();
    return false;
  }

  const std::filesystem::path config_path =
      directory_ / core::diagnostics::kResolvedConfigFileName;
  std::ofstream config_stream(config_path, std::ios::binary | std::ios::trunc);
  if (!config_stream) {
    error = "could not write " + config_path.string();
    return false;
  }
  config_stream << resolved_config_text_;
  config_stream.flush();
  if (!config_stream) {
    error = "failed writing " + config_path.string();
    return false;
  }

  if (!manifest.Write(directory_ / core::diagnostics::kRunManifestFileName, error)) {
    return false;
  }
  finished_ = true;
  return true;
}

}  // namespace perception::integration
