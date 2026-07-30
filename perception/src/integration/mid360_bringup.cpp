#include "perception/integration/mid360_bringup.h"

#include <iomanip>
#include <iostream>
#include <sstream>

#include "perception/adapters/mujoco/extrinsic_guard.h"

namespace perception::integration {

void Mid360BringUp::LoadConfig(const std::filesystem::path& path) {
  config_ = PerceptionConfig::LoadFromYaml(path);
  config_source_ = path.string();
  std::cout << "[perception] loaded " << path.string() << '\n';
}

void Mid360BringUp::SetConfig(const PerceptionConfig& config, const std::string& source_label) {
  config.Validate();
  config_ = config;
  // Recording the source honestly matters here: an empty label makes the dump provenance say
  // "<built-in defaults>" rather than naming a YAML that was never read, while a caller that
  // loaded a file and then overrode it passes a label saying so. Either way the config_hash is
  // a real hash of the tree actually in force, so the hash - not this string - is what
  // identifies the run.
  config_source_ = source_label;
}

bool Mid360BringUp::VerifyExtrinsic(const mjModel* model, std::string& message) const {
  const auto check = adapters::mujoco::VerifyLidarExtrinsic(model, config_.lidar);
  message = check.message;
  return check.ok;
}

bool Mid360BringUp::BindModel(const mjModel* model, mjData* data, std::string& error) {
  have_scanned_ = false;
  next_scan_sim_time_s_ = 0.0;
  last_report_sim_time_s_ = 0.0;
  last_forced_scan_ = {};
  cloud_.Clear();
  stats_ = core::ScanStats{};

  if (!raycaster_.Bind(model, data, config_.lidar, error)) {
    return false;
  }

  if (config_.visualization.viewer_overlay) {
    std::string overlay_error;
    if (!overlay_.Allocate(model, config_.visualization, raycaster_.pattern().size(),
                           overlay_error)) {
      error = overlay_error;
      return false;
    }
    std::cout << "[perception] WARNING: visualization.viewer_overlay is ON, but MuJoCo 3.3.6's\n"
                 "[perception]   simulate app only appends user_scn geoms in PASSIVE mode and\n"
                 "[perception]   this application runs managed - the 3D overlay will NOT appear\n"
                 "[perception]   unless the vendored app is patched. See "
                 "perception/docs/architecture.md.\n";
  } else {
    overlay_.Free();
  }

#ifdef PERCEPTION_HAVE_OPENCV
  if (live_view_) {
    live_view_->Configure(config_.visualization, config_.lidar.sensor.azimuth_rays,
                          config_.lidar.sensor.elevation_rays, config_.lidar.sensor.max_range_m);
  }
#endif

  const auto& sensor = config_.lidar.sensor;
  std::cout << "[perception] Mid-360 bound: site '" << config_.lidar.extrinsic.site_name
            << "' on '" << config_.lidar.extrinsic.parent_body << "', "
            << raycaster_.pattern().size() << " rays/frame (" << sensor.azimuth_rays << " x "
            << sensor.elevation_rays << "), vertical FOV [" << sensor.vertical_fov_deg_min
            << ", " << sensor.vertical_fov_deg_max << "] deg in the SENSOR frame, "
            << sensor.scan_rate_hz << " Hz\n"
            << "[perception]   raycast=" << (raycaster_.raycast_exact() ? "exact" : "batched")
            << "  aperture_group=" << config_.lidar.aperture.transparent_geom_group
            << "  range=[" << sensor.min_range_m << ", " << sensor.max_range_m << "] m"
            << "  range_noise=" << (sensor.apply_range_noise ? "on" : "off") << '\n';
  return true;
}

bool Mid360BringUp::MaybeScan(double sim_time_s, bool force) {
  if (!raycaster_.bound() || !config_.enabled) {
    return false;
  }

  if (force) {
    // Sim time is frozen (paused viewer); pace off the wall clock so a full exact scan
    // does not run every millisecond.
    const auto now = std::chrono::steady_clock::now();
    const double min_interval_s = 1.0 / config_.visualization.refresh_hz;
    if (have_scanned_ &&
        std::chrono::duration<double>(now - last_forced_scan_).count() < min_interval_s) {
      return false;
    }
    last_forced_scan_ = now;
  } else {
    if (have_scanned_ && sim_time_s < next_scan_sim_time_s_) {
      return false;
    }
    next_scan_sim_time_s_ = sim_time_s + 1.0 / config_.lidar.sensor.scan_rate_hz;
  }

  if (!raycaster_.Scan(cloud_, stats_)) {
    return false;
  }
  have_scanned_ = true;

  if (config_.visualization.viewer_overlay) {
    overlay_.Update(cloud_, raycaster_.last_snapshot(), stats_);
  }
#ifdef PERCEPTION_HAVE_OPENCV
  if (live_view_) {
    // Copies only; the view's own thread does the drawing.
    live_view_->Publish(cloud_, raycaster_.last_snapshot(), stats_);
  }
#endif

  // Debug dumps. Runs on the physics thread inside the caller's lock scope, like everything
  // else here, so a dump write's cost lands in the physics-step budget - which is why it is
  // off by default and why the expensive part (the cloud) is separately gated.
  //
  // A write failure does NOT abort the scan or throw: the simulator must keep running when a
  // disk fills up. The first error is remembered and reported by FinishDumps.
  if (dumps_.enabled() && dump_error_.empty() && dumps_.ShouldRecordFrame()) {
    if (!dumps_.RecordBringUpScan(cloud_, stats_, raycaster_.last_snapshot(), dump_error_)) {
      std::cout << "[perception] dump write FAILED (dumping stops here): " << dump_error_
                << std::endl;
    }
  }

  // A census once per sim second, so the numbers are visible without a GUI. Flushed,
  // because the simulator is usually stopped with a signal and a buffered tail would be
  // lost exactly when you want to read it.
  if (sim_time_s - last_report_sim_time_s_ >= 1.0 || last_report_sim_time_s_ == 0.0) {
    last_report_sim_time_s_ = sim_time_s;
    std::cout << "[perception] " << CensusLine() << std::endl;
  }
  return true;
}

bool Mid360BringUp::StartDumps(const std::string& producer,
                               const std::filesystem::path& directory_override,
                               std::string& error) {
  dump_error_.clear();
  if (!dumps_.Configure(config_, config_source_, producer, directory_override, error)) {
    return false;
  }
  if (!dumps_.enabled()) return true;

  // What this slice retains. `raw_cloud` follows dumps.retain_stage_clouds; the other eight
  // stages do not exist yet, so their flags stay false and a replay reports them as NEVER
  // RECORDED rather than as stages that ran and produced nothing. That distinction is the
  // whole reason RetainedStages is in the dump header.
  core::RetainedStages retained;
  retained.raw_cloud = config_.dumps.retain_stage_clouds;
  dumps_.set_retained_stages(retained);

  std::cout << "[perception] dumps ON -> " << dumps_.directory().string() << "  (format="
            << config_.dumps.format << ", retain_stage_clouds="
            << (config_.dumps.retain_stage_clouds ? "true" : "false")
            << ", decimation=" << config_.dumps.decimation << ")\n";
  return true;
}

bool Mid360BringUp::FinishDumps(std::string& error) {
  if (!dumps_.enabled()) return true;
  if (!dump_error_.empty()) {
    error = dump_error_;
    return false;
  }
  if (!dumps_.Finish(error)) return false;
  std::cout << "[perception] dumps finished: " << dumps_.frames_recorded() << " of "
            << dumps_.frames_seen() << " scans recorded -> "
            << (dumps_.directory() / core::diagnostics::kRunManifestFileName).string() << '\n';
  return true;
}

void Mid360BringUp::StartLiveView() {
#ifdef PERCEPTION_HAVE_OPENCV
  if (!config_.enabled || !config_.visualization.enabled) {
    return;
  }
  if (!live_view_) {
    live_view_ = std::make_unique<adapters::opencv::LiveScanView>();
  }
  live_view_->Configure(config_.visualization, config_.lidar.sensor.azimuth_rays,
                        config_.lidar.sensor.elevation_rays, config_.lidar.sensor.max_range_m);
  live_view_->Start();
  std::cout << "[perception] live scan window started (OpenCV): "
               "\"Mid-360 live scan (perception)\"\n";
#else
  if (config_.visualization.enabled) {
    std::cout << "[perception] visualization.enabled is true but this build has no OpenCV; "
                 "the live window is unavailable\n";
  }
#endif
}

void Mid360BringUp::RequestLiveViewSnapshot(const std::string& path) {
#ifdef PERCEPTION_HAVE_OPENCV
  if (live_view_) {
    live_view_->RequestSnapshot(path);
  }
#else
  (void)path;
#endif
}

void Mid360BringUp::StopLiveView() {
#ifdef PERCEPTION_HAVE_OPENCV
  if (live_view_) {
    live_view_->Stop();
  }
#endif
}

std::string Mid360BringUp::CensusLine() const {
  const auto& snapshot = raycaster_.last_snapshot();
  const Eigen::Vector3d origin = snapshot.world_from_sensor.translation();

  std::ostringstream out;
  out << std::fixed << std::setprecision(4);
  out << "t=" << cloud_.stamp_s << "s origin_world=(" << origin.x() << ", " << origin.y()
      << ", " << origin.z() << ")";
  out << std::setprecision(2);
  out << " rays=" << stats_.rays_cast << " raw_hits=" << stats_.raw_hits
      << " no_hit=" << stats_.no_hit << " self=" << stats_.self_rejected
      << " range_rejected=" << stats_.range_rejected << " accepted=" << stats_.accepted
      << " self_frac=" << std::setprecision(4) << stats_.SelfHitFraction();
  if (stats_.aperture_diagnostic_valid) {
    out << " aperture_transmitted=" << stats_.aperture_transmitted;
  }
  out << std::setprecision(1) << " scan=" << stats_.scan_wall_time_us << "us";
  return out.str();
}

}  // namespace perception::integration
