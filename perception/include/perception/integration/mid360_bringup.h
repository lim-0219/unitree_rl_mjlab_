// Mid360BringUp - the single facade the application constructs for the Phase-4 sensor
// bring-up slice.
//
// SCOPE, deliberately narrow. This is the ray-simulation and validation slice only:
// extrinsic guard, ray pattern, ray casting, per-scan census, live overlay. It is NOT the
// PerceptionRunner. There is no perception thread, no SPSC queue, no deskew, no frame
// accumulation, no projection, no detection and no tracking - those belong to later
// phases and building them now is explicitly listed as premature in
// perception/docs/architecture.md ("what must not be implemented too early").
//
// Everything here runs on the physics thread inside the simulator's existing `sim.mtx`
// critical section, because rays must be cast against a consistent mjData.
#ifndef PERCEPTION_INTEGRATION_MID360_BRINGUP_H_
#define PERCEPTION_INTEGRATION_MID360_BRINGUP_H_

#include <chrono>
#include <filesystem>
#include <string>

#include <mujoco/mujoco.h>

#include "perception/adapters/mujoco/live_scan_overlay.h"
#include "perception/adapters/mujoco/raycaster_mj.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/integration/dump_recorder.h"
#include "perception/integration/perception_config.h"

#ifdef PERCEPTION_HAVE_OPENCV
#include <memory>

#include "perception/adapters/opencv/live_scan_view.h"
#endif

namespace perception::integration {

class Mid360BringUp {
 public:
  Mid360BringUp() = default;

  // Throws std::runtime_error on a bad or missing file. Call once at startup.
  void LoadConfig(const std::filesystem::path& path);

  // Uses a programmatically-built config instead of a file. Used by tests and benches.
  //
  // `source_label` is what the dump provenance will record as `config_source`. Pass the
  // originating path plus a note when this is a file that was then overridden - a run whose
  // config no longer matches the YAML on disk must not claim the YAML, and must not claim
  // built-in defaults either. Empty means genuinely built-in defaults.
  void SetConfig(const PerceptionConfig& config, const std::string& source_label = "");

  const PerceptionConfig& config() const { return config_; }
  bool enabled() const { return config_.enabled; }

  // Cross-checks the compiled MJCF <site> against perception.lidar and writes a
  // human-readable verdict into `message`. Returns false on disagreement; the caller is
  // expected to terminate the process, which is what makes the check load-bearing.
  bool VerifyExtrinsic(const mjModel* model, std::string& message) const;

  // Binds to a freshly loaded (model, data) pair, mirroring
  // DynamicObstacleManager::BindModel. Rebuilds the ray table and the overlay scene.
  bool BindModel(const mjModel* model, mjData* data, std::string& error);

  // Casts a scan if one is due. `force` re-scans on a wall-clock cadence instead of a
  // sim-time one, for when the simulator is paused and sim time is frozen.
  // Returns true if a scan was taken.
  bool MaybeScan(double sim_time_s, bool force);

  // Valid after a successful MaybeScan.
  const core::TimedPointCloud& latest_cloud() const { return cloud_; }
  const core::ScanStats& latest_stats() const { return stats_; }
  const core::FrameTransformSnapshot& latest_snapshot() const {
    return raycaster_.last_snapshot();
  }

  // The MuJoCo-viewer overlay scene, or nullptr when it is disabled (the default - see
  // LidarVisualizationConfig::viewer_overlay for why).
  mjvScene* overlay_scene() { return overlay_.scene(); }

  // Starts/stops the OpenCV live window. No-ops when built without OpenCV or when
  // visualization.enabled is false.
  void StartLiveView();
  void StopLiveView();

  // Debugging aid: write the next rendered live-view canvas to `path`. No-op without
  // OpenCV or a running view.
  void RequestLiveViewSnapshot(const std::string& path);

  bool bound() const { return raycaster_.bound(); }
  adapters::mujoco::RaycasterMj& raycaster() { return raycaster_; }

  // One-line census, e.g. for a periodic status print.
  std::string CensusLine() const;

  // --- Debug dumps (the headless inspection / regression-fixture channel) ---------------
  //
  // Gated by the `dumps:` config section: with `dumps.enabled: false` (the shipped default)
  // StartDumps is a successful no-op and MaybeScan writes nothing. What gets dumped is what
  // this slice genuinely produces - ScanStats and FrameTransformSnapshot always, the raw
  // TimedPointCloud only when `dumps.retain_stage_clouds` is set.
  //
  // `directory_override` replaces `dumps.directory` when non-empty, for the bench's
  // --dump-dir. `producer` names the executable in the run manifest.
  bool StartDumps(const std::string& producer, const std::filesystem::path& directory_override,
                  std::string& error);

  // Closes the dump files and writes run_manifest.json + resolved_config.txt. Must be called
  // explicitly: a manifest written from a destructor would describe an aborted run as a
  // finished one.
  bool FinishDumps(std::string& error);

  bool dumping() const { return dumps_.enabled(); }
  const DumpRecorder& dumps() const { return dumps_; }

 private:
  PerceptionConfig config_;
  // Where the config came from, for the dump provenance. Empty means built-in defaults.
  std::string config_source_;
  DumpRecorder dumps_;
  // Set when a dump write fails, so the failure is reported once at FinishDumps rather than
  // spamming the physics thread's stdout every scan.
  std::string dump_error_;
  adapters::mujoco::RaycasterMj raycaster_;
  adapters::mujoco::LiveScanOverlay overlay_;
  core::TimedPointCloud cloud_;
  core::ScanStats stats_;

  double next_scan_sim_time_s_ = 0.0;
  bool have_scanned_ = false;
  std::chrono::steady_clock::time_point last_forced_scan_{};
  double last_report_sim_time_s_ = 0.0;

#ifdef PERCEPTION_HAVE_OPENCV
  std::unique_ptr<adapters::opencv::LiveScanView> live_view_;
#endif
};

}  // namespace perception::integration

#endif  // PERCEPTION_INTEGRATION_MID360_BRINGUP_H_
