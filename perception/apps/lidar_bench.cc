// lidar_bench - headless Mid-360 raycast harness.
//
// Loads a scene, binds the sensor, reports the extrinsic verdict and a per-scan census,
// and times the exact and batched paths against each other. Useful when you want the
// numbers without a GUI.
//
// Usage:
//   perception_lidar_bench <scene.xml> [perception.yaml] [--scans N] [--aperture-off]
//                                                        [--batched] [--quiet]

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include "perception/integration/mid360_bringup.h"

namespace {

void Usage() {
  std::printf(
      "usage: perception_lidar_bench <scene.xml> [perception.yaml]\n"
      "                             [--scans N] [--aperture-off] [--batched] [--quiet]\n"
      "                             [--view] [--step] [--snapshot PATH]\n"
      "  --view      open the live OpenCV scan window (needs a display)\n"
      "  --step      advance physics between scans. NOTE: the bench runs no controller,\n"
      "              so the G1 collapses under gravity - useful only to watch the view\n"
      "              update. Use the full simulator for a standing/walking robot.\n"
      "  --snapshot  write one rendered live-view canvas to PATH (implies --view)\n"
      "  --dump-dir  write debug dumps to DIR. Forces dumps.enabled on and overrides\n"
      "              dumps.directory; everything else (format, retain_stage_clouds,\n"
      "              decimation) still comes from the config, so this is a destination\n"
      "              override rather than a second set of dump settings.\n"
      "  --dump-clouds   also set dumps.retain_stage_clouds (the expensive one)\n"
      "  --dump-format F  override dumps.format: jsonl | binary | both\n");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    Usage();
    return 2;
  }

  const std::filesystem::path scene = argv[1];
  std::filesystem::path config_path;
  int scans = 5;
  bool aperture_off = false;
  bool batched = false;
  bool quiet = false;
  bool view = false;
  bool step = false;
  std::string snapshot_path;
  std::string dump_dir;
  bool dump_clouds = false;
  std::string dump_format;

  for (int i = 2; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--scans" && i + 1 < argc) {
      scans = std::atoi(argv[++i]);
    } else if (argument == "--aperture-off") {
      aperture_off = true;
    } else if (argument == "--batched") {
      batched = true;
    } else if (argument == "--quiet") {
      quiet = true;
    } else if (argument == "--view") {
      view = true;
    } else if (argument == "--step") {
      step = true;
    } else if (argument == "--snapshot" && i + 1 < argc) {
      snapshot_path = argv[++i];
      view = true;
    } else if (argument == "--dump-dir" && i + 1 < argc) {
      dump_dir = argv[++i];
    } else if (argument == "--dump-clouds") {
      dump_clouds = true;
    } else if (argument == "--dump-format" && i + 1 < argc) {
      dump_format = argv[++i];
    } else if (argument.rfind("--", 0) == 0) {
      std::printf("unknown option: %s\n", argument.c_str());
      Usage();
      return 2;
    } else {
      config_path = argument;
    }
  }

  perception::integration::Mid360BringUp bringup;
  try {
    if (!config_path.empty()) {
      bringup.LoadConfig(config_path);
    } else {
      std::printf("[bench] no config given; using built-in documented defaults\n");
    }
    // Apply the command-line overrides to the CONFIG, before binding, so the banner the
    // bind prints describes the mode actually used.
    if (aperture_off || batched || !dump_dir.empty() || dump_clouds || !dump_format.empty()) {
      perception::integration::PerceptionConfig config = bringup.config();
      if (aperture_off) {
        std::printf("[bench] aperture mask DISABLED (reproduces pre-aperture behaviour)\n");
        config.lidar.aperture.transparent_geom_group = -1;
      }
      if (batched) {
        std::printf("[bench] using the BATCHED mj_multiRay path (lossy; see raycaster_mj.h)\n");
        config.lidar.raycast.exact = false;
      }
      // --dump-dir is what turns dumping on from the command line.
      if (!dump_dir.empty()) config.dumps.enabled = true;
      if (dump_clouds) config.dumps.retain_stage_clouds = true;
      if (!dump_format.empty()) config.dumps.format = dump_format;
      // The provenance must not claim the YAML unqualified once we have changed values in it,
      // nor claim built-in defaults when a file was in fact the starting point. Say both.
      const std::string source_label =
          config_path.empty() ? std::string()
                              : config_path.string() + " (with command-line overrides)";
      bringup.SetConfig(config, source_label);
    }
  } catch (const std::exception& error) {
    std::printf("FATAL: %s\n", error.what());
    return 1;
  }

  char load_error[1024] = "";
  mjModel* model = mj_loadXML(scene.string().c_str(), nullptr, load_error, sizeof(load_error));
  if (model == nullptr) {
    std::printf("FATAL: could not load %s: %s\n", scene.string().c_str(), load_error);
    return 1;
  }
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  std::string message;
  const bool extrinsic_ok = bringup.VerifyExtrinsic(model, message);
  std::printf("%s\n", message.c_str());
  if (!extrinsic_ok) {
    mj_deleteData(data);
    mj_deleteModel(model);
    return 1;
  }

  std::string error;
  if (!bringup.BindModel(model, data, error)) {
    std::printf("FATAL: %s\n", error.c_str());
    mj_deleteData(data);
    mj_deleteModel(model);
    return 1;
  }

  auto& raycaster = bringup.raycaster();
  // Costs a second full pass, which is why scan_wall_time_us deliberately excludes it.
  raycaster.set_collect_aperture_diagnostic(true);

  {
    std::string dump_error;
    if (!bringup.StartDumps("perception_lidar_bench", dump_dir, dump_error)) {
      std::printf("FATAL: could not start dumps: %s\n", dump_error.c_str());
      mj_deleteData(data);
      mj_deleteModel(model);
      return 1;
    }
  }

  if (view) {
    bringup.StartLiveView();
  }

  const double frame_period_s = 1.0 / bringup.config().lidar.sensor.scan_rate_hz;
  double total_us = 0.0;
  for (int scan = 0; scan < scans; ++scan) {
    if (step) {
      // Real physics, so the view animates: the robot settles under gravity. Step until
      // the clock has actually passed the next frame boundary - stepping a fixed count
      // leaves d->time a hair short of it and the rate limiter then skips every other
      // scan.
      const double target_time_s = (scan + 1) * frame_period_s;
      while (data->time < target_time_s) {
        mj_step(model, data);
      }
    } else {
      // Static scene: just advance the clock so the rate limiter lets each scan through.
      data->time = scan * frame_period_s;
      mj_forward(model, data);
    }
    // Requested on the LAST scan, not the first: with --step the robot is still standing
    // for the first few frames, and the interesting state is the one at the end.
    if (!snapshot_path.empty() && scan == scans - 1) {
      bringup.RequestLiveViewSnapshot(snapshot_path);
    }
    if (!bringup.MaybeScan(data->time, /*force=*/false)) {
      std::printf("scan %d: no scan taken\n", scan);
      continue;
    }
    total_us += bringup.latest_stats().scan_wall_time_us;
    if (!quiet) {
      std::printf("scan %d: %s\n", scan, bringup.CensusLine().c_str());
    }
  }

  if (scans > 0) {
    std::printf("[bench] mean scan cost %.1f us over %d scans (%s path)\n", total_us / scans,
                scans, raycaster.raycast_exact() ? "exact" : "batched");
  }

  if (!snapshot_path.empty()) {
    // Give the render thread a moment to pick up the request before it is torn down.
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
  }
  bringup.StopLiveView();

  int status = 0;
  {
    std::string dump_error;
    if (!bringup.FinishDumps(dump_error)) {
      std::printf("ERROR: dumps did not finish cleanly: %s\n", dump_error.c_str());
      status = 1;
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
  return status;
}
