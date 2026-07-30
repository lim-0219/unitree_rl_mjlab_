// Contract tests for the typed PerceptionConfig tree.
//
// What is asserted here:
//   * the SHIPPED perception/configs/perception.yaml loads cleanly and produces the
//     documented values (a config file nobody loads in a test is a config file that
//     silently rots);
//   * a default-constructed PerceptionConfig is already valid, so a programmatically-built
//     config is not a second, unvalidated schema;
//   * unknown keys are rejected at every level of the tree, not just the top;
//   * every cross-field constraint fails when violated, each with its own case;
//   * a full valid tree round-trips - write out an edited copy of the shipped file, load
//     it, and get back exactly what was written.
//
// argv[1] is the perception/ directory, so the test can find configs/perception.yaml.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "perception/integration/perception_config.h"

#include "check.h"

using perception::integration::PerceptionConfig;
using perception_test::Check;
using perception_test::Section;

namespace {

std::filesystem::path g_perception_dir;
std::filesystem::path g_scratch_dir;

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream stream(path);
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::filesystem::path WriteScratch(const std::string& name, const std::string& contents) {
  const std::filesystem::path path = g_scratch_dir / name;
  std::ofstream stream(path, std::ios::trunc);
  stream << contents;
  stream.close();
  return path;
}

// Loads a YAML string and reports whether it threw, plus the message. Every negative case
// checks the MESSAGE as well as the throw: a test that accepts any exception passes just
// as happily when an unrelated constraint is what actually fired.
struct LoadResult {
  bool threw = false;
  std::string message;
  PerceptionConfig config;
};

LoadResult TryLoad(const std::string& name, const std::string& contents) {
  LoadResult result;
  const std::filesystem::path path = WriteScratch(name, contents);
  try {
    result.config = PerceptionConfig::LoadFromYaml(path);
  } catch (const std::exception& error) {
    result.threw = true;
    result.message = error.what();
  }
  return result;
}

void CheckRejects(const std::string& name, const std::string& contents,
                  const std::string& expected_substring, const std::string& description) {
  const LoadResult result = TryLoad(name, contents);
  if (!result.threw) {
    Check(false, description, "expected a throw, config loaded cleanly");
    return;
  }
  const bool matches = result.message.find(expected_substring) != std::string::npos;
  Check(matches, description, matches ? "" : "got: " + result.message);
}

// The shipped file with one line replaced. Used for the negative cases so each one differs
// from the real config by exactly the field under test.
std::string ShippedWith(const std::string& find, const std::string& replace) {
  std::string text = ReadFile(g_perception_dir / "configs" / "perception.yaml");
  const std::size_t at = text.find(find);
  if (at == std::string::npos) {
    std::printf("  FATAL  the shipped config no longer contains '%s'\n", find.c_str());
    std::exit(2);
  }
  return text.substr(0, at) + replace + text.substr(at + find.size());
}

void TestShippedDefaultsLoad() {
  Section("the shipped perception.yaml");
  const std::filesystem::path path = g_perception_dir / "configs" / "perception.yaml";
  Check(std::filesystem::exists(path), "configs/perception.yaml exists");

  PerceptionConfig config;
  bool threw = false;
  std::string message;
  try {
    config = PerceptionConfig::LoadFromYaml(path);
  } catch (const std::exception& error) {
    threw = true;
    message = error.what();
  }
  Check(!threw, "the shipped config loads without error", threw ? message : "");
  if (threw) return;

  Check(config.schema_version == 1, "schema_version is 1");
  Check(config.seed == 42, "the master seed is 42");
  Check(config.lidar.sensor.seed == config.seed,
        "the raycaster seed follows the master seed");

  // The bring-up values must survive the schema extension unchanged - this is the
  // regression guard on the sections that already shipped.
  Check(config.lidar.extrinsic.parent_body == "torso_link", "the extrinsic parent is torso_link");
  Check(std::abs(config.lidar.extrinsic.translation_xyz_m.z() - 0.428434) < 1e-12,
        "the extrinsic translation is unchanged");
  Check(config.lidar.sensor.azimuth_rays == 360 && config.lidar.sensor.elevation_rays == 32,
        "the ray grid is unchanged");
  Check(config.lidar.raycast.exact, "exact raycasting is still the default");

  // The frozen sections.
  Check(config.frames.base_body == "pelvis", "frames.base_body is pelvis (not torso_link)");
  Check(config.frames.base_body != config.lidar.extrinsic.parent_body,
        "the DPCBF base body and the sensor's parent body are deliberately different");
  Check(config.motion_compensation.interpolation == "linear", "deskew interpolates linearly");
  Check(config.segmentation.backend == "zband", "the default segmenter is the z-band");
  Check(config.projection.bins == 360, "the scan has 360 bins");
  Check(config.detection.min_group_points == 5, "min_group_points is upstream's 5");
  Check(std::abs(config.detection.max_circle_radius_m - 0.60) < 1e-12,
        "max_circle_radius_m is upstream's 0.60");
  Check(std::abs(config.tracking.min_correspondence_cost_m - 0.30) < 1e-12,
        "min_correspondence_cost_m is upstream's 0.30");
  Check(std::abs(config.safety.min_radius_m - 0.20) < 1e-12,
        "the radius floor is dpcbf's radius_range minimum");
  Check(config.dpcbf_adapter.max_obstacles == 20, "the adapter caps at 20 obstacles");
  Check(config.mode.name == "oracle", "THE DEFAULT MODE IS ORACLE");
  Check(config.mode.scenario_allowlist.empty(), "no scenario is allow-listed by default");
  Check(!config.ros2.enabled, "the ROS2 adapter is off by default");
  Check(!config.dumps.enabled, "dumps are off by default");
  Check(!config.experimental.allow_experimental, "experimental options are off by default");

  // The derived default: the shipped file deliberately OMITS distance_proportion, so it
  // must come out matched to the bin count rather than at upstream's 1000-beam literal.
  const double expected = 2.0 * 3.14159265358979323846 / config.projection.bins;
  Check(std::abs(config.detection.distance_proportion - expected) < 1e-12,
        "distance_proportion is derived from projection.bins when the key is absent");
  Check(std::abs(config.detection.distance_proportion - 0.00628) > 1e-3,
        "the derived value is NOT upstream's 1000-beam literal");
}

void TestProgrammaticDefaults() {
  Section("programmatically-built defaults");
  PerceptionConfig config;
  bool threw = false;
  std::string message;
  try {
    config.Validate();
  } catch (const std::exception& error) {
    threw = true;
    message = error.what();
  }
  Check(!threw, "a default-constructed PerceptionConfig is already valid",
        threw ? message : "");

  // If the struct defaults and the YAML defaults ever diverge, there are two schemas.
  const PerceptionConfig loaded =
      PerceptionConfig::LoadFromYaml(g_perception_dir / "configs" / "perception.yaml");
  Check(config.projection.bins == loaded.projection.bins &&
            config.frames.base_body == loaded.frames.base_body &&
            config.mode.name == loaded.mode.name &&
            std::abs(config.safety.max_age_s - loaded.safety.max_age_s) < 1e-12,
        "the struct defaults agree with the shipped YAML");
}

void TestUnknownKeyRejection() {
  Section("unknown-key rejection");
  CheckRejects("unknown_top.yaml",
               ShippedWith("  schema_version: 1", "  schema_version: 1\n  typo_here: 3"),
               "unknown key 'typo_here'", "an unknown top-level key is rejected");

  CheckRejects("unknown_nested.yaml",
               ShippedWith("    bins: 360", "    bins: 360\n    binz: 720"),
               "unknown key 'binz'", "an unknown key inside projection is rejected");

  CheckRejects("unknown_deep.yaml",
               ShippedWith("      max_ground_height_m: -0.75",
                           "      max_ground_height_m: -0.75\n      max_ground_hight_m: -0.7"),
               "unknown key 'max_ground_hight_m'",
               "an unknown key two levels down is rejected");

  // The section that already shipped must still reject typos.
  CheckRejects("unknown_extrinsic.yaml",
               ShippedWith("      site_name: \"lidar_mid360\"",
                           "      site_name: \"lidar_mid360\"\n      site_nmae: \"x\""),
               "unknown key 'site_nmae'", "an unknown key in the extrinsic is still rejected");

  // The experimental subtree rejects too, but for its own reason.
  CheckRejects("unknown_experimental.yaml",
               ShippedWith("    allow_experimental: false",
                           "    allow_experimental: false\n    new_knob: 3"),
               "allow_experimental is false",
               "an experimental option without the flag is rejected");

  const LoadResult allowed =
      TryLoad("allowed_experimental.yaml",
              ShippedWith("    allow_experimental: false",
                          "    allow_experimental: true\n    new_knob: 3"));
  Check(!allowed.threw && allowed.config.experimental.options.count("new_knob") == 1,
        "an experimental option WITH the flag is accepted and recorded",
        allowed.threw ? allowed.message : "");
}

void TestEnumRejection() {
  Section("closed-set values");
  CheckRejects("bad_mode.yaml", ShippedWith("    name: \"oracle\"", "    name: \"yolo\""),
               "unsupported value 'yolo'", "an unknown mode name is rejected");
  CheckRejects("bad_backend.yaml",
               ShippedWith("    backend: \"zband\"", "    backend: \"ransac\""),
               "unsupported value 'ransac'", "an unknown segmentation backend is rejected");
  CheckRejects("bad_interp.yaml",
               ShippedWith("    interpolation: \"linear\"", "    interpolation: \"cubic\""),
               "unsupported value 'cubic'", "an unknown interpolation mode is rejected");
  CheckRejects("bad_format.yaml", ShippedWith("    format: \"jsonl\"", "    format: \"csv\""),
               "unsupported value 'csv'", "an unknown dump format is rejected");
}

void TestWithinSectionConstraints() {
  Section("within-section constraints");
  CheckRejects("bins_too_few.yaml", ShippedWith("    bins: 360", "    bins: 4"),
               "projection.bins must be >= 8", "bins < 8 is rejected");
  CheckRejects("bins_absurd.yaml", ShippedWith("    bins: 360", "    bins: 100000"),
               "projection.bins must be <= 36000", "an absurd bin count is rejected");
  CheckRejects("range_inverted.yaml",
               ShippedWith("    range_max_m: 10.0", "    range_max_m: 0.05"),
               "projection.range_min_m must be < projection.range_max_m",
               "range_min >= range_max is rejected");
  CheckRejects("group_points.yaml",
               ShippedWith("    min_group_points: 5", "    min_group_points: 1"),
               "detection.min_group_points must be >= 2",
               "a one-point group threshold is rejected");
  CheckRejects("confirm_hits.yaml", ShippedWith("    confirm_hits: 3", "    confirm_hits: 0"),
               "tracking.confirm_hits must be >= 1", "zero confirmation hits is rejected");
  CheckRejects("ring_capacity.yaml",
               ShippedWith("    snapshot_ring_capacity: 512", "    snapshot_ring_capacity: 1"),
               "snapshot_ring_capacity must be >= 2",
               "a ring too short to interpolate across is rejected");
  CheckRejects("self_filter_z.yaml", ShippedWith("    z_min_m: -1.00", "    z_min_m: 1.00"),
               "self_filter.z_min_m must be <", "an inverted self-filter band is rejected");
  CheckRejects("slope.yaml", ShippedWith("      max_slope_deg: 10.0", "      max_slope_deg: 95.0"),
               "max_slope_deg must lie in (0, 90)", "a slope limit past vertical is rejected");
  CheckRejects("decimation.yaml", ShippedWith("    decimation: 1", "    decimation: 0"),
               "dumps.decimation must be >= 1", "a zero decimation is rejected");
}

void TestCrossFieldConstraints() {
  Section("cross-section constraints");

  // safety.max_age >= sensor frame period. The example the configuration roadmap names.
  CheckRejects("stale_on_arrival.yaml",
               ShippedWith("    max_age_s: 0.30", "    max_age_s: 0.05"),
               "safety.max_age_s must be >= the sensor frame period",
               "a max_age shorter than one scan period is rejected");

  // mode.fallback_max_age >= safety.max_age.
  CheckRejects("early_fallback.yaml",
               ShippedWith("    fallback_max_age_s: 0.30", "    fallback_max_age_s: 0.15"),
               "mode.fallback_max_age_s must be >= safety.max_age_s",
               "falling back before safety declares staleness is rejected");

  // projection range window must lie inside the sensor's.
  CheckRejects("below_blind_zone.yaml",
               ShippedWith("    range_min_m: 0.10 ", "    range_min_m: 0.01 "),
               "projection.range_min_m must be >= sensor.min_range_m",
               "projecting inside the sensor blind zone is rejected");
  CheckRejects("beyond_cutoff.yaml",
               ShippedWith("    range_max_m: 10.0", "    range_max_m: 100.0"),
               "projection.range_max_m must be <= sensor.max_range_m",
               "projecting beyond the sensor cutoff is rejected");

  // The ground double gate.
  CheckRejects("band_below_ground.yaml",
               ShippedWith("    height_band_min_m: -0.70", "    height_band_min_m: -0.90"),
               "height_band_min_m must be >= segmentation.zband.max_ground_height_m",
               "a height band reaching below the ground threshold is rejected");

  // A self cylinder wider than the scan.
  CheckRejects("self_swallows_scan.yaml",
               ShippedWith("    radius_m: 0.45", "    radius_m: 20.0"),
               "self_filter.radius_m must be < projection.range_max_m",
               "a self cylinder wider than the scan is rejected");

  // Snapshot gap vs scan period.
  CheckRejects("sparse_snapshots.yaml",
               ShippedWith("    max_snapshot_gap_s: 0.02", "    max_snapshot_gap_s: 0.5"),
               "max_snapshot_gap_s must be <= the sensor frame period",
               "pose snapshots sparser than the scan period are rejected");

  // The buffer chain.
  CheckRejects("safety_over_tracks.yaml",
               ShippedWith("    max_obstacles: 32", "    max_obstacles: 128"),
               "safety.max_obstacles must be <= tracking.max_tracks",
               "more safety obstacles than tracks is rejected");
  CheckRejects("adapter_over_safety.yaml",
               ShippedWith("    max_obstacles: 20", "    max_obstacles: 64"),
               "dpcbf_adapter.max_obstacles must be <= safety.max_obstacles",
               "more adapter obstacles than safety obstacles is rejected");
  CheckRejects("primitives_under_clusters.yaml",
               ShippedWith("    max_primitives: 512", "    max_primitives: 8"),
               "detection.max_primitives must be >= detection.max_clusters",
               "fewer primitives than clusters is rejected");

  // The radius ceiling must cover what the detector can produce.
  CheckRejects("safety_clips_detector.yaml",
               ShippedWith("    max_radius_m: 1.00", "    max_radius_m: 0.30"),
               "safety.max_radius_m must be >= detection.max_circle_radius_m",
               "a safety ceiling below the detector's cap is rejected");

  // The enlargement-versus-cap trap: enlarge then compare, so enlargement >= cap rejects
  // every circle the detector will ever fit.
  CheckRejects("enlargement_swallows_cap.yaml",
               ShippedWith("    radius_enlargement_m: 0.25", "    radius_enlargement_m: 0.60"),
               "radius_enlargement_m must be < detection.max_circle_radius_m",
               "an enlargement at or above the circle cap is rejected");

  // THE SHORT-ARC BIAS BUDGET, added at P10. Only `detection.radius_enlargement_m` and
  // `safety.radius_inflation_fixed_m` cover a SYSTEMATIC radius under-estimate; the stochastic
  // `k_sigma * sigma_r` term provably does not (P10 measured containment falling to 93.9% on a
  // corpus with the enlargement absent). Zeroing the enlargement is a detection-side edit whose
  // only consequence is at the far end of the pipeline, so it is rejected rather than trusted.
  CheckRejects("short_arc_budget.yaml",
               ShippedWith("    radius_enlargement_m: 0.25", "    radius_enlargement_m: 0.0"),
               "must be >= 0.20 m, the measured worst-case short-arc radius under-estimate",
               "zeroing the detector enlargement without raising the safety fixed term is "
               "rejected");

  // Safety cannot gate on fewer hits than the tracker needs to confirm.
  CheckRejects("safety_before_confirm.yaml",
               ShippedWith("    min_track_hits: 3", "    min_track_hits: 1"),
               "safety.min_track_hits must be >= tracking.confirm_hits",
               "a safety hit gate below the confirmation gate is rejected");

  // Coasting shorter than a scan period.
  CheckRejects("no_coast.yaml", ShippedWith("    max_coast_s: 2.0", "    max_coast_s: 0.01"),
               "tracking.max_coast_s must be >= the sensor frame period",
               "a coast horizon shorter than a scan period is rejected");

  // ROS2 cannot republish faster than frames arrive.
  CheckRejects("ros2_too_fast.yaml",
               ShippedWith("    publish_rate_hz: 10.0", "    publish_rate_hz: 100.0"),
               "ros2.publish_rate_hz must be <= sensor.scan_rate_hz",
               "publishing faster than the scan rate is rejected");

  // The one-seed rule.
  CheckRejects("two_seeds.yaml", ShippedWith("      seed: 42", "      seed: 7"),
               "disagree", "two seeds that disagree are rejected");

  const LoadResult inherited =
      TryLoad("seed_inherited.yaml", ShippedWith("      seed: 42\n", ""));
  Check(!inherited.threw && inherited.config.lidar.sensor.seed == 42,
        "omitting the raycaster seed inherits the master seed",
        inherited.threw ? inherited.message : "");
}

void TestFullTreeRoundTrip() {
  Section("full-tree round trip");
  // Take the shipped tree, change one value in every frozen section to something that is
  // still valid, load it, and confirm every change survives. This is what catches a
  // section that was declared in the struct but never wired into the loader - the failure
  // mode where a field silently keeps its default no matter what the YAML says.
  std::string text = ReadFile(g_perception_dir / "configs" / "perception.yaml");
  struct Replacement {
    std::string find;
    std::string replace;
  };
  const Replacement replacements[] = {
      {"    world_frame_name: \"world\"", "    world_frame_name: \"odom\""},
      {"    max_extrapolation_s: 0.005", "    max_extrapolation_s: 0.004"},
      {"    radius_m: 0.45", "    radius_m: 0.40"},
      {"      band_tolerance_m: 0.08", "      band_tolerance_m: 0.06"},
      {"    height_band_max_m: 0.20", "    height_band_max_m: 0.30"},
      {"    max_split_distance_m: 0.20", "    max_split_distance_m: 0.18"},
      {"    delete_misses: 10", "    delete_misses: 12"},
      {"    latency_inflation_s: 0.15", "    latency_inflation_s: 0.12"},
      {"    id_offset: 0", "    id_offset: 1000"},
      {"    heartbeat_timeout_s: 0.50", "    heartbeat_timeout_s: 0.40"},
      {"    diagnostics_rate_hz: 1.0", "    diagnostics_rate_hz: 2.0"},
      {"    max_frames: 0", "    max_frames: 500"},
      {"    association_max_distance_m: 0.50", "    association_max_distance_m: 0.40"},
      {"    report_period_s: 10.0", "    report_period_s: 5.0"},
      {"    scenario_allowlist: []", "    scenario_allowlist: [\"crossing\", \"head_on\"]"},
  };
  for (const auto& replacement : replacements) {
    const std::size_t at = text.find(replacement.find);
    if (at == std::string::npos) {
      Check(false, "round-trip source line present", "missing: " + replacement.find);
      return;
    }
    text = text.substr(0, at) + replacement.replace +
           text.substr(at + replacement.find.size());
  }

  const LoadResult result = TryLoad("round_trip.yaml", text);
  if (result.threw) {
    Check(false, "the edited full tree loads", result.message);
    return;
  }
  const PerceptionConfig& config = result.config;
  Check(config.frames.world_frame_name == "odom", "frames round-trips");
  Check(std::abs(config.motion_compensation.max_extrapolation_s - 0.004) < 1e-12,
        "motion_compensation round-trips");
  Check(std::abs(config.self_filter.radius_m - 0.40) < 1e-12, "self_filter round-trips");
  Check(std::abs(config.segmentation.zband.band_tolerance_m - 0.06) < 1e-12,
        "segmentation round-trips");
  Check(std::abs(config.projection.height_band_max_m - 0.30) < 1e-12,
        "projection round-trips");
  Check(std::abs(config.detection.max_split_distance_m - 0.18) < 1e-12,
        "detection round-trips");
  Check(config.tracking.delete_misses == 12, "tracking round-trips");
  Check(std::abs(config.safety.latency_inflation_s - 0.12) < 1e-12, "safety round-trips");
  Check(config.dpcbf_adapter.id_offset == 1000, "dpcbf_adapter round-trips");
  Check(std::abs(config.mode.heartbeat_timeout_s - 0.40) < 1e-12, "mode round-trips");
  Check(config.mode.scenario_allowlist.size() == 2 &&
            config.mode.scenario_allowlist[0] == "crossing",
        "the scenario allowlist sequence round-trips");
  Check(std::abs(config.ros2.diagnostics_rate_hz - 2.0) < 1e-12, "ros2 round-trips");
  Check(config.dumps.max_frames == 500, "dumps round-trips");
  Check(std::abs(config.metrics.association_max_distance_m - 0.40) < 1e-12,
        "metrics round-trips");
  Check(std::abs(config.profiling.report_period_s - 5.0) < 1e-12, "profiling round-trips");
}

void TestMissingFileAndSchema() {
  Section("file-level failures");
  bool threw = false;
  std::string message;
  try {
    PerceptionConfig::LoadFromYaml(g_scratch_dir / "does_not_exist.yaml");
  } catch (const std::exception& error) {
    threw = true;
    message = error.what();
  }
  Check(threw && message.find("was not found") != std::string::npos,
        "a missing config file throws", threw ? "" : "no throw");

  CheckRejects("bad_schema.yaml", ShippedWith("  schema_version: 1", "  schema_version: 2"),
               "unsupported schema_version 2", "a future schema version is rejected");

  CheckRejects("no_root.yaml", "something_else:\n  a: 1\n", "has no 'perception:' map",
               "a file without the perception root is rejected");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: %s <perception-dir>\n", argv[0]);
    return 2;
  }
  g_perception_dir = argv[1];
  g_scratch_dir = std::filesystem::temp_directory_path() / "perception_config_test";
  std::filesystem::create_directories(g_scratch_dir);

  std::printf("perception config tests (%s)\n", g_perception_dir.string().c_str());

  TestShippedDefaultsLoad();
  TestProgrammaticDefaults();
  TestUnknownKeyRejection();
  TestEnumRejection();
  TestWithinSectionConstraints();
  TestCrossFieldConstraints();
  TestFullTreeRoundTrip();
  TestMissingFileAndSchema();

  std::error_code ignored;
  std::filesystem::remove_all(g_scratch_dir, ignored);
  return perception_test::Report("perception_config_test");
}
