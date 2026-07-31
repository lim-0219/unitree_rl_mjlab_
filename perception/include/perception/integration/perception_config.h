// Typed, immutable-after-load configuration for the perception subsystem.
//
// yaml-cpp appears in exactly one translation unit (src/integration/perception_config.cpp);
// core/ and adapters/ receive plain structs. That is what keeps core dependency-free and
// satisfies the "no configuration access from arbitrary modules" rule in
// perception/docs/architecture.md.
//
// Every default below is the documented value for the Unitree G1 / Livox Mid-360 pair.
// Provenance for each number is given in the comment next to it; do not change one
// without a new citation.
#ifndef PERCEPTION_INTEGRATION_PERCEPTION_CONFIG_H_
#define PERCEPTION_INTEGRATION_PERCEPTION_CONFIG_H_

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace perception::integration {

// Where the sensor is bolted. Transcribed verbatim from Unitree's `mid360_joint` fixed
// joint (parent `torso_link`) in the 16 up-to-date g1_description URDFs.
// Source: g1_mid360_extrinsic.md sections 5.1 / 6.
struct LidarExtrinsicConfig {
  std::string parent_body = "torso_link";
  std::string site_name = "lidar_mid360";

  Eigen::Vector3d translation_xyz_m{0.0002835, 0.00003, 0.428434};

  // Fixed-axis (extrinsic) XYZ, as in URDF. roll = pi is the UPSIDE-DOWN mount and is
  // deliberate; pitch = 0.05112069379091391 rad = 2.929 deg nose-down.
  Eigen::Vector3d rotation_rpy_rad{3.141592653589793, 0.05112069379091391, 0.0};
};

// Livox Mid-360 published specification. Source: Livox Mid-360 User Manual v1.0 p.20,
// via g1_mid360_extrinsic.md section 3.2.
struct LidarSensorModelConfig {
  // SENSOR-FRAME vertical FOV, as published. Do NOT negate these: the roll = pi in the
  // extrinsic already performs the flip, so negating here would double-invert and point
  // the sensor at the sky. In the robot frame this becomes +4.071 deg ... -54.929 deg.
  // Source: g1_mid360_extrinsic.md section 6.3.
  double vertical_fov_deg_min = -7.0;
  double vertical_fov_deg_max = 52.0;
  double horizontal_fov_deg = 360.0;

  // Uniform-grid APPROXIMATION of the real non-repetitive rosette: 360 x 32 = 11520
  // rays/frame against ~20000 points/frame on hardware (200000 pts/s at 10 Hz).
  int azimuth_rays = 360;
  int elevation_rays = 32;

  double min_range_m = 0.10;   // Close-proximity blind zone, manual p.20.
  double scan_rate_hz = 10.0;  // Frame rate, manual p.20.

  // NOT from either source document: an implementation-side far cutoff. The manual
  // quotes detection range by target reflectivity rather than a single max range, so
  // this is a tunable, flagged as an open parameter rather than a citation.
  double max_range_m = 40.0;

  // Distance random error, 1 sigma at 10 m, manual p.20. Implemented and seeded, but
  // OFF by default: the P4 acceptance gate is "hit-distance error <= 1e-6 m vs
  // analytic", which requires a noise-free sensor. Turn on for fidelity studies.
  double range_noise_std_m = 0.02;
  bool apply_range_noise = false;
  uint32_t seed = 42;

  // Angular random error, 1 sigma, manual p.20. Recorded here for completeness and
  // deliberately NOT modelled - see the open items in the final report.
  double angular_noise_deg_1sigma = 0.15;

  // Livox body size in mm, for packaging/collision sanity checks only.
  Eigen::Vector3d body_size_mm{65.0, 65.0, 60.0};
};

// The optical aperture, modelled as a MuJoCo geom-visibility group that LiDAR rays
// ignore. `head_link.STL` is a closed opaque envelope with no modelled window and the
// sensor origin sits inside it, so without this every ray self-terminates on the head.
// Source: g1_mid360_sensor_model.md sections 3 / 4.1.
struct LidarApertureConfig {
  // Geoms in this group are invisible to LiDAR rays only. -1 disables the mask entirely
  // and reproduces the pre-aperture (100% self-hit) behaviour, which is what the V0/V1
  // before-and-after evidence uses.
  int transparent_geom_group = 2;
};

struct LidarRaycastConfig {
  // Cast each ray with mj_ray rather than batching through mj_multiRay. This is a
  // correctness setting, not a style choice: mj_multiRay prunes candidate geoms with an
  // angular bound derived from the 8 AABB corners, which is not a valid outer bound for
  // a geom subtending a large solid angle, so it silently drops hits on large flat
  // surfaces (measured up to 23.7% of wall rays). The error is always a MISSED
  // occlusion, never a wrong range. Source: g1_mid360_sensor_model.md section 4.2.
  // `false` is offered only as a performance fallback and is never the default.
  bool exact = true;
};

// Defensive post-raycast self rejection. The aperture mask is the primary mechanism;
// this catches everything else on the robot (arms, torso, legs), which must keep
// occluding - see the V4 asymmetry check.
struct LidarSelfFilterConfig {
  std::string robot_root_body = "pelvis";  // Kinematic root; the G1 URDF has no base_link.
  bool reject_self_hits = true;
};

struct LidarConfig {
  LidarExtrinsicConfig extrinsic;
  LidarSensorModelConfig sensor;
  LidarApertureConfig aperture;
  LidarRaycastConfig raycast;
  LidarSelfFilterConfig self_filter;
};

// Live visualization.
struct LidarVisualizationConfig {
  // The OpenCV live window (top-down + range image). This is the working live view; see
  // adapters/opencv/live_scan_view.h.
  bool enabled = true;

  // The MuJoCo-viewer 3D overlay (adapters/mujoco/live_scan_overlay.h). OFF by default and
  // NOT because it is untested: MuJoCo 3.3.6's `simulate` app only appends `user_scn` geoms
  // into the render scene in PASSIVE mode, and this application runs managed
  // (`is_passive_ = false`), so the overlay would be a silent no-op. Enabling it without
  // patching the vendored app buys nothing but the geom-building cost. See
  // perception/docs/architecture.md for the exact patch if you want it.
  bool viewer_overlay = false;

  // Overlay-only knobs. Deliberately decimated: drawing all 11520 returns as individual
  // geoms every frame is not worth the frame rate.
  bool draw_points = true;
  bool draw_rays = true;
  bool draw_sensor_frame = true;
  int point_stride = 3;    // Draw every Nth accepted return.
  int ray_stride = 128;    // Draw every Nth ray line.
  double point_size_m = 0.015;
  double ray_width_px = 1.0;  // mjGEOM_LINE width is denominated in pixels, not metres.
  double refresh_hz = 10.0;
  Eigen::Vector4d point_rgba{1.0, 0.35, 0.0, 0.95};
  Eigen::Vector4d ray_rgba{0.15, 0.85, 1.0, 0.18};
};

// =========================================================================================
// Sections below this line were frozen in the contract/config phase. Their consuming
// modules mostly do not exist yet; the point of declaring them now is that a later phase
// reads an already-typed, already-validated field instead of amending the schema, and that
// the cross-section constraints between them are checked in ONE place from the start.
//
// Where a default is upstream obstacle_detector's, it is marked UPSTREAM and its value is
// transcribed from the audited source, not guessed. Where a default is this repository's
// own judgement it says so and names the evidence that should replace it.
// =========================================================================================

// Which body the rest of the system calls "the robot". NOTE the deliberate difference from
// LidarExtrinsicConfig::parent_body: the Mid-360 hangs off `torso_link` (behind three waist
// joints), while DPCBF and the ground-truth reader use `pelvis`. They are different bodies
// and both values are correct; composing the extrinsic against `pelvis` would be
// pose-dependent and wrong.
struct FramesConfig {
  std::string base_body = "pelvis";  // Matches dpcbf/config/dpcbf_config.yaml `robot.base_body`.
  std::string world_frame_name = "world";

  // FrameTransformSnapshot ring capacity, in entries. The physics step is 0.002 s in this
  // repository, so 512 entries span ~1.02 s - ten scan periods of pose history, which is
  // what lets a late scan still be deskewed.
  int snapshot_ring_capacity = 512;
};

// Deskew and gravity alignment (the P5 stages).
struct MotionCompensationConfig {
  bool deskew_enabled = true;
  bool gravity_align_enabled = true;

  // "nearest" | "linear". Linear interpolation between the two bracketing snapshots is the
  // design; nearest exists as the ablation baseline that quantifies what deskew buys.
  std::string interpolation = "linear";

  // Largest gap between consecutive pose snapshots that deskew will interpolate across.
  // 0.02 s is ten physics steps; beyond that the constant-velocity assumption between
  // snapshots stops being a small correction.
  double max_snapshot_gap_s = 0.02;

  // How far past the newest snapshot a point may be extrapolated. Deliberately about one
  // physics step: this is what absorbs the known one-step scan/kinematics lag (~4.7 mm at
  // fall speed) rather than letting it extrapolate freely.
  double max_extrapolation_s = 0.005;

  // Fail the frame outright when the snapshot ring does not span the whole scan window,
  // instead of silently deskewing part of it. Fault-injection tests depend on this being
  // an explicit choice.
  bool require_full_window = true;
};

// Defensive self filter - the CORE pipeline stage. Distinct from LidarSelfFilterConfig,
// which rejects returns by MuJoCo body ancestry at the raycaster. This one is geometric,
// runs on the cloud, and is the belt to that adapter's braces: it is the only self
// rejection that survives a swap to real hardware, where no body ids exist.
//
// Coordinates are in the GRAVITY-ALIGNED BASE frame, whose origin is `torso_link`. The
// floor therefore sits near z = -0.837 m, not z = 0.
struct CoreSelfFilterConfig {
  bool enabled = true;

  // Cylinder radius about the base origin. The G1's shoulder span sets this; 0.45 m clears
  // the arms in a neutral stance without eating the 0.2 m obstacles at their closest
  // approach.
  double radius_m = 0.45;
  double z_min_m = -1.00;  // Just below the feet.
  double z_max_m = 0.50;   // Just above the head crown (crown is +0.487 m from torso).
};

// Ground segmentation. `travel` is accepted by the schema but requires the
// PERCEPTION_WITH_TRAVEL build option; the schema carries it now so the later phase does
// not have to touch the loader.
struct ZBandSegmenterConfig {
  // Gravity-aligned base z at or below which a point is ground. The floor is at
  // -0.837 m under the neutral standing pose, so -0.75 m leaves ~9 cm of margin for
  // torso pitch and for the deskew residual.
  double max_ground_height_m = -0.75;

  // Half-width of the transition band around the threshold, within which a point is
  // labelled ground only if the optional plane refinement agrees.
  double band_tolerance_m = 0.08;

  bool refine_with_plane = false;  // Single-plane RANSAC on the candidate ground points.
  double plane_ransac_distance_m = 0.05;
  int plane_ransac_iterations = 128;

  // Reject a refined plane tilted more than this from the gravity-aligned horizontal.
  // The arena is flat; a steeply tilted "ground" plane means the fit found a wall.
  double max_slope_deg = 10.0;
};

struct SegmentationConfig {
  std::string backend = "zband";  // "zband" | "travel".
  ZBandSegmenterConfig zband;
};

// Scan projection (the ported pointcloud_to_laserscan core).
struct ProjectionConfig {
  // 360 bins = 1 degree. The bin-count sweep is a planned ablation (risk R4).
  int bins = 360;

  double range_min_m = 0.10;  // Must be >= sensor.min_range_m; see the cross-field checks.

  // 10 m rather than the sensor's 40 m cutoff: DPCBF only considers obstacles within
  // `robot.p_max = 3.0 m` (dpcbf_config.yaml), so 10 m is already a 3x margin and keeping
  // the scan short bounds both the detector's work and its false-positive surface.
  double range_max_m = 10.0;

  // The slab of the gravity-aligned base frame collapsed into the 2-D scan. The lower edge
  // sits ABOVE segmentation.max_ground_height_m on purpose - the height band and the
  // ground label are a deliberate double gate against ground contamination (risk R6).
  double height_band_min_m = -0.70;
  double height_band_max_m = 0.20;

  // Fills ProjectedScan::bin_point_counts. Cheap, and it is the only source for the
  // information-loss metric the projection acceptance gate asks for.
  bool collect_bin_histogram = true;
};

// Detection. Parameter NAMES are upstream obstacle_extractor's so the two can be tuned
// with the same numbers (architecture doc section 14).
struct DetectionConfig {
  int min_group_points = 5;           // UPSTREAM default.
  double max_group_distance_m = 0.10; // UPSTREAM default.

  // UPSTREAM default is 0.00628, which is the per-beam arc length at unit range for a
  // ~1000-beam scan. Ours is a 360-bin scan, so the matched value is 2*pi/bins = 0.01745,
  // which is what this default is. When the YAML omits the key the loader RE-DERIVES it
  // from the configured projection.bins, so changing the bin count alone keeps the
  // grouping threshold matched. Setting the key explicitly overrides both.
  // Copying upstream's literal 0.00628 onto a 360-bin scan would silently make the
  // grouping threshold 2.8x too tight.
  double distance_proportion = 0.017453292519943295;  // 2*pi/360.

  double max_split_distance_m = 0.20;    // UPSTREAM default.
  double max_merge_separation_m = 0.20;  // UPSTREAM default.
  double max_merge_spread_m = 0.20;      // UPSTREAM default.
  double max_circle_radius_m = 0.60;     // UPSTREAM default.

  // MEASURED, replacing upstream's 0.25 m default that P1 kept "pending the P8 short-arc bias
  // experiment (Q11)". Sized to cover the sqrt(3)/3 rule's systematic short-arc radius
  // UNDER-estimate at the stage where it is made: P8 measured that at 0.16127 m worst case over
  // 18 matched cylinders, and P10's corpus C measured 0.1668 m surviving the Kalman filter to
  // the safety stage's input. 0.17 m is the smallest 0.01 m-granular value covering both.
  // Full derivation, including why the flat budget stops at 0.25 m total rather than the 0.20 m
  // cross-field floor, is in configs/perception.yaml next to the shipped value.
  double radius_enlargement_m = 0.17;

  bool circles_from_visibles = true;  // UPSTREAM default.
  bool use_split_and_merge = true;    // UPSTREAM default.

  // UPSTREAM default (true). A segment that produced an accepted circle is dropped from the
  // segment list, so the two detector outputs partition the scan rather than describing the
  // same points twice. Not in the section-14 key list, which names the thresholds; it is a
  // ported extractor parameter all the same and is exposed here under upstream's own name so
  // the port and its regression oracle stay tunable with one set of numbers.
  bool discard_converted_segments = true;

  // Preallocation maxima. Splitting can produce more primitives than clusters, which is
  // why max_primitives is the larger of the three.
  int max_clusters = 256;
  int max_primitives = 512;
  int max_circles = 256;
};

// Tracking. The KF variances are upstream's; the lifecycle counters are NOT - upstream has
// no confirmation gate, only a timer-driven fade counter, and replacing that with a
// measurement-driven lifecycle is the deliberate design change recorded as risk R11.
struct TrackingConfig {
  // RETUNED, not upstream's. Per-second noise densities (m^2/s, (m/s)^2/s) rather than
  // upstream's per-100-Hz-tick variances, and re-sized because this subsystem's R is 2-3 orders
  // of magnitude smaller than upstream's flat 1.00 m^2 - only the Q/R ratio reaches the Kalman
  // gain, so carrying upstream's Q across would invert its behaviour rather than preserve it.
  // Set by an NIS sweep; see configs/perception.yaml for the numbers and
  // core/tracking/kf_circle_tracker.h change (a) for the dt scaling.
  double process_variance = 0.0001;
  double process_rate_variance = 0.03;

  // UPSTREAM default, with a narrowed role: the FALLBACK measurement variance for an
  // observation that reports a zero sigma. The detector's per-observation sigmas are the
  // primary source. See kf_circle_tracker.h, change (e).
  double measurement_variance = 1.00;

  // UPSTREAM default (`min_correspondence_cost`): the association gate, a Euclidean
  // distance in (x, y, r) space, in metres.
  double min_correspondence_cost_m = 0.30;

  // NEW, and the numbers the tracking phase is answerable for. Every one of them is documented
  // at its point of use in core/tracking/kf_circle_tracker.h - the rationale lives with the
  // algorithm, not with the parser.

  // Weight on the radius term of the association cost, in (0, 1]. 1.0 is upstream's unweighted
  // cost exactly. 0.25 is the response to the measured short-arc radius bias.
  double association_radius_weight = 0.25;

  // Calibration of the detector's PROVISIONAL measurement sigmas: scale, then floor in metres.
  double measurement_sigma_scale = 1.0;
  double measurement_sigma_floor_m = 0.005;

  // Prior variance on a newborn track's rate state, (m/s)^2. 0.8 m/s is the arena's top
  // obstacle speed (dpcbf_config.yaml `speed_range`), so 0.64 covers it and nothing more.
  double initial_rate_variance = 0.64;

  // NEW (measurement-driven lifecycle). Three hits at 10 Hz is the 0.3 s confirmation
  // delay the tracking acceptance gate allows.
  int confirm_hits = 3;
  int delete_misses = 10;

  // UPSTREAM's `tracking_duration` (2.0 s), reinterpreted as a coasting horizon rather
  // than a timer-tick fade.
  double max_coast_s = 2.0;

  bool enable_fusion = true;   // N tracks -> 1 measurement, covariance-weighted merge.
  bool enable_fission = true;  // 1 track -> N measurements.

  int max_tracks = 64;
};

// Safety-state generation: validity gating and conservative inflation.
struct SafetyConfig {
  // Oldest a frame may be and still be consumed. Must be >= one sensor frame period, or
  // every frame would be stale on arrival - checked as a cross-field constraint.
  double max_age_s = 0.30;

  // A track must have existed this long AND accumulated this many hits before it is
  // allowed to reach DPCBF.
  double min_track_age_s = 0.20;
  int min_track_hits = 3;

  // The implemented rule, and the two terms the architecture doc's one-line formula does not
  // have, are documented at length in core/safety/safety_state_generator.h and beside these
  // keys in the shipped configs/perception.yaml. In brief:
  //   radius = max(radius_true_m, min_radius_m) + k_sigma*(sigma_r + sigma_pos) + fixed
  //            + (age_s + latency_inflation_s)*|v|
  // k_sigma was RETAINED at 2.0 after a P10 sweep rather than read off a chi-square quantile;
  // the chi-square model is rejected on this data because the errors are bias-dominated.
  double radius_inflation_k_sigma = 2.0;

  // The only configured term besides detection.radius_enlargement_m that covers a SYSTEMATIC
  // radius under-estimate. Their sum is held to a measured floor by a cross-field constraint.
  // Raised 0.05 -> 0.08 when the enlargement was re-derived to 0.17 m: the enlargement is sized
  // by the bias it covers and this term carries whatever the flat TOTAL has to be, which the
  // safety suite's strict-form containment gate puts at 0.2395 m. The two are interchangeable
  // downstream, so only the total is a performance number.
  double radius_inflation_fixed_m = 0.08;

  // One scan period plus processing. This IS the doc's `latency * k_lat`; the coefficient is
  // folded in, there is no separate k_lat.
  double latency_inflation_s = 0.15;

  // A DOCUMENTED NO-OP: `radius_enclosing_m` does not survive the tracker, and P10 measured it
  // SMALLER than the fitted radius on 20 of 20 corpus circles, so plumbing it through would
  // carry a term that never binds. Retained because the schema is frozen; asserted inert.
  bool use_enclosing_radius = true;

  // Radius floor, applied to the BASE radius before inflation. 0.20 m is the minimum of
  // dpcbf_config.yaml's `radius_range`, i.e. the smallest object that exists in this arena.
  //
  // Q13, RESOLVED, and not the way the open question guessed: this is a DEGENERATE-FIT GUARD,
  // not the answer to the short-arc radius bias. P10 measured it binding on 0 of 171 emissions.
  double min_radius_m = 0.20;

  // Ceiling, to stop a degenerate fit inflating into a wall. Must be >= the detector's
  // max_circle_radius_m or the two stages would disagree about what is representable.
  double max_radius_m = 1.00;

  // Velocity spike clamp, m/s. Oracle obstacles top out at 0.8 m/s
  // (dpcbf_config.yaml `speed_range`); 1.6 m/s is 2x that, so a clamp only ever fires on
  // an estimator fault, never on a real obstacle.
  double max_speed_mps = 1.60;

  int max_obstacles = 32;
};

// SafetyObstacle -> dpcbf::ObstacleState. The adapter itself may include dpcbf headers;
// this config struct may not, so the DPCBF-side limits appear here as plain numbers with
// their source named.
struct DpcbfAdapterConfig {
  // Track ids pass through unchanged. The filter selects top-k constraints and the
  // visualizer colours by constraint, so a stable id is what keeps that selection from
  // flickering.
  bool preserve_track_ids = true;
  int id_offset = 0;  // Added to track ids, to keep oracle and estimated id spaces apart.

  // Upper bound on states handed to Filter(). dpcbf_config.yaml uses
  // `default_num_constraints: 10`, so 20 leaves the filter's own top-k selection something
  // to select from without unbounded QP growth.
  int max_obstacles = 20;

  bool drop_invalid = true;  // Omit invalid/stale obstacles rather than passing them on.
};

// The obstacle-source mode ladder (architecture doc section 16).
struct ModeConfig {
  // "oracle" | "shadow" | "compare" | "estimated_fallback" | "estimated".
  // Default is oracle: the estimated path is never live by default, at any phase.
  std::string name = "oracle";

  // Per-frame fallback trigger. Must be >= safety.max_age_s, otherwise the selector would
  // fall back before the safety stage ever got to declare a frame stale and the safety
  // gate would be dead code.
  double fallback_max_age_s = 0.30;
  bool fallback_on_invalid_frame = true;

  // Missed perception-thread heartbeat that also triggers fallback.
  double heartbeat_timeout_s = 0.50;

  // Scenario names in which the estimated path may drive the live filter. Empty means
  // "none", which is the safe default for every mode below `estimated`.
  std::vector<std::string> scenario_allowlist;
};

// Optional ROS2 visualization adapter. Runtime enablement here is necessary but NOT
// sufficient - PERCEPTION_WITH_ROS2 must also be ON at build time. The loader cannot see
// the build flag, so a true value with the adapter absent is a no-op, not an error.
struct Ros2Config {
  bool enabled = false;
  std::string topic_namespace = "/perception";
  double publish_rate_hz = 10.0;       // Scan rate.
  double diagnostics_rate_hz = 1.0;
  bool publish_clouds = true;
  bool publish_scan = true;
  bool publish_markers = true;
  bool publish_tf = true;
  bool use_sim_time = true;  // Stamp with MuJoCo sim time rather than wall time.
  int debug_level = 0;       // 0 = off; higher publishes progressively more intermediates.
};

// Debug dumps - the headless substitute for a visualization, and the regression-fixture
// format for every later phase.
struct DumpsConfig {
  bool enabled = false;
  std::string directory = "perception_dumps";
  std::string format = "jsonl";  // "jsonl" | "binary" | "both".

  // Whether PerceptionFrame keeps its intermediate clouds. Off by default because it is
  // the expensive one: ~11520 points per stage per frame.
  bool retain_stage_clouds = false;

  int max_frames = 0;  // 0 = unlimited.
  int decimation = 1;  // Write every Nth frame; 1 = every frame.
};

struct MetricsConfig {
  bool enabled = true;
  std::string manifest_path = "perception_metrics.json";

  // Oracle-to-estimate comparison. Independent of `mode`: comparison metrics can be
  // collected in shadow mode without the estimated path driving anything.
  bool compare_against_oracle = false;

  // Nearest-centre association gate for that comparison, metres. Beyond this an oracle
  // obstacle counts as missed rather than badly estimated.
  double association_max_distance_m = 0.50;
};

struct ProfilingConfig {
  bool enabled = false;
  bool per_stage_timing = true;
  bool allocation_counters = false;  // Test-only counting allocator; never in production.
  double report_period_s = 10.0;
};

// The experimental escape hatch. Any key inside `experimental:` other than the flag itself
// is REJECTED unless allow_experimental is true, at which point it is captured verbatim
// as a string so the run manifest still records it. This keeps a deterministic evaluation
// run from silently containing an unreviewed knob.
struct ExperimentalConfig {
  bool allow_experimental = false;
  std::map<std::string, std::string> options;
};

struct PerceptionConfig {
  int schema_version = 1;
  bool enabled = true;

  // Master run seed. `lidar.sensor.seed` is the raycaster's own copy: it defaults to this
  // value, and the loader hard-fails if both keys are present and disagree, so there is
  // never a second source of truth for reproducibility.
  uint32_t seed = 42;

  LidarConfig lidar;
  LidarVisualizationConfig visualization;

  FramesConfig frames;
  MotionCompensationConfig motion_compensation;
  CoreSelfFilterConfig self_filter;
  SegmentationConfig segmentation;
  ProjectionConfig projection;
  DetectionConfig detection;
  TrackingConfig tracking;
  SafetyConfig safety;
  DpcbfAdapterConfig dpcbf_adapter;
  ModeConfig mode;
  Ros2Config ros2;
  DumpsConfig dumps;
  MetricsConfig metrics;
  ProfilingConfig profiling;
  ExperimentalConfig experimental;

  // Derived, not configured: the sensor frame period in seconds. Several cross-field
  // constraints are stated against it, so it gets one definition.
  double SensorFramePeriodSeconds() const { return 1.0 / lidar.sensor.scan_rate_hz; }

  // Throws std::runtime_error on a missing file, an out-of-range value, a failed
  // cross-field constraint, or an unknown key. Unknown-key rejection is deliberate:
  // a silently-ignored typo in an extrinsic is exactly the failure this subsystem
  // cannot afford.
  static PerceptionConfig LoadFromYaml(const std::filesystem::path& path);

  // Same checks LoadFromYaml applies; separate so programmatically-built configs
  // (tests, benches) get validated too.
  void Validate() const;
};

}  // namespace perception::integration

#endif  // PERCEPTION_INTEGRATION_PERCEPTION_CONFIG_H_
