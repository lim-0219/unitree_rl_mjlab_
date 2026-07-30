# DPCBF Perception Subsystem — Repository-Grounded Architecture & Implementation Roadmap

**Repository:** `unitree_rl_mjlab_` (branch `dpcbf`)
**Robot:** Unitree G1
**Simulator:** MuJoCo (vendored fork of `simulate` app)
**Date:** 2026-07-30

**Label conventions:** `[FACT]` confirmed from source · `[INFERENCE]` derived · `[RECOMMENDATION]` architectural choice · `[OPEN]` unresolved.

---

## 1. Executive summary

The repository already contains a complete, working DPCBF safety-filter stack whose obstacle input is a ground-truth oracle: kinematic mocap cylinders managed by `dpcbf::DynamicObstacleManager` and consumed by `dpcbf::DpcbfSafetyFilter::Filter()` at a single call site inside the simulator's SDK-bridge thread (`simulate/src/main.cc:677-709`) `[FACT]`. There is no lidar, raycast, point-cloud, or ROS code anywhere in the production tree — `mj_ray` exists only in the vendored MuJoCo headers and is never called `[FACT]`.

The DPCBF obstacle interface is a six-field POD, `ObstacleState {x, y, radius, velocity_x, velocity_y, id}` in world frame (`dpcbf/include/dpcbf/dpcbf_safety_filter.h:19-26`) `[FACT]`, which means the perception subsystem's final contract is already frozen by the repository and needs zero edits to `dpcbf/`.

**The plan:** build a new, self-contained `perception/` subsystem — an Eigen/STL-only core with a MuJoCo adapter (Mid-360 ray simulation + oracle extraction), a repository-owned scan projector (ported from `pointcloud_to_laserscan`'s ~60-line math core), a repository-owned detection/tracking backend (ported from `obstacle_detector`'s ROS-free algorithmic core, re-based from Armadillo onto Eigen), a safety-state generator, and a thin DPCBF adapter. ROS2 is an optional, build-flag-gated visualization adapter only. The two reference packages stay in-tree as reference implementations and regression oracles, never as production dependencies:

- `obstacle_detector` is ROS1/catkin (package.xml format 2, catkin buildtool) `[FACT]` while `pointcloud_to_laserscan` is ROS2/ament `[FACT]` — they cannot even run together as external nodes without a `ros1_bridge`, which kills Option C outright.
- Both packages' algorithmic cores are cleanly separable from their ROS plumbing `[FACT — detailed in §4/§5]`, making PORT cheap and safe.

Integration into the existing application proceeds through staged modes (oracle-only → shadow → comparison → fallback-guarded closed loop), with the oracle path preserved permanently. Total foreseen edits to existing files: **two files** (`simulate/CMakeLists.txt`, `simulate/src/main.cc`), everything else is new files under `perception/`. *(Amended in §1.1 below to three files — see the P0/P4 bring-up status.)*

---

## 1.1 Amendment — Phase 0 / P4 Mid-360 bring-up status (2026-07-30)

**P0: COMPLETE.** `perception/` skeleton, five CMake options, guarded `add_subdirectory`, `docs/architecture.md`, and ON/OFF build verification all landed exactly as specified in §9/§10. `PERCEPTION_ENABLE=OFF` produces a `unitree_mujoco` binary byte-for-byte identical to the pre-change baseline (SHA-256 verified).

**P4: a validated vertical slice, not the full phase.** Before implementation, the two Mid-360 documents (`g1_mid360_extrinsic.md`, `g1_mid360_sensor_model.md`) were exhaustively checked against this branch (`grep`, full `git log --all`) and confirmed to describe work from a **different working tree** (`~/unitree_rl_mjlab_backup`) that was never landed here — this repository's original audit ("no lidar/raycast/point-cloud code anywhere") was correct as written, and there was no existing `MountedLidarSimulator` to reconcile with or wrap. Everything below is a from-scratch implementation of the module boundaries in §6, using the two documents purely as the numeric/behavioral source of truth (extrinsic, FOV, aperture defect, `raycast_exact` defect).

Delivered: `RaycasterMj`/`IRaycaster`, `RayPatternGenerator` (uniform 360×32 grid), `FrameProviderMj`, `ExtrinsicGuard` (hard-fails on MJCF/YAML disagreement), the head-shell aperture fix (`group="2"` + `geomgroup` mask) and `raycast_exact` (exact `mj_ray` per-ray, default true — batched `mj_multiRay` silently missed up to 19–24% of wall hits in this repo's own measurement), a typed `perception/configs/perception.yaml` loader, `perception_lidar_bench` (standalone, no DDS/joystick needed), and a live OpenCV scan view. Validation suite V0 (aperture dynamics-neutrality, new) through V5 + T1 all pass (35/35 checks) and independently reproduce the source docs' own V1–V5 numbers to within expected variance (different elevation-endpoint/range-cutoff conventions, documented). Deliberately **not** built yet, per the original scope boundary: `TimedPointCloud` queueing to a perception-thread consumer, frame-accumulation windows, deskew — these remain P4/P5 work for later.

**Two deviations from the original spec, now folded into the sections below:**
1. **Three files edited outside `perception/`, not two** — `src/assets/robots/unitree_g1/xmls/scene_g1.xml` was unavoidably added (the `<site>` pose and the aperture `group="2"` are MJCF authoring facts with no runtime API equivalent). See the updated §16/§23.
2. **Live visualization ships as a new standalone OpenCV window** (`Mid-360 live scan (perception)`, top-down + range-image panels), not as an overlay merged into the existing `DpcbfVisualizer` window with ground-truth obstacles drawn in the same frame. Two reasons: (a) P0/P4 has no detector yet, so there is nothing to compare against ground truth except raw points; (b) the vendored MuJoCo `simulate` app only appends user-drawn scene geoms in **passive mode** (`simulate.cc:2861`), and this app runs `is_passive_ = false`, so a native 3D overlay in the MuJoCo viewer itself is currently a dead end without patching vendored third-party code (deliberately not done). This is tracked as an open item — see §13. **Decision (Q20): defer the merge to P8–P10**, when `CircleObservation`/`SafetyObstacle` exist to actually overlay against ground truth — confirmed by the user rather than assumed.

**Follow-up — Mid-360 rigid-attachment diagnosis (2026-07-30, same day).** A live-view observation ("the scan looks similar even when the G1 falls over") triggered a dedicated diagnostic before any further phase work, since a real pose-tracking bug would have made every prior V0–V5 result suspect. Verdict: **not a bug — a visualization defect.** New test `perception/tests/adapter/mid360_pose_tracking_test.cpp` ("V6", registered with CTest) compares three independent read paths — the code's own reported sensor pose, an independent direct `mjData` read of `torso_link` composed with the extrinsic, and the pose re-derived purely from the emitted point cloud (trilateration + Kabsch, blind to anything the code reports) — and finds them agreeing to machine precision (position error down to ~1e-16–1e-7 m depending on path, rotation floor 4.9e-10, the same published-quaternion-rounding floor V5 hit) continuously through both a real dynamic fall (`mj_step`, no controller, 1.21 m height excursion, 177° tilt) and a larger prescribed kinematic sweep (1.36 m excursion, 85° tilt, independent waist motion). 16/16 checks pass, run per-scan rather than at discrete static poses like V5.

The actual cause of "looks similar": (1) the live view's point-height colour scale was **sensor-relative**, so an identical world-frame floor rendered a different colour depending on sensor height (actively misleading, not merely uninformative); (2) no unmissable posture indicator existed — the one disambiguating number was buried in small text. Fixed in `live_scan_view.cpp`: absolute world-z colour scale, a STANDING/TILTED/FALLEN banner (from sensor world height + torso tilt), large-type world-Z/tilt/accepted-points/self-hit readouts, and an absolute 0–1.6 m height gauge.

Two minor bugs surfaced and were fixed en route: a rank-deficiency bug in the new V6 test itself (a flat-floor-only scene leaves ray-origin trilateration mathematically unconstrained in one axis — fixed with an SVD conditioning check and richer 3-D test scenes), and an inaccurate code comment about physics/kinematics sync (corrected at the `main.cc` hook). One real, small, **deliberately unfixed** effect was found and documented rather than patched: scan geometry lags its own timestamp by one physics step (~4.7 mm at fall speed — `mj_step` doesn't refresh kinematics until the next `mj_forward`), left for the deskew phase (P5) to correct properly, since it is ~50× smaller than what deskew already has to handle over a 100 ms scan window.

**Not tested:** an actual walking-policy trajectory. A working ONNX velocity policy exists (`deploy/robots/g1/config/policy/velocity/`) but runs as a separate real-time DDS process in `deploy/`, off-limits to edit/reimplement inside a test; the prescribed kinematic sweep was judged stronger evidence anyway (larger excursion, independent joint motion, fully controlled, reproducible). Flagged as a gap if walking-specific dynamics ever matter later — see the new open item below.

**Environment note (superseded by P2, see below):** this sandbox has no physical joystick. `simulate/config.yaml` briefly carried a local `use_joystick: 0` override; P2 reverted that to the tracked `use_joystick: 1` default (§16-adjacent decision, below) and confirmed it as a tracked-vs-local distinction rather than leaving the override in place. Consequently, running the full `unitree_mujoco` binary interactively in this sandbox will still hit a **pre-existing, perception-unrelated** crash (SDK bridge worker thread calls `exit(1)` on a missing `/dev/input/js0`, confirmed to reproduce identically on the untouched baseline binary) — this is not something the tracked default should paper over, since a real robot/joystick environment needs `use_joystick: 1` to work correctly. For local, joystick-free work here, use an uncommitted local override or the standalone tools that never touch the joystick path (`perception_lidar_bench`, `perception_replay`). A keyboard-driven left/right steering input remains on the near-term backlog, re-scoped: it is now a workaround for *this sandbox's* no-joystick crash when someone wants to drive the full closed-loop simulator interactively, not a "joystick replacement" for a disabled default (see Q25).

Full suite after this follow-up: 4/4 tests pass (`dpcbf_safety_filter_test`; `perception_geometry_rpy_test` 11/11; `perception_mid360_bringup_test` 35/35; `perception_mid360_pose_tracking_test` 16/16, new). `PERCEPTION_ENABLE=OFF` still builds clean and `unitree_mujoco` remains byte-for-byte identical to the pre-change baseline. File count outside `perception/` unchanged at three (the `main.cc` touch this round was comment-only).

**Decision — `simulate/config.yaml` file-count drift (found during P1, resolved here).** P1's reconciliation found `simulate/config.yaml` (`use_joystick: 1 → 0`) showing as a **tracked** modification, not just local runtime state — which would make the edit budget four files, not three. Ruling: this is a joystick-less-sandbox convenience, not an architectural fact of the perception subsystem (unlike the `scene_g1.xml` site/aperture edit, which is a real fact with no other place to live). It should **not** be committed as a change to the repo's tracked default — a future environment with a real joystick would silently inherit it disabled. Instruction for the next agent: revert `simulate/config.yaml` to the tracked `use_joystick: 1` default, and use a local, uncommitted override for this sandbox instead (an untracked local copy, an env var, or `git update-index --skip-worktree` — implementer's choice). This keeps "three files outside `perception/`" accurate rather than amending it to four for an environment quirk.

**P1: COMPLETE.** All remaining §7 contracts implemented (`DeskewedPointCloud`, `GravityAlignedCloud`, `PointLabel`/`GroundPlane`/`GroundSegmentationResult`/`LabeledPointCloud`, `ProjectionStats`/`ProjectedScan`, `Cluster2D`, `FittedPrimitive2D`, `CircleObservation`, `TrackState2D`/`TrackPrediction2D`, `PerceptionObstacle`, `SafetyObstacle`, `OracleObstacleState`, `PipelineStage`/`StageTiming`, `MatchedPairError`, `EstimationDiagnostics`, `FrameInvalidReason`/`RetainedStages`/`PerceptionFrameCapacities`, `PerceptionFrame`) plus a `Validate()` retrofitted onto `FrameTransformSnapshot` (previously just a bare `valid` bool — the load-bearing new check is `world_from_base * base_from_sensor == world_from_sensor`, which is exactly the composition-consistency question the V6 diagnosis needed and now gets asserted per-snapshot, not just investigated once). The full §14 config tree is typed and validated (`frames`, `motion_compensation`, `self_filter`, `segmentation`, `projection`, `detection`, `tracking`, `safety`, `dpcbf_adapter`, `mode`, `ros2`, `dumps`/`metrics`/`profiling`, `experimental`), every value tagged SOURCED/UPSTREAM/JUDGEMENT in the YAML so a judgement call can't later be quoted as a measurement, with 17 cross-section validation rules (e.g. `safety.max_radius_m ≥ detection.max_circle_radius_m` — a safety ceiling below the detector's cap would silently shrink large obstacles, the one unrecoverable error per §17). Detection/tracking defaults were pulled from the audited upstream `obstacle_detector` source rather than invented, with one deliberate exception documented in-line: `radius_enlargement_m` is left at upstream's value even though it roughly doubles this arena's obstacle size, pending the P8 short-arc-bias experiment (Q11) rather than retuned as a guess now.

Test suite grew from 4/4 to **6/6** (`perception_contracts_test` 178/178, `perception_config_test` 82/82, new), zero warnings, and the `PERCEPTION_ENABLE=OFF` byte-identity check was re-verified from a fresh git-stash baseline rather than assumed (three independently-built binaries, identical SHA-256). Zero edits outside `perception/` this phase (the config.yaml drift above predates P1 and is being resolved, not caused, here). Two small doc corrections surfaced and are applied below: §19 Q15 cited `count=20 [FACT max]` for the dynamic-obstacle count; `dpcbf/config/dpcbf_config.yaml` actually says `count: 90` — corrected in place.

**P2: COMPLETE.** The `simulate/config.yaml` drift from P1 is resolved: reverted to the tracked `use_joystick: 1` default, confirmed via `git diff --stat` back to the exact three-file P0/P4 baseline (unchanged line counts), no local override needed since P2's real-data path runs entirely through joystick-free standalone harnesses (`perception_lidar_bench`, `perception_dump_bringup_test`). See the corrected Environment note above.

Every one of the 24 §7 contract types now has a working binary + JSONL dump writer/reader, built on one generic mechanism (`VisitFields` field declarations walked by four codec visitors) rather than per-type code, so a writer and its reader cannot drift apart. Dump files carry a header (dump schema version — **deliberately distinct** from config `schema_version`, per §14 — `RetainedStages` bitmask, record counts/ranges, and a full provenance block including git sha/dirty-flag and config hash) plus a footer; a `run_manifest.json` mirrors the `dpcbf_rollout_evaluator`/`tune_dpcbf` reproducibility style (§2.1/§14) across a whole dump directory. `RetainedStages` resolves to four states on replay, not two: not-recorded, recorded-empty (a real finding), recorded, and flag/disk-inconsistent (trusted by nothing) — all four exercised by tests. `perception/apps/perception_replay.cc` (reserved in §9) is implemented and reads real dumps produced by the live P4 bring-up path (`TimedPointCloud`, `ScanStats`, `FrameTransformSnapshot`) bit-identically against what the live scan reported, closing the loop end to end; 21 of the 24 contract types are necessarily fixtured synthetically (§9 acceptance target is a format, not premature pipeline logic — no P5+ stage exists yet to produce them).

Test suite grew to **8/8** (`perception_dump_roundtrip_test` 564/564 — every contract × {int-exact, float-bit-exact} × {binary, JSONL}, including deliberately pathological values like `-0.0`, the smallest denormal, and `1/3`; `perception_dump_bringup_test` 50/50 — real-data replay against the live bring-up, both new). Versioned-schema rejection is a hardened negative-test suite (wrong dump-schema version, wrong magic, unknown record-type id, truncated file with no footer, cross-type read, out-of-range enum, concatenated files, missing required JSONL field, manifest/footer disagreement — all refused with a specific error, none silently misparsed). The no-MuJoCo build target (§9's "builds without MuJoCo" requirement) was re-verified with a nonexistent MuJoCo root: `perception_diagnostics`/`perception_config`/`perception_core`/`perception_replay` and 4 tests build and pass — a fixture can be replayed on a machine that cannot run the simulator. `PERCEPTION_ENABLE=OFF` byte-identity was re-verified from a fresh stash and initially showed a false mismatch, root-caused to FetchContent'd osqp-cpp/abseil embedding absolute build-directory paths into the binary — a pre-existing property of the `dpcbf`/osqp build, not something P2 introduced, but any future re-run of this check must hold the build path constant or it will report a spurious failure.

Known limitation carried forward deliberately rather than built now: `DumpReader` loads whole files at `Open()` (no streaming); fine for today's corpora, flagged as needing a dump-schema-version bump (per-record length prefixes) before P15-scale campaigns with retained stage clouds. `OracleObstacleState` (record type 20) is fully wired and unused — P3's bit-exact oracle-equivalence acceptance test can be written directly against dumped fixtures with no new format work.

**P3: COMPLETE.** `OracleProviderMj` (`adapters/mujoco`, the one module permitted both a MuJoCo header and a dpcbf one — and permitted only `dpcbf/dynamic_obstacles.h`, never `dpcbf::ObstacleState`) and `ObstacleSourceSelector` (`integration`) are wired in oracle-only mode; the inline `DynamicObstacle → ObstacleState` conversion has MOVED out of the `main.cc` axis-filter lambda without changing. Integration mode 1 is now explicit rather than implicit.

**Design decision — `adapters/dpcbf` was created now, not deferred to P11 (option (a)).** Something had to turn `OracleObstacleState` into `dpcbf::ObstacleState`, and the dependency rules name `adapters/dpcbf` as the only module allowed to touch that type (integration may *name* the adapter's output type, not perform the conversion). `adapters/dpcbf/oracle_to_dpcbf.{h,cpp}` is therefore a trivial, total, order-preserving field copy — no inflation, no gating — linked as `perception_dpcbf_adapter`, which links `perception_contracts` plus a headers-only `perception_dpcbf_headers` interface target and, deliberately, **no MuJoCo** (verified: it builds and its tests pass in the no-MuJoCo configuration). P11 adds `SafetyObstacle → dpcbf::ObstacleState` *beside* it rather than migrating live safety-critical conversion code out of a file that would by then also hold mode-selection logic. The include-path split (`perception_mujoco` may include `dynamic_obstacles.h` only; `perception_dpcbf_adapter` may include `dpcbf_safety_filter.h` only) is the one rule a CMake link set cannot enforce by itself and is stated in `perception/CMakeLists.txt` at the target that grants it.

**The `dpcbf_adapter` truncation knobs are deliberately NOT applied in oracle mode, and must not be "fixed" later without a decision.** `dpcbf_adapter.max_obstacles: 20` and `drop_invalid: true` exist in config (P1) but belong to the P11 adapter on the ESTIMATED path. The shipped `dpcbf_config.yaml` runs `count: 90`, so honouring the cap would drop 70 obstacles; this was measured, not assumed — truncating to 20 changes `DpcbfSafetyFilter`'s output on **30/30** probed samples. `DpcbfSafetyFilter` does its own top-k selection (`default_num_constraints: 10`) from whatever it is given, which is why the unfiltered list is the correct input.

**Acceptance methodology and result.** The golden fixture was captured *before* the new modules existed, by a tool (`perception_oracle_golden_capture`) carrying a verbatim transcription of the pre-change inline conversion, and committed: `perception/tests/fixtures/oracle_golden/oracle_obstacles.jsonl` (2700 `OracleObstacleState` records, dump record type 20 — P2's mechanism used for exactly what it was built for) and `dpcbf_obstacle_states.txt` (the final `dpcbf::ObstacleState` vectors as exact hex doubles; a plain test-side log, because `dpcbf::ObstacleState` may not enter `core/diagnostics`). `perception_oracle_equivalence_test` (48/48) then checks the new path from four independent directions on a 600-step / 1.2 s run of the real G1 scene with 90 moving obstacles: (A) against the verbatim old code, same snapshot; (B) against the committed dump fixture; (C) against the committed pre-change state log, in order; (D) **end to end through two independent real `DpcbfSafetyFilter` instances** — `SafetyFilterResult` identical on all 30 samples across command, acceleration, constraint counts, decay/slack variables and the whole `selected_obstacles` set. All comparisons are bit-exact (`memcmp`, so `-0.0` and NaN answer correctly), order-strict, and id-for-id. The robot ground-truth pose leg matches the inline `ReadRobotGroundTruth` bit-exactly too.

Two findings worth carrying forward rather than burying. First, 2 of 30 samples drive OSQP to `kMaxIterations` (the uncontrolled robot falls into a 90-cylinder arena) and the filter holds its last feasible command — pre-existing behaviour under a pathological state, and the equivalence holds *through* those samples on both instances, which is stronger evidence than a run that never stressed the solver. Second, reversing the obstacle list changed `DpcbfSafetyFilter`'s output on **0/30** samples: its top-k selection is empirically order-invariant on this corpus, so order preservation is defence-in-depth here rather than presently load-bearing — measured and reported rather than assumed either way. The refactor preserves order regardless, and the comparator is order-strict on 30/30 probed samples, so §A/§C could not have passed on a silent reordering.

**Not covered, stated plainly:** a full closed-loop rerun of the `unitree_mujoco` binary was not possible in this sandbox. The binary starts, prints `[perception] obstacle source: oracle`, binds and runs, but the axis-filter lambda is never invoked because the bridge worker hits the pre-existing, perception-unrelated joystick crash (`Error: Joystick open failed.`) — confirmed to reproduce identically on the `PERCEPTION_ENABLE=OFF` baseline binary. What the acceptance test covers instead is that same call chain (real scene, real `DynamicObstacleManager`, real `DpcbfSafetyFilter`) minus the DDS/joystick plumbing around it. Q25 (keyboard steering) remains the unblocker for a genuine interactive closed-loop rerun here.

Test suite grew from 8/8 to **9/9**. `PERCEPTION_ENABLE=OFF` byte-identity was re-verified from a genuinely fresh baseline with the build path held constant per the P2 osqp/abseil lesson: a pristine-tree build (tracked edits stashed), a pre-edit build and a from-scratch post-edit build, all three in the same directory, all SHA-256 `013e43027ca05fca...`. Zero new files outside `perception/`; the three-file budget (`simulate/CMakeLists.txt`, `simulate/src/main.cc`, `scene_g1.xml`) is unchanged, and `main.cc`'s hook marks were renumbered `n/4 → n/5` with the new obstacle-source hook as `PERCEPTION HOOK 5/5` inside the lambda. With `PERCEPTION_ENABLE=OFF` that hook's `#else` arm *is* the original inline text, so the rollback is a deletion of the `#ifdef` arm rather than a rewrite.

---

## 2. Repository audit

### 2.1 Build & language landscape

| Item | Finding |
|---|---|
| Build system | CMake, C++17. `simulate/CMakeLists.txt` builds `unitree_mujoco` and does `add_subdirectory(dpcbf)`; `dpcbf/CMakeLists.txt` defines 3 static libs + evaluator + CTest test `[FACT]` |
| Python side | `setup.py`: package `unitree_rl_mjlab`, deps `mjlab==1.2.0`, `mujoco-warp==3.5.0` `[FACT]` |
| Eigen | System `/usr/include/eigen3`; used by `deploy/` headers (`isaaclab/*`), not by `simulate/` or `dpcbf/` `[FACT]` |
| YAML | yaml-cpp everywhere (`simulate/src/param.h:5`, all three dpcbf `LoadConfig`s) `[FACT]` |
| QP | OSQP via FetchContent-pinned `osqp-cpp` + abseil (`dpcbf/CMakeLists.txt:8-21`) `[FACT]` |
| ROS/ROS2 | None in production code; only the two untracked reference packages `[FACT]` |
| Test framework | No gtest. `dpcbf_safety_filter_test` is a plain executable registered with CTest (`dpcbf/CMakeLists.txt:66-68`) `[FACT]` |
| Logging | spdlog in `deploy/` (`deploy/include/param.h:19-36`); plain stdout in `simulate/` `[FACT]` |
| Metrics | `dpcbf_rollout_evaluator` writes JSON metrics (collisions, clearance, tracking MSE…) headlessly (`dpcbf/parameter_optimization/dpcbf_rollout_evaluator.cpp:611-647`) `[FACT]` |

### 2.2 Runtime architecture of the simulator (the integration target)

`[FACT — all from simulate/src/main.cc]`:

- Globals: `mjModel* m`, `mjData* d` (lines 106-107); `dpcbf::DynamicObstacleManager dynamic_obstacles`, `DpcbfSafetyFilter safety_filter`, `DpcbfVisualizer dpcbf_visualizer` (lines 109-111).
- **Physics thread** (`PhysicsThread`, 605-639): loads model, `mj_makeData`, runs `PhysicsLoop` (400-599). Steps under `std::recursive_mutex sim.mtx` (line 478). Calls `dynamic_obstacles.Step(m, d, m->opt.timestep)` before `mj_step` (lines 529/571).
- **Bridge thread** (`UnitreeSdk2BridgeThread`, 641-723): waits for `d`, resolves DPCBF base body (`pelvis` per config, fallback logic at 657-660), `safety_filter.Initialize(m->opt.timestep)` (line 670), installs a joystick `axis_filter` lambda (677-709) that per invocation: snapshots obstacles (687) → converts `DynamicObstacle` → `ObstacleState` (686-695) → `ReadRobotGroundTruth(m, d, body_id)` (696) → `safety_filter.Filter(...)` (697) → `dpcbf_visualizer.Update(...)` (698). Bridge publishes robot state via `unitree_sdk2` DDS at 1 kHz (`simulate/src/unitree_sdk2_bridge.h:170-248`).
- **Visualizer thread**: OpenCV window at 30 Hz, receives copies only, never touches `m`/`d` (`dpcbf/src/dpcbf_visualizer.cpp:90-129, 391-394`).

**Known hazard** `[FACT/INFERENCE]`: the audit found that the `Filter()` call path reads `m`/`d` while the physics thread may be stepping — `sim.mtx` coverage of `ReadRobotGroundTruth` is not clearly complete. The perception subsystem must not worsen this; its MuJoCo reads will be explicitly lock-scoped (see §6).

### 2.3 Oracle obstacle provider

`[FACT — dpcbf/src/dynamic_obstacles.cpp]`: obstacles are mocap bodies named `dpcbf_obstacle_<n>` (line 156), cylinders (`mjGEOM_CYLINDER`, line 164), created via `AddToSpec` (117-199), bound by `mj_name2id` in `BindModel` (201-222), kinematically integrated with arena-boundary reflection in `Step` (224-239), positions written to `mocap_pos` (255-268). Velocity lives only in the C++-side `DynamicObstacle` struct — MuJoCo mocap has no velocity, so ground-truth velocity is available exclusively through `Snapshot()` (250-253, mutex-protected). This is the oracle the perception path will be compared against.

### 2.4 DPCBF interface and call sites

- **Contract:** `ObstacleState {x, y, radius, velocity_x, velocity_y, id}` — meters, m/s, world XY, no timestamp, no validity/confidence field `[FACT, dpcbf_safety_filter.h:19-26]`. Staleness/validity is the caller's job `[INFERENCE]`.
- `Filter(RobotState, VelocityCommand, vector<ObstacleState>) → SafetyFilterResult` `[FACT, header:85-87]`. Not thread-safe `[FACT]`.
- **Call sites:** exactly one in C++ (`simulate/src/main.cc` axis-filter lambda). `deploy/` contains no DPCBF usage `[FACT]`. Python has an independent, mathematically-identical pure-DPCBF implementation in `src/tasks/navigation/mdp/state.py:283-389` used for RL training with synthetic noisy obstacle observations (noise/dropout/latency injection at `state.py:221-246`) `[FACT]`. The Python path is out of scope for this subsystem but its noise parameters (σ_pos 0.03 m, σ_vel 0.05 m/s, σ_r 0.015 m, dropout 3%, latency 1 step, `navigation_config.py:67-74`) are the training-time assumption the real perception pipeline should eventually be measured against `[INFERENCE → feeds §12 metrics]`.

### 2.5 Robot state, frames, IMU

`[FACT]`: base pose from `d->xpos`/`d->xmat` of the configured base body; yaw via `atan2(mat[3], mat[0])` (`main.cc:133-151`); IMU sensors `imu_quat`/`imu_gyro`/`imu_acc`, world-frame `frame_pos`/`frame_vel` sensors resolved by name (`unitree_sdk2_bridge.h:94-148`). G1 asset (`src/assets/robots/unitree_g1/xmls/g1.xml`) has no lidar mount and an empty `<sensor>` section beyond IMU/camera `[FACT]`. The velocity task uses an mjlab `RayCastSensorCfg` terrain scanner on the Python side only (`src/tasks/velocity/velocity_env_cfg.py:43-49`) `[FACT]`.

### 2.6 Existing-component classification table

*(existing code reuse map — §3 merged here)*

| Component | Path | Role today | Treatment |
|---|---|---|---|
| `ObstacleState`, `RobotState`, `VelocityCommand` | `dpcbf/include/dpcbf/dpcbf_safety_filter.h:19-56` | DPCBF I/O contract | REUSE (unchanged; perception adapter converts into it) |
| `DpcbfSafetyFilter` | `dpcbf/src/dpcbf_safety_filter.cpp` | QP safety filter | REUSE (must not be edited) |
| `DynamicObstacleManager` | `dpcbf/src/dynamic_obstacles.cpp` | Oracle obstacle simulation + GT source | REUSE + WRAP (a thin `OracleProvider` adapter reads `Snapshot()`; the manager itself untouched) |
| `DpcbfVisualizer` | `dpcbf/src/dpcbf_visualizer.cpp` | OpenCV debug view | REUSE (keeps rendering whichever obstacle set feeds the filter) |
| `simulate/src/main.cc` axis-filter lambda | lines 677-709 | Only DPCBF call site | EXTEND (minimal hook: obstacle-source selector; see §16) |
| `PhysicsLoop` | `main.cc:400-599` | Owns stepping + `sim.mtx` | EXTEND (one hook call to sample lidar rays per step window) |
| `simulate/CMakeLists.txt` | — | builds `unitree_mujoco` | EXTEND (one guarded `add_subdirectory(perception)` + link) |
| `dpcbf_rollout_evaluator` | `parameter_optimization/` | Headless metric JSON pattern | REFERENCE ONLY (its JSON-manifest style is the template for the perception evaluator; not modified) |
| `param.h` YAML loading pattern | `simulate/src/param.h:34-43` | config idiom | REFERENCE ONLY (perception gets its own typed config loader in the same yaml-cpp style) |
| Python nav noise model | `src/tasks/navigation/mdp/state.py:221-246` | synthetic perception | REFERENCE ONLY (benchmark target for real-pipeline error statistics) |
| `deploy/` | all | real-robot FSM | must not be edited in this project |
| `obstacle_detector/` | untracked | reference pkg | see §4 |
| `pointcloud_to_laserscan/` | untracked | reference pkg | see §5 |

---

## 3. Existing code reuse map

Covered by table §2.6. Summary of the load-bearing conclusions:

- The DPCBF interface needs no change. Missing timestamp/validity semantics are handled upstream of the interface: the `SafetyObstacle → ObstacleState` adapter simply omits invalid/stale obstacles and applies conservative inflation before conversion `[RECOMMENDATION]`. This is what keeps `dpcbf/` at zero edits.
- The oracle path is already exactly what the ground-truth-principle demands — it stays wired, and the new selector makes it explicit rather than implicit.
- No existing file owns anything the perception core needs to own. Ray simulation, projection, clustering, fitting, tracking are all net-NEW.

---

## 4. `obstacle_detector` audit and adoption strategy

### 4.1 Audit summary

`[FACT unless noted; citations into obstacle_detector/]`:

- ROS1, catkin, package format 2, BSD license (Poznan UT, 2017). Dependencies: `roscpp`, `tf`, `laser_geometry`, Armadillo, Boost.
- Four nodes: `scans_merger`, `obstacle_extractor`, `obstacle_tracker`, `obstacle_publisher`. Only extractor + tracker matter here.

**Extractor pipeline** (`src/obstacle_extractor.cpp`):

- LaserScan ingestion: polar unroll using `angle_min`/`angle_increment`, filters by `range_min`/`max` (132-146). Also accepts unordered `sensor_msgs::PointCloud` (148-156). No NaN/inf guard on ranges `[FACT]` — our adapter must pre-filter.
- Grouping (173-214): break condition `distance < max_group_distance + range * distance_proportion`; occlusion detection via Heron's-formula sine of inter-beam angle vs `sin(2·distance_proportion)` → sets `PointSet::is_visible`.
- Split-and-merge (216-270): recursive max-distance split with threshold `max_split_distance + range * distance_proportion`; least-squares line fit `fitSegment` via `arma::pinv` (`include/obstacle_detector/utilities/figure_fitting.h:54-104`).
- Segment merge (272-324): endpoint proximity (`max_merge_separation`) + collinearity spread test (`max_merge_spread`).
- Circle from segment (`utilities/circle.h:53-56`): circumcircle of equilateral triangle on the segment — `radius = (√3/3)·length`, center offset along segment normal; then `radius += radius_enlargement`; accepted only if `< max_circle_radius`; optionally only from fully-visible segments (`circles_from_visibles`).
- Circle merging (354-403): containment and weighted-overlap merge.

**Tracker** (`src/obstacle_tracker.cpp`, `include/obstacle_detector/utilities/tracked_obstacle.h`):

- Three independent 2-state Kalman filters per obstacle (x, y, radius), state `[value, rate]`, constant-velocity A matrix, measurement of value only (`tracked_obstacle.h:46, 128-162`). Velocity = KF rate state (`tracked_obstacle.h:79-80`).
- Association: greedy over a cost matrix, cost = Euclidean distance in (x, y, r) space, gate `min_correspondence_cost` (238-333). Explicit fusion (N tracks → 1 measurement; covariance-weighted merge, 405-441) and fission (1 track → N measurements, 443-454).
- Lifecycle: fade counter = `loop_rate × tracking_duration` decremented per timer tick; timer-driven update at `loop_rate` (default 100 Hz) decoupled from measurement arrival `[FACT]` — a design we will not copy (see risk R11).
- Messages: `CircleObstacle {center, velocity, radius (inflated), true_radius}` `[FACT, msg/CircleObstacle.msg]` — note upstream already distinguishes inflated vs true radius, which we mirror in `SafetyObstacle`.

**ROS coupling map** `[FACT]`: `utilities/point.h`, `segment.h`, `circle.h`, `point_set.h`, `figure_fitting.h`, `kalman.h` are ROS-free (Armadillo + STL). ROS enters only at: message parsing, tf transforms, node parameter plumbing, publishing.

**Livox-hostility** `[FACT]`: grouping and occlusion logic assume uniform angular increments; non-uniform rosette sampling breaks `distance_proportion` semantics and `is_visible`.

### 4.2 Adoption decision

The hypothesis in the task statement is validated, with one sharpening: because we insert a uniform-grid scan projector before detection (§5), the Livox-hostile assumptions are neutralized at the boundary — the detector always sees an ordered, uniform-increment scan. Therefore the algorithms can be ported essentially faithfully.

`[RECOMMENDATION]` Per-component classification:

| Component | Classification | Rationale |
|---|---|---|
| Grouping + occlusion logic | PORT (Eigen/STL, faithful semantics) | ROS-free already; Armadillo unused here; needs our `ProjectedScan` input type |
| Split-and-merge | PORT | ROS-free; replace `arma::pinv` least squares with Eigen SVD/QR solve |
| `fitSegment`/`fitCircle` | PORT | same |
| Circle-from-segment (√3/3 rule) + `radius_enlargement` | PORT | 4 lines of math; keep as the default fitting backend; alternative fitters can be added behind the same interface later |
| Segment/circle merging | PORT | ROS-free |
| Tracker KF core (`kalman.h`, per-axis 2-state) | PORT with redesigned scheduling | Math ported; the timer-driven decoupled update loop is REPLACEd by measurement-driven update with explicit timestamps (risk R11) |
| Association + fusion/fission | PORT | conceptually reused; gate values re-tuned |
| `Obstacles.msg` family | REFERENCE ONLY (interface emulated) | our `PerceptionObstacle` carries the same information natively; the optional ROS2 adapter may publish an equivalent message but never uses upstream msg definitions (they're ROS1) |
| `scans_merger`, `obstacle_publisher`, `nodes/`, `launch/`, rviz panel plugin | REFERENCE ONLY / not used | ROS1-specific, out of scope |
| Whole package as external process | rejected | ROS1 in a ROS-free repo (and next to a ROS2 package) is a non-starter `[FACT-driven]` |
| Upstream ROS-free headers compiled in a test-only target (linking Armadillo) | code used only in tests | serves as the regression oracle: same synthetic scan → upstream core vs ported core, compare segments/circles/track outputs |

**Distinctions requested:**

- Code reused directly — none in production.
- Algorithms conceptually reused — all of the above PORTs.
- Interfaces emulated — `CircleObstacle` field semantics inside `PerceptionObstacle`.
- Test-only code — upstream `utilities/*.h` + `figure_fitting.h` behind a `PERCEPTION_BUILD_UPSTREAM_ORACLE` CMake option (requires Armadillo present; OFF by default).
- Never enters production — everything ROS1.

**License:** BSD-3 — retain upstream headers' license blocks in ported files and add an attribution note in `perception/third_party_notices.md` `[RECOMMENDATION]`.

---

## 5. `pointcloud_to_laserscan` audit and adoption strategy

### 5.1 Audit summary

`[FACT — pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp]`: ROS2/ament component, BSD. Conversion core (lines 155-228): allocate `ceil((angle_max-angle_min)/increment)` bins initialized to `inf` (or `range_max+inf_epsilon` if `use_inf=false`); per point: NaN skip → height band `[min_height, max_height]` → `range = hypot(x,y)` with `[range_min, range_max]` → `angle = atan2(y,x)` window → `index = (angle - angle_min)/increment` → keep min range per bin (nearest return). Output stamp copied from input cloud; `time_increment = 0.0`; `scan_time` is pure metadata. TF transform per message only when `target_frame` set; `SensorDataQoS`; lazy-subscription thread (plus a real bug in the reverse node's destructor, `laserscan_to_pointcloud_node.cpp:95` sets `alive_ = true` — evidence this code shouldn't be treated as untouchable) `[FACT]`.

### 5.2 Option analysis (A/B/C/D)

| Criterion | A (core cloud → ROS2 p2l node → detector eats ROS LaserScan) | B (internal projector; ROS2 only adapts outward) | C (everything external ROS nodes) | D (hybrid: B in production + original packages as validation baselines) |
|---|---|---|---|---|
| Maintainability | poor — ROS2 in the middle of the pipeline | good | worst (ROS1+ROS2 mix impossible without bridge `[FACT]`) | good |
| Latency | +2 serializations + DDS hop | in-process, zero-copy | ++hops | =B |
| Deterministic sim | broken (async QoS best-effort) | deterministic, same-thread | broken | =B |
| Timestamp/deskew integrity | stamp survives but sim-time vs ROS-time mapping needed mid-pipeline | full control, sim-time end-to-end | worst | =B |
| Testable w/o ROS2 | no | yes | no | yes |
| Headless eval | no | yes | no | yes |
| Build complexity | rclcpp required always | rclcpp optional | two ROS distros | rclcpp optional |
| Tracking-state ownership | split across processes | in core | external | in core |
| Clean deletion | hard | delete `perception/` + 2 hooks | hard | =B |
| Mid-360 HW migration | plausible (Livox ROS2 driver) but core still ROS-free is better | best: swap MuJoCo adapter for a Livox-driver adapter feeding the same `TimedPointCloud` | tied to ROS graph | =B |

`[RECOMMENDATION]` Adopt **Option D**, with Option B as the production path. This is not merely echoing the stated preference; Option C is eliminated by fact (ROS1/ROS2 mismatch), Option A fails determinism and headless-evaluation requirements, and D adds to B exactly what the ground-truth principle wants: the original ROS2 `pointcloud_to_laserscan` node retained as an out-of-process behavioral baseline. Because the projector math is ~60 lines `[FACT]`, the equivalence test can even run without ROS2: a test fixture re-implements the upstream loop verbatim from the audited source and compares bin-for-bin (exact equality expected — same floats, same order), plus an optional CI job that runs the real node when a ROS2 environment exists `[OPEN: ROS2 distro availability, §19]`.

**Per-component classification:** binning math — PORT (canonical production path); TF/QoS/lazy-subscription plumbing — REFERENCE ONLY; the ROS2 node itself — benchmark baseline / optional validation node; `laserscan_to_pointcloud` reverse node — not used; `dummy_pointcloud_publisher` — not used. License BSD-3, same attribution treatment as §4.

---

## 6. Proposed module architecture

The user's preferred layering is validated with two amendments `[RECOMMENDATION]`:

1. The `obstacle_detector`-compatible processing lives inside the core as ported algorithms behind backend interfaces (`IDetector2D`, `ITracker2D`) — not as an external process (§4), not as a compiled third-party lib (Armadillo dependency + ROS1 build system make that worse than porting ~800 lines).
2. A thin integration layer (`PerceptionRunner` + `ObstacleSourceSelector`) is added between the adapters and `main.cc`, so the application touches exactly one facade.

```
┌────────────────────────────────────────────────────────────────────┐
│ simulate/src/main.cc  (2 hooks: sample-rays, obstacle-source)      │
└───────────────┬────────────────────────────────────┬───────────────┘
                │                                    │
        perception/adapters/mujoco          perception/integration
        (raycast, frames, oracle GT)        (PerceptionRunner thread,
                │                            ObstacleSourceSelector,
                ▼                            comparison Evaluator)
┌────────────────────────────────────────────────────────────────────┐
│ PERCEPTION CORE (Eigen/STL only)                                   │
│  contracts → pipeline stages:                                      │
│  deskew → gravity align → self filter → ground seg (ISegmenter)    │
│  → scan projection (ported p2l math) → detection (IDetector2D,     │
│    ported obstacle_extractor) → tracking (ITracker2D, ported       │
│    obstacle_tracker) → safety-state generation                     │
│  diagnostics + debug-dump (file I/O only, no framework deps)       │
└───────┬──────────────────────┬─────────────────────┬───────────────┘
        │ optional             │                      │
  adapters/travel        adapters/dpcbf         adapters/ros2 (optional)
  (TRAVEL ISegmenter)    (SafetyObstacle →      (contracts → PointCloud2/
                          dpcbf::ObstacleState)  LaserScan/Markers/TF)
                                │                      │
                          dpcbf/ (unchanged)        RViz2
```

### Per-module answers (architecture questionnaire)

*Compact form: Exists/separate because · owns · per-frame alloc · multi-frame state · calling thread · reentrant/deterministic · frame in→out · testing.*

- **`core/contracts`** — pure types + validation; separated so every other module depends only on it. Owns nothing at runtime. No alloc rules of its own. Any thread. Deterministic trivially. Tested by contract unit tests, no MuJoCo/ROS.
- **`adapters/mujoco/raycaster`** — the only module (with `oracle_provider` and `frame_provider`) allowed to include MuJoCo. Owns the `RayPattern` instance and per-scan scratch buffers (preallocated). Called from the physics thread inside the `sim.mtx` critical section (rays must be cast against a consistent `mjData`); emits `RawTimedPoint` batches into a SPSC queue. Deterministic given seed + sim state. Self-hits suppressed at the source via `mj_ray`'s `bodyexclude`/`geomgroup` arguments `[FACT: API available in mujoco.h]` plus defensive min-range filter in core. Tested against analytic scenes (unit-box rooms) using a tiny standalone MuJoCo harness (needs MuJoCo, headless, no ROS).
- **`adapters/mujoco/frame_provider`** — snapshots base pose/velocity + orientation (gravity) per physics step into a ring buffer of `FrameTransformSnapshot`; consumed by deskew. Owns the ring buffer. Physics thread writes, perception thread reads (lock-free ring, monotonic sim-time keys).
- **`adapters/mujoco/oracle_provider`** — wraps `DynamicObstacleManager::Snapshot()` + `mjData` robot pose into `OracleObstacleState` sets with sim-time stamps. May know MuJoCo and dpcbf's `DynamicObstacle` type. Never knows about the estimator (forbidden dep).
- **`core/pipeline/deskew`** — per-point motion compensation to scan reference time using pose interpolation from `FrameTransformSnapshot`s. Stateless between frames apart from config; allocates into caller-provided buffers. Perception thread. Deterministic. In: sensor frame, out: base frame at `t_ref`.
- **`core/pipeline/gravity_align`** — rotates cloud into yaw-preserving gravity-aligned base frame using the orientation in the snapshot (sim: ground truth quaternion; HW later: IMU estimate). Stateless.
- **`core/pipeline/self_filter`** — defensive cylinder/box exclusion around the robot (post-raycast belt-and-braces; primary exclusion already at source). Stateless. Knows the G1 only through config numbers (radius/height band), not through the model — keeps the core G1-agnostic; the config default is G1-specific.
- **`core/segmentation`** — `ISegmenter` interface; production default `ZBandSegmenter` (gravity-aligned z threshold + optional 1-plane RANSAC refinement); `adapters/travel` provides `TravelSegmenter` later (only module that may know TRAVEL). Stateless per frame. `[INFERENCE: the DPCBF arena is flat (arena config, flat floor), so a z-band is a sound production default and TRAVEL is genuinely optional — see §22.]`
- **`core/projection/scan_projector`** — ported p2l math over `LabeledPointCloud` non-ground points → `ProjectedScan`. Owns nothing; fills a caller-owned scan buffer. Deterministic, ordered output. This module resolves the Livox-hostility for the detector (§4.2).
- **`core/detection`** — `IDetector2D`; default `SegmentCircleDetector` (ported extractor). Per-frame state only (cleared each scan, mirroring upstream `obstacle_extractor.cpp:159-160`). Consumes `ProjectedScan` (gravity-aligned base frame), outputs `CircleObservation`s in the same frame, transformed to world by the pipeline using the scan's `FrameTransformSnapshot` (world-frame tracking matches how DPCBF consumes states and how upstream tracked in a fixed frame).
- **`core/tracking`** — `ITracker2D`; default `KfCircleTracker` (ported per-axis KFs + association + fusion/fission), measurement-driven with explicit dt from timestamps. Owns the multi-frame tracking state (the answer to "which module owns tracking state"). Perception thread only; not reentrant; deterministic given input sequence.
- **`core/safety`** — `SafetyStateGenerator`: validity gating (min track age, max staleness), conservative inflation (`radius ← max(fitted, enclosing) + k_σ·σ_r + latency·|v|·k_lat`), uncertainty fields. Owns conservative safety inflation. Outputs `SafetyObstacle`s.
- **`core/diagnostics` + debug-dump** — owns `StageTiming`, counters, and binary/JSONL dumps of every contract type (the headless "visualization"). Only module doing file I/O in core. Logging responsibility: each stage returns status; only the runner and diagnostics log.
- **`integration/PerceptionRunner`** — owns the perception thread, the queues, stage wiring, config root, and the latest-`PerceptionFrame` double buffer. Owns scan timing (accumulation windows). The only module the app constructs.
- **`integration/ObstacleSourceSelector`** — selects oracle vs estimated per configured mode; implements fallback (per-frame, see §16). Never mutates either source.
- **`integration/Evaluator`** — performs oracle-to-estimate comparison (association by nearest-center, error stats, JSON manifests in the `dpcbf_rollout_evaluator` output style). Reads both paths; forbidden from feeding anything back into the estimator.
- **`adapters/dpcbf`** — `SafetyObstacle` → `dpcbf::ObstacleState`; the only module that may include dpcbf headers. Forbidden from MuJoCo.
- **`adapters/ros2`** — optional; converts contracts to sensor_msgs/visualization_msgs/TF and publishes from its own thread reading the `PerceptionFrame` double buffer. May never own or mutate algorithm state (enforced: it only ever receives `const PerceptionFrame&` copies).

**Who may know what (explicit):** MuJoCo → only `adapters/mujoco/*`; ROS2 → only `adapters/ros2/*`; `obstacle_detector` (upstream code) → only test targets; `pointcloud_to_laserscan` (upstream) → only test targets + optional external baseline; TRAVEL → only `adapters/travel`; DPCBF → only `adapters/dpcbf` (+ integration via the adapter's output type); Unitree G1 specifics → only config defaults and `adapters/mujoco` (body/geom names); frame transforms → owned by `frame_provider` (produce) and pipeline (consume).

---

## 7. Internal data contracts

**Amendment (§1.1):** five of these contracts are already implemented and test-covered as of the P4 bring-up slice — `RayPattern`, `TimedPointCloud`/`RawTimedPoint`, `ScanStats` (the shipped name for what this table calls the diagnostics side of `RawTimedPoint`/scan census — tracks `no_hit`/`self_rejected`/`range_rejected`/`accepted` with an asserted `IsBalanced()` invariant so every ray is accounted for exactly once), `FrameTransformSnapshot`, and `FrameId`. Note also a resolved ambiguity worth carrying forward: "self-hit fraction" has two plausible denominators (self-hits ÷ rays cast, vs. self-hits ÷ raw hits) that differ meaningfully (0.0415 vs. 0.0553 on identical data) — the shipped definition is `self_rejected / rays_cast`, and any future stage reporting a "fraction" metric should state its denominator explicitly in the contract, not just in a comment.

All contracts live in `perception/include/perception/core/contracts/`.

**Global conventions** `[RECOMMENDATION]`: time = double seconds of MuJoCo sim time (`mjData.time` — the same clock the bridge already exports as `tick`, `unitree_sdk2_bridge.h:227` `[FACT]`); frames named by enum `{kSensor, kBase, kGravityAlignedBase, kWorld}`; invalid scalar = NaN internally with explicit validity flags at object level; all pipeline types are value types, immutable after stage output; buffers preallocated to configured maxima (no steady-state allocation).

| Contract | Semantics / key fields | Owner & lifetime | Frame | Notes |
|---|---|---|---|---|
| `RayPattern` | precomputed unit directions + intra-frame time offsets for one Mid-360 rosette period; fields: `dirs[N]`, `t_offset[N]`, `period` | raycaster, static per config | sensor | regenerated only on config change; deterministic function of seed |
| `TimedRay` / `RayHit` | one cast: dir, t_offset, hit flag, distance, geomid/bodyid | transient (stack) | sensor | bodyid retained → self-hit accounting metric |
| `RawTimedPoint` | xyz (float), t_offset (float, rel. to scan stamp) | inside `TimedPointCloud` | sensor | |
| `TimedPointCloud` | contiguous `RawTimedPoint[]` + stamp (scan end time), frame, seq id, motion-comp status = none | produced by raycaster, moved through queue, owned by pipeline frame | sensor | ordering: acquisition order; zero-copy views not required (≤ ~20k pts/frame at 200k pts/s ÷ 10 Hz `[FACT Livox spec / OPEN Q7]`) |
| `DeskewedPointCloud` | same layout, t_offset retained, status = deskewed-to-stamp | pipeline | base @ stamp | |
| `GravityAlignedCloud` | same, plus roll/pitch removed | pipeline | gravityAlignedBase | |
| `LabeledPointCloud` | + per-point label {ground, nonground, self, invalid} | pipeline | gravityAlignedBase | replaces separate `GroundSegmentationResult` container; `GroundSegmentationResult` = label stats + plane params |
| `ProjectedScan` | `ranges[B]`, `angle_min=-π`, `angle_max=+π`, `increment=2π/B` (B config, default 360→1°), `range_min`/`max`, missing bin = NaN (internal policy; ROS2 adapter converts NaN→+inf to match `use_inf=true` semantics `[FACT of upstream defaults]`), nearest-return per bin, stamp = cloud stamp, `time_increment=0` (post-deskew scan is synchronic — matches upstream's `time_increment=0.0` `[FACT]`), height-band metadata `[z_lo, z_hi]`, frame, motion-comp status | pipeline | gravityAlignedBase | the detector-facing contract; the ported detector requires only these numerical semantics, not `sensor_msgs/LaserScan` (upstream ingestion uses only `angle_min`/`increment`/range bounds `[FACT obstacle_extractor.cpp:132-146]`) |
| `ScanBin` | implicit (index ↔ angle); no separate struct | — | — | keep it an array, not objects |
| `Cluster2D` | point-index span + visibility flags (ported `PointSet`) | detector, per frame | gravityAlignedBase | |
| `FittedPrimitive2D` | segment (endpoints, normal) or circle | detector, per frame | gravityAlignedBase | |
| `CircleObservation` | center, radius_fitted, arc extent, visibility, fit residual, stamp | detector out; transformed to world by pipeline | world (after transform) | uncertainty: residual + arc-length proxy → measurement σ for tracker |
| `TrackState2D` / `TrackPrediction2D` | id, center, velocity, radius + per-axis variances (from ported KFs), age, hits, misses, status {tentative, confirmed, coasting} | tracker, lifetime = track | world | |
| `PerceptionObstacle` | confirmed-track view: id, center, radius_true, velocity, covariances, last_update, confidence | tracker output per frame | world | mirrors upstream `CircleObstacle {radius, true_radius}` semantics `[FACT §4.1]` |
| `SafetyObstacle` | inflated radius, center (+ optional position inflation), velocity, validity, staleness, source = estimated | safety stage | world | direct precursor of `dpcbf::ObstacleState` |
| `PerceptionFrame` | the per-scan bundle: stamps, all stage outputs (or dump handles), `SafetyObstacle[]`, `EstimationDiagnostics`, `StageTiming` | runner double buffer (latest-wins) | mixed, each element tagged | ROS2/evaluator/dumps read this only |
| `OracleObstacleState` | GT center/velocity/radius + stamp + id | oracle provider | world | never contains estimator fields |
| `EstimationDiagnostics` | per-stage counts, drop reasons, matched-pair errors vs oracle when comparison mode on | evaluator/diagnostics | — | serialized to JSONL |
| `FrameTransformSnapshot` | sim time, base pose (SE3), base velocity (6D), gravity dir | frame provider ring buffer | world | conversion responsibility: MuJoCo→this in adapter; this→TF in ROS2 adapter |

Serialization: every contract gets a versioned plain-binary + JSONL debug dump writer in `core/diagnostics` (needed for regression fixtures and headless inspection).

---

## 8. Dependency graph and forbidden dependencies

**Allowed edges** (one-way, top depends on bottom):

```
app (main.cc hooks)
 → integration (runner, selector, evaluator)
    → adapters/mujoco  → MuJoCo, core/contracts
    → adapters/dpcbf   → dpcbf headers, core/contracts
    → adapters/ros2    → rclcpp/sensor_msgs [optional], core/contracts
    → adapters/travel  → TRAVEL/PCL [optional], core/contracts
    → core/pipeline → core/{projection,segmentation,detection,tracking,safety}
       → core/geometry → core/contracts → Eigen/STL only
    → core/diagnostics → core/contracts

tests → everything, incl. upstream obstacle_detector headers (Armadillo) and
        upstream p2l math re-implementation [test-only]
```

**Forbidden** (all confirmed as "should be forbidden", enforced by CMake target link sets + include-what-you-use review): `core → MuJoCo`; `core → ROS2`; `core → dpcbf`; `core → application/main loop`; `clustering/detection → DPCBF`; `fitting → ROS2`; `tracking → RViz/ROS2`; `dpcbf adapter → MuJoCo`; `ros2 adapter →` owning/mutating tracker state (receives const frames only); any p2l/obstacle_detector-derived module → MuJoCo; visualization → mutation of perception outputs; oracle provider → estimator; evaluator → feeding estimator; `core → yaml-cpp` (config parsed in `integration/config` into plain structs; core receives typed structs only — keeps core dependency-free and satisfies "no configuration access from arbitrary modules").

---

## 9. Proposed directory structure

Adapted to repo conventions (top-level C++ subsystem dirs with own CMakeLists, like `dpcbf/` `[FACT]`):

```
perception/
  CMakeLists.txt                  # options: PERCEPTION_ENABLE (default ON),
                                  # PERCEPTION_WITH_ROS2 (OFF), PERCEPTION_WITH_TRAVEL (OFF),
                                  # PERCEPTION_BUILD_TESTS (ON), PERCEPTION_BUILD_UPSTREAM_ORACLE (OFF)
  include/perception/
    core/contracts/  core/geometry/  core/pipeline/  core/projection/
    core/segmentation/  core/detection/  core/tracking/  core/safety/
    core/diagnostics/  interfaces/           # ISegmenter, IDetector2D, ITracker2D
  src/
    core/...                                  # mirrors include tree
    integration/                              # PerceptionRunner, ObstacleSourceSelector,
                                              # Evaluator, config loader (yaml-cpp here only)
    adapters/mujoco/  adapters/dpcbf/  adapters/ros2/  adapters/travel/
  apps/
    perception_replay.cc                      # headless: dump-file → pipeline → metrics
    lidar_bench.cc                            # headless raycast validation harness
  tests/
    unit/  contract/  adapter/  integration/  regression/
    fixtures/                                 # dumped clouds/scans, analytic scenes (xml)
  configs/perception.yaml
  docs/  third_party_notices.md

obstacle_detector/          # REFERENCE ONLY, untracked → commit as-is or submodule (§15)
pointcloud_to_laserscan/    # REFERENCE ONLY, same
```

- **Production:** `core/*`, `integration/`, `adapters/mujoco`, `adapters/dpcbf`.
- **Adapters/optional deps:** `adapters/ros2` (rclcpp), `adapters/travel` (PCL), upstream-oracle test target (Armadillo).
- **Builds without ROS2 / TRAVEL / obstacle_detector / Armadillo:** everything except the respective optional targets — enforced by the CMake options, all default OFF except the core.
- **Builds without MuJoCo:** all `core/*` libs, `perception_replay`, unit/contract/regression tests. (`adapters/mujoco`, `lidar_bench` need MuJoCo — supplied by the existing simulate/mujoco tree `[FACT]`.)
- **To disable/delete perception:** delete `perception/`, set `PERCEPTION_ENABLE=OFF` (or remove the guarded block in `simulate/CMakeLists.txt`), and the two hook regions in `main.cc` compile away via `#if`/no-op selector default. Rest of repo intact.
- **Hooks remaining outside the directory:** the three files listed in §16 (amended from two — see §1.1).

---

## 10. Phase-by-phase implementation roadmap

**Common to all phases:** repository must build with `PERCEPTION_ENABLE=ON` and `OFF`; oracle behavior byte-identical in oracle-only mode; each phase = one commit (conventional `feat(perception): …`); rollback = revert that commit (no phase edits files owned by another phase); tests via CTest alongside `dpcbf_safety_filter_test` `[FACT pattern]`. "Must not edit" always includes: everything in `dpcbf/`, `deploy/`, `src/`, `scripts/`, both reference packages. Listed below are only phase-specific deltas.

**P0 — Scaffolding & decision record.**
Objective: `perception/` skeleton, CMake options, empty `perception_core` lib linked nowhere; `docs/architecture.md` capturing §6-§9 verbatim. Prereqs: none. New files: CMakeLists, dirs, docs. Existing edits: `simulate/CMakeLists.txt` (guarded `add_subdirectory`). Tests: build matrix ON/OFF. Acceptance: both configs build; `unitree_mujoco` binary unchanged when OFF. Parallel: none (everything waits on this). Artifacts: frozen module boundaries.

**P1 — Core contracts + typed config.**
Objective: all §7 types, validation rules, config structs (`PerceptionConfig` tree, §14) parsed once in `integration/config` (yaml-cpp), immutable thereafter; unknown-key rejection. Inputs: §7 spec. Outputs: `perception_contracts` header lib + `perception_config`. Ownership: N/A (types). Thread: N/A. New files: contracts headers, config loader, `configs/perception.yaml`. Unit tests: construction/validation/round-trip of every contract; config: defaults, bad values, unused-key rejection, cross-field constraints (e.g. `range_min < range_max`, `bins ≥ 8`). Acceptance: 100% of contracts covered by contract tests. Parallel after P1 freeze: P2, P3, and the upstream-oracle test harness can all proceed concurrently.

**P2 — Diagnostics & debug-dump subsystem.**
Objective: dump writers/readers (binary + JSONL) for all contracts, `StageTiming`, run manifests (config hash + git sha, mirroring `dpcbf_rollout_evaluator` JSON style `[FACT §2.1]`). Rationale: dumps are the regression-fixture format and the headless visualization channel — needed before any algorithm. Thread: caller's. Tests: round-trip fidelity (exact for ints, bit-exact for floats), versioned-schema rejection. Artifacts: fixture format for every later phase.

**P3 — MuJoCo adapter (frames + oracle) and mode-1 integration.**
Objective: `FrameProviderMj` (ring buffer written each physics step), `OracleProviderMj` (wraps `DynamicObstacleManager::Snapshot()` `[FACT]` → `OracleObstacleState`), `ObstacleSourceSelector` with only mode oracle; hook edits in `main.cc`: (a) construct runner-less selector + providers after `BindModel`; (b) replace the inline `DynamicObstacle`→`ObstacleState` conversion in the axis-filter lambda (`main.cc:686-695` `[FACT]`) with `selector.GetObstacleStates(now)`. Justification for the edit: this lambda is the sole consumer; a wrapper cannot interpose without editing the lambda anyway (§16). Data/thread ownership: ring buffer written by physics thread, read by bridge thread (lock-free, seq-counted). Files must-not-edit: `dpcbf/*` (conversion logic moves, does not change). Tests: adapter test comparing selector output vs the pre-change inline conversion on recorded snapshots (exact equality); integration test: run headless sim steps, assert filter inputs identical before/after (bit-exact). Acceptance: oracle-only behavior provably unchanged; this is integration mode 1. Rollback: revert restores inline lambda. Oracle relationship: this phase is the oracle path formalization.

**P4 — Mid-360 ray simulation.**
Objective: `RayPatternGenerator` (parametric rosette: horiz 360°, vert −7°…+52°, cfg points/s & frame rate `[OPEN Q7 exact pattern]`), `RaycasterMj` using `mj_ray` (or `mj_multiRay`) with `bodyexclude = robot root body` and `geomgroup` filtering `[FACT API]`; per-step sampling hook in `PhysicsLoop` (edit #2 region in `main.cc`: one call inside the existing `sim.mtx` scope, adjacent to `dynamic_obstacles.Step` at lines 529/571 `[FACT]`); accumulation into `TimedPointCloud` frames (default 0.1 s); SPSC queue to (future) perception thread; `lidar_bench` app + analytic scenes (planes, boxes, cylinders at known poses). Ownership: raycaster owns pattern + scratch; queue owned by runner (introduced here in stub form). Thread: physics thread produces; consumer stub drains to dumps. Config additions: `sensor.*` (§14). Tests: unit (pattern statistics: FOV coverage, density histogram); simulation tests vs analytic expected distances (tolerance ≤ 1e-6 m — `mj_ray` is exact vs analytic geoms `[INFERENCE]`); self-hit rate = 0 on robot-only scene; timing: scan period exactly N·timestep. Metrics: §12 sensor block. Visualization: dump → replay app renders nothing yet (headless dumps only). Acceptance: dumped clouds match analytic scenes; physics-loop overhead measured and < budget (cfg rays/step). Parallel: P5-P9 core algorithms can develop against synthetic fixtures without this; P4 only gates simulation validation.

**P5 — Motion compensation: deskew + gravity alignment.**
Objective: pose interpolation over `FrameTransformSnapshot`s; per-point re-projection to scan-end time; roll/pitch removal. Inputs: `TimedPointCloud` + snapshot ring. Outputs: `DeskewedPointCloud`, `GravityAlignedCloud`. Thread: perception (still test-driven here). Tests: unit — synthetic constant-velocity/yaw trajectories with analytically distorted clouds; static scene under robot yaw ±2 rad/s must collapse to static (residual < 5 mm); invalid/missing snapshot → frame marked invalid (fault-injection). Metrics: §12 deskew block. Acceptance: residual distortion bounds met; deterministic across runs.

**P6 — Self filter + ground segmentation (fallback path).**
Objective: defensive self filter (config cylinder around base) and `ZBandSegmenter` (z-band + optional plane refine) behind `ISegmenter`. Tests: labeled synthetic scenes (flat floor, ramp ≤ cfg slope, cylinder bases); metrics §12 segmentation block; obstacle-base retention ≥ 99% on synthetic cylinders. Note: TRAVEL deliberately deferred (P14, §22).

**P7 — Scan projection (p2l-equivalent) + equivalence tests.**
Objective: `ScanProjector` per §7 `ProjectedScan` spec. Tests: (a) analytic bins for hand-placed points (exact); (b) equivalence vs verbatim re-implementation of upstream loop (`pointcloud_to_laserscan_node.cpp:155-228` semantics `[FACT]`) — exact equality; (c) optional live-node comparison job when ROS2 present `[OPEN Q-ROS]`; property tests: nearest-return, empty-bin policy, boundary angles ±π. Config: `projection.*`. Artifacts: golden `ProjectedScan` fixtures for P8.

**P8 — Detection backend (`obstacle_extractor` port).**
Objective: `SegmentCircleDetector` implementing grouping → split-merge → segment fit → circle-from-segment (√3/3 + enlargement) → merges, faithful to audited semantics (§4.1), Armadillo→Eigen. Interfaces: `IDetector2D::Detect(const ProjectedScan&)` → `{FittedPrimitive2D[], CircleObservation[]}`. Optional dep: upstream-oracle test target (Armadillo) compiling `obstacle_detector/include/obstacle_detector/utilities/*` `[FACT ROS-free]` to cross-check on identical inputs. Tests: unit per sub-stage; regression vs upstream (segments: endpoint tolerance ≤ 1e-9 where both use exact LS — `[OPEN]` pinv vs QR may differ on degenerate clusters → tolerance policy §11); scenario tests: partial arcs, two adjacent cylinders, occlusion visibility flags. Acceptance: upstream-parity within tolerances on the fixture corpus; zero heap allocation in steady state (preallocated pools).

**P9 — Tracking backend (`obstacle_tracker` port, rescheduled).**
Objective: `KfCircleTracker`: per-axis 2-state KFs, (x,y,r) Euclidean-gate association, fusion/fission, tentative→confirmed→coasting lifecycle, measurement-driven dt (design change vs upstream timer model — recorded as deliberate REPLACE). Owns tracking state. Tests: synthetic detection streams (constant velocity, crossing, occlusion gaps, radius jitter); regression vs upstream KF math on identical (dt, measurement) sequences (exact within 1e-12 — same linear algebra); metrics §12 tracking block. Acceptance: velocity RMSE ≤ target on synthetic corpus; ID-switch count ≤ threshold on crossing scenario.

**P10 — Safety-state generation.**
Objective: `SafetyStateGenerator` per §6; explicit staleness (`max_age`), confirmation gates, inflation formula with config k's. Tests: unit (containment property: true circle ⊆ safety circle under sampled error distributions — statistical, §11); fault injection (stale tracks → dropped, not shrunk). Artifacts: `SafetyObstacle` streams for P11.

**P11 — DPCBF adapter + paired evaluator.**
Objective: `SafetyObstacle → dpcbf::ObstacleState` conversion (id preserved for hysteresis in the filter's top-k selection `[INFERENCE from selected_obstacles/priority logic]`); `Evaluator` producing per-frame oracle↔estimate association and JSON metrics (§12 DPCBF block) including duplicate side-effect-free DPCBF evaluation: instantiate a second `DpcbfSafetyFilter` (own OSQP state — safe since the class is self-contained `[FACT pimpl]`) fed by estimated states, commands discarded, deltas logged. Tests: adapter unit tests; evaluator association correctness on synthetic pairs. This delivers integration modes 4-5 machinery without touching the live path.

**P12 — Pipeline orchestration, perception thread, shadow integration (modes 2-3).**
Objective: `PerceptionRunner` — perception thread consuming the P4 queue, running P5-P10 stages, publishing `PerceptionFrame` double buffer; `ObstacleSourceSelector` gains modes `{oracle, shadow, compare}`; hook edit #3 in `main.cc` (extend the P3 selector construction with runner wiring; still the same two files). Thread ownership finalized: physics→(queue)→perception→(double buffer)→bridge/ros2/evaluator. Tests: integration — end-to-end sim runs headless, shadow logs produced, oracle path bit-identical to P3 baseline; determinism test: two runs same seed → identical dumps; latency budget measured (§12 performance). Acceptance: modes 1-4 all selectable via config; dropped-frame accounting correct under artificial load.

**P13 — ROS2 adapter + RViz topic set (optional build).**
Objective: `PERCEPTION_WITH_ROS2` target: publisher node (in-process thread by default `[OPEN Q-ROS in-proc vs external]`, reading double buffer), topics per §13, TF (world→base→sensor), sim-time→ROS-time policy (publish sim time on `/clock` or stamp with sim time directly `[OPEN]`). Tests: adapter tests (contract→msg field mapping); serialization consistency vs dumps. Must not block: all prior phases fully headless-valid without this.

**P14 — TRAVEL adapter (optional).**
Objective: `TravelSegmenter` behind `ISegmenter`, A/B evaluated vs `ZBandSegmenter` on ramp/stair fixtures. Only justified if evaluation scenes stop being flat `[INFERENCE §6]`. Acceptance: segmentation metrics ≥ fallback on rough terrain, ≤ 2× runtime.

**P15 — Closed-loop with fallback (modes 5-7).**
Objective: selector gains `estimated_with_fallback`; fallback policy per §16; scenario-gated activation (config allowlist of evaluation scenarios). Tests: end-to-end campaigns (§11/§12): paired oracle vs estimated runs, collision count, clearance deltas, QP feasibility, fallback trigger counts; fault injection (perception thread stalled → fallback within `max_age`). Acceptance criteria are the §12 DPCBF thresholds — this phase is gated on evidence, not schedule.

**P16 — Hardening & promotion (modes 8-9).**
Objective: performance passes (allocation audit, per-stage budgets), documentation, config default flips to estimated-primary with oracle permanently selectable; reference baselines retained. Rollback: config-level (mode flag), not code-level.

---

## 11. Testing roadmap

**Layers → where they live and what gates what:**

- **Unit** (`tests/unit`, no MuJoCo/ROS): geometry, deskew math, projector, each detector sub-stage, KF, safety inflation. Every P≥5 phase ships with its own.
- **Contract** (`tests/contract`): §7 validity rules, dump round-trips, config schema (P1-P2).
- **Adapter** (`tests/adapter`): MuJoCo adapters vs analytic scenes (P3/P4); dpcbf adapter field mapping (P11); ROS2 msg mapping (P13).
- **Integration** (`tests/integration`): queue/threading, determinism (same seed → identical dumps), mode-switch equivalence (P12).
- **Simulation validation** (`tests/…/simulation` + `lidar_bench`, needs MuJoCo): analytic ray scenes — walls, flat ground, ramps, stair discontinuity, single/multiple/partially-visible upright cylinders, overlapping angular clusters, occlusion, crossing/approaching/receding movers, robot yaw / translation / combined motion (P4-P12).
- **Regression** (`tests/regression`): golden-dump replays through `perception_replay`; upstream-oracle comparisons (`obstacle_detector` core via Armadillo target; p2l via verbatim reimplementation; optional live ROS2 node job).
- **Ablation:** segmenter fallback vs TRAVEL; fitted-circle vs conservative-enclosing feed; bin-count sweep; deskew on/off (quantifies Q-deskew).
- **End-to-end DPCBF:** paired campaigns (P15) reusing the `dpcbf_rollout_evaluator` scenario style `[FACT]`.
- **Performance:** per-stage timing asserts, allocation counters (custom counting allocator in tests), physics-loop overhead budget.
- **Fault injection:** timestamp jitter, scan latency, dropped scans, invalid transforms, self-hit contamination (disable source exclusion), forced ground-seg failure, forced cluster split/merge failure, degenerate circle fits, injected track swaps, velocity spikes, stale tracks, synthetic FP/FN streams.

**Tolerance policy** (as required):
- Exact equality — p2l equivalence, oracle-mode behavior preservation (P3/P12), dump round-trips.
- Numerical tolerance — ported vs upstream LS/KF (≤1e-9/1e-12 nominal; degenerate-input divergences documented case-by-case).
- Statistical tolerance — noise-driven metrics (fit error, velocity RMSE) over fixture corpora with fixed seeds.
- Behavioral equivalence — track lifecycle events, occlusion flags (same decisions, not same floats).
- Safety dominance — `SafetyObstacle` must contain the true circle with ≥ configured probability; radius may over-estimate freely within a bound, never under-estimate beyond tolerance.

---

## 12. Quantitative validation roadmap

Per stage (metric → target `[RECOMMENDATION, initial; refined by P4-P9 evidence]` → phase):

- **Sensor/rays:** direction error = 0 (deterministic); hit-distance error ≤ 1e-6 m vs analytic; missed-hit rate ≤ cfg; self-hit rate = 0 with exclusion on; scan period error = 0 steps; timestamp error = 0 (P4).
- **Deskew:** residual distortion ≤ 5 mm @ 2 rad/s yaw; static-scene consistency ≤ 5 mm under combined motion (P5).
- **Gravity alignment:** residual ground tilt ≤ 0.1° (GT quaternion in sim) (P5).
- **Self filter:** robot-point rejection ≥ 99.9%; environment false-rejection ≤ 0.1% (P6).
- **Ground segmentation:** ground precision/recall ≥ 0.98/0.98 flat; non-ground recall ≥ 0.98; obstacle-base retention ≥ 0.99; runtime ≤ 2 ms/frame (P6/P14).
- **Projection:** bin correctness exact; range error ≤ 1e-6; information-loss stats reported (points/bin histogram); p2l compatibility exact (P7).
- **Clustering/detection:** cluster precision/recall ≥ 0.95 on corpus; over/under-segmentation rates reported; purity ≥ 0.95 (P8).
- **Circle fitting:** center error ≤ 0.05 m full-arc, ≤ 0.15 m half-arc (short-arc bias explicitly measured — the √3/3 rule biases with arc length `[INFERENCE]`, this is the key P8 experiment feeding Q-fit); radius error ≤ 0.05/0.1 m; distance-to-center error ≤ 0.05 m; failure rate ≤ 1% (P8).
- **Tracking:** velocity RMSE ≤ 0.1 m/s @ obstacle speeds ≤ 0.8 m/s `[FACT range from config]`; position RMSE ≤ 0.05 m; ID switches ≤ 1 per crossing pair; confirmation delay ≤ 3 scans (0.3 s); deletion delay ≤ `tracking_duration`; stale-output rate = 0 (P9).
- **Safety generation:** conservative containment ≥ 99.9%; radius under-estimation rate ≤ 0.1%; position/velocity uncertainty coverage calibrated (χ² test); latency inflation correctness (analytic) (P10).
- **DPCBF end-to-end:** command-delta RMSE vs oracle ≤ 0.1 m/s; min-clearance difference ≥ −0.05 m (estimated never less safe by more than 5 cm); collision count = 0 across campaign; intervention-rate difference ≤ 20%; QP feasibility ≥ oracle's; fallback frequency ≤ 1% frames; end-to-end latency (physics stamp → filter consumption) ≤ 150 ms (one scan + processing) (P11/P15).
- **Performance:** per-stage CPU budgets (raycast ≤ 20% of physics step budget; pipeline total ≤ 50 ms/frame on target machine); steady-state allocations = 0; memory growth = 0 over 1 h; throughput = scan rate; dropped frames ≤ 0.1%; ROS2 publishing overhead measured ON vs OFF (P12/P16).

Comparison also against the training-time noise envelope (σ_pos 0.03, σ_vel 0.05, σ_r 0.015, dropout 3%, latency 1 high-level step `[FACT §2.4]`): if the real pipeline's error statistics exceed what the RL policy was trained to tolerate, that is a release-blocking finding for closed-loop mode `[RECOMMENDATION]`.

---

## 13. Visualization and ROS2 roadmap

The Core→optional-ROS2-adapter→RViz2 architecture is appropriate and confirmed `[RECOMMENDATION]`; the existing OpenCV `DpcbfVisualizer` remains untouched as the DPCBF-level view `[FACT it consumes copies only]`. Every stage is also dumpable headlessly (P2), so RViz is never load-bearing.

**Amendment (§1.1):** a native 3D overlay inside the vendored MuJoCo viewer (ray lines / hit markers drawn directly in the sim scene) was attempted first during the P4 bring-up and found to be a dead end without patching third-party code: the vendored `simulate.cc` only appends user-scene geoms (`user_scn`) to the render scene in **passive mode**, and this app constructs `Simulate` with `is_passive_ = false`, so any geoms an adapter appends are silently discarded (`simulate.cc:2861`). The fix is a one-line move of that append call out of the passive-only branch, but that is vendored MuJoCo application code, outside the sanctioned edit set, and was deliberately not touched. This constraint applies to any future in-viewer 3D overlay work (clusters, tracks, safety circles), not just the raw-point bring-up case — plan around it or budget a reviewed vendored-code patch.

In its place, the production early-phase debug channel is a **live OpenCV window** (`adapters/opencv/live_scan_view.cpp`), built following the exact `DpcbfVisualizer` threading pattern (dedicated render thread, `Publish()` hands it copies under a mutex, never touches `mjModel`/`mjData`, never holds `sim.mtx`). Today it renders a standalone top-down + range-image view of raw scan points only — it is **not yet merged into the existing `DpcbfVisualizer` window with ground-truth oracle obstacles drawn in the same frame**, which was the explicit ask for comparing "what DPCBF sees" against "what's actually there." That merge is deferred for two reasons: P0/P4 has no detector yet (only raw points to show, not an estimated obstacle to overlay against a ground-truth cylinder), and merging into `DpcbfVisualizer` itself changes a file the architecture treats as reuse-only (§2.6). The two candidate paths, to be decided before/at P8–P10 (once `CircleObservation`/`SafetyObstacle` exist to actually overlay): (a) add a raw-point layer to `DpcbfVisualizer` now as a cheap interim step, or (b) wait and add the full estimated-vs-oracle overlay once detection lands, skipping an intermediate raw-point-only merge. Not yet decided — track under §19 open questions.

**Topic plan** (all frames = world unless noted; publish rate = scan rate 10 Hz unless noted; none latched except markers with lifetime; all optional; source = `PerceptionFrame`; conversion = `adapters/ros2`; namespace/color conventions per stage; all available headless as dumps):

| Topic | Type | Notes / RViz display |
|---|---|---|
| `/perception/raw_cloud`, `/deskewed_cloud`, `/gravity_aligned_cloud`, `/self_filtered_cloud` | `sensor_msgs/PointCloud2` | frames: sensor/base/gravity-aligned; PointCloud2 display |
| `/perception/ground_cloud`, `/non_ground_cloud` | `PointCloud2` | label-colored |
| `/perception/projected_scan` | `sensor_msgs/LaserScan` | NaN→inf conversion here (`use_inf` semantics `[FACT]`); frame gravity-aligned base; LaserScan display |
| `/perception/clusters`, `/fitted_primitives` | `visualization_msgs/MarkerArray` | split boundaries as spheres, lines as LINE_LIST, circles as CYLINDER; visible endpoints green / occluded red |
| `/perception/detections` | MarkerArray + custom-free (no new msg pkg: encode id/conf in marker text) | raw vs associated in separate namespaces |
| `/perception/tracks`, `/track_velocities` | MarkerArray | confirmed/tentative/rejected by namespace+color; velocity arrows scaled 1 s; prediction ghosts; covariance ellipses (SPHERE scaled) |
| `/perception/safety_obstacles` | MarkerArray | fitted radius (solid) + conservative radius (translucent shell) |
| `/perception/oracle_obstacles` | MarkerArray | 1 Hz-lifetime cylinders, distinct color |
| `/perception/estimation_error` | MarkerArray | center/velocity error vectors oracle→estimate |
| `/perception/diagnostics` | `diagnostic_msgs/DiagnosticArray`, 1 Hz | stage timings, drop counts, stale/invalid flags, mode |
| TF | — | world→base→mid360 from `FrameTransformSnapshot` |

Deliberately no custom message package `[RECOMMENDATION]`: avoids third-party/message leakage and keeps the adapter deletable; if a structured estimate topic is later needed, that's a separate decision `[OPEN, non-blocking]`.

---

## 14. Configuration roadmap

Single file `perception/configs/perception.yaml`, loaded once by `integration/config` into an immutable `PerceptionConfig` struct tree; validation with hard errors (matching the dpcbf `LoadConfig` throw-on-invalid idiom `[FACT]`); `schema_version` field; unknown keys rejected; every run's evaluator manifest embeds the resolved config + git sha (mirroring `tune_dpcbf` reproducibility style `[FACT]`). Cross-module constraints validated centrally (e.g. `safety.max_age ≥ sensor.frame_period`). ROS2 parameters: none — the ROS2 adapter reads the same struct; RViz-side tuning is deliberately not runtime-dynamic in v1 `[RECOMMENDATION]`. Experimental vs production: `experimental:` subtree gated by `allow_experimental` flag; deterministic evaluation freezes config by manifest hash.

**Section → owner:**

- `sensor` (pattern, rates, mount pose, timing) → mujoco adapter — **implemented (§1.1) with real, sourced values, not placeholders:** `extrinsic.parent_body: torso_link`, `translation_xyz_m: [0.0002835, 0.00003, 0.428434]`, `rotation_rpy_rad: [π, 0.05112069379091391, 0.0]` (Unitree `mid360_joint`, `rev_1_0`/`mode_*` variant); `sensor.vertical_fov_deg: [-7.0, 52.0]` (sensor frame, unnegated — becomes +4.071°…−54.929° in the robot frame after the roll=π mount, asymmetric-downward as verified by test); `horizontal_fov_deg: 360`, `azimuth_rays: 360`, `elevation_rays: 32` (11520 rays/frame — a uniform-grid approximation of the real non-repetitive rosette, still open per Q3 below), `min_range_m: 0.10`, `scan_rate_hz: 10.0`, `range_noise_std_m: 0.02` (implemented, seeded, default OFF so the P4 accuracy gate stays noise-free), `angular_noise_deg_1sigma: 0.15` (recorded, deliberately unmodelled), `body_size_mm: [65, 65, 60]`. `max_range_m: 40.0` is the one config value with **no citation** in either source document (the manual specifies range by target reflectivity, not one number) — flagged as an open tunable, not a fact. Lives in `perception/configs/perception.yaml` only — deliberately **not** duplicated into any `dpcbf/config/*.yaml` (core must not reach into `dpcbf/`, §8).
- `motion_compensation` (deskew on/off, interpolation) → pipeline
- `frames` (base body name — default `pelvis`, matching dpcbf config `[FACT]`) → frame provider
- `self_filter` → self filter
- `segmentation` (backend id: `zband`|`travel`, params) → segmenter
- `projection` (bins, height band, ranges, missing policy) → projector
- `detection` (all ported extractor params, same names as upstream for cross-tuning: `min_group_points`, `max_group_distance`, `distance_proportion`, `max_split_distance`, `max_merge_separation`/`spread`, `max_circle_radius`, `radius_enlargement`, `circles_from_visibles`) → detector
- `tracking` (KF variances, gate, confirm/delete counts) → tracker
- `safety` (inflation k's, `max_age`, `min_track_age`) → safety stage
- `dpcbf_adapter` (id policy) → adapter
- `mode` (oracle|shadow|compare|estimated_fallback|estimated, fallback thresholds, scenario allowlist) → selector
- `ros2` (enable, topics, rates, debug level) → ros2 adapter
- `dumps`/`metrics`/`profiling` → diagnostics
- `seed` → raycaster/evaluator

Optional-module enablement: build-time CMake options (ROS2, TRAVEL, upstream oracle) × run-time booleans (dumps, comparison, expensive viz, profiling) — both required to be ON for the feature to be active.

---

## 15. Third-party dependency policy

| Question | `obstacle_detector` | `pointcloud_to_laserscan` |
|---|---|---|
| Copied into repo? | Already present untracked. `[RECOMMENDATION]` commit as-is under a `reference/` prefix or add as git submodule pinned to current tree — never into `perception/` | same |
| Submodule / external / linked lib / node? | Not linked, not a node (ROS1 `[FACT]`). Its ROS-free headers compiled only in the optional Armadillo test target | Not linked. Optionally run as a real ROS2 node in validation environments only |
| Reference-only? | Production: yes. Tests: regression oracle | Production: yes. Tests: behavioral reference + optional live baseline |
| License | BSD-3 `[FACT]` — attribution headers preserved in ported files + `third_party_notices.md` | BSD `[FACT]` — same |
| Local modifications | None, ever — both trees are read-only; any needed change happens in the port | same |
| Upstream updates | Pin (submodule sha or vendored snapshot); updates are explicit commits re-running the regression suites | same |
| Interface protection | Repo-owned contracts (§7) are canonical; no upstream type crosses `perception/interfaces` | `sensor_msgs/LaserScan` appears only inside `adapters/ros2` |
| Removal | delete directory + the two optional test/CI targets | same |
| Output comparison | regression tests §11 item 6 | equivalence tests §11 |

`sensor_msgs/LaserScan`, upstream messages, and third-party classes are explicitly barred from being the canonical internal model — `ProjectedScan`/`PerceptionObstacle` are (§7) `[RECOMMENDATION, as required]`.

---

## 16. Oracle and estimated-path integration roadmap

**Minimal integration hooks (complete list, with justification):**

1. `simulate/CMakeLists.txt` — guarded `add_subdirectory(perception)` + `target_link_libraries(unitree_mujoco … perception_integration)`. Unavoidable: CMake has no injection mechanism from a subdirectory upward.
2. `simulate/src/main.cc` — small marked regions (`PERCEPTION HOOK n/4`, all inside `#ifdef PERCEPTION_ENABLED`), introduced starting P4 and extended through P3/P12: facade construction and `RebindPerception()` after model bind; one `MaybeScan` call inside the physics loop's existing lock scope (adjacent to `dynamic_obstacles.Step`); the extrinsic guard at all three model-load sites; and, from P3 onward, the axis-filter lambda sourcing obstacles from `ObstacleSourceSelector` instead of inline conversion. Unavoidable: the lambda body is the sole DPCBF input site and lambdas can't be interposed externally; the physics loop is the only place a consistent `mjData` + `sim.mtx` exist.
3. `src/assets/robots/unitree_g1/xmls/scene_g1.xml` — **added in the P4 bring-up slice (§1.1), not originally planned.** Two edits: the `lidar_mid360` `<site>` (pose has no runtime-settable equivalent — MuJoCo sites are authored in MJCF, there is no API to add one to a compiled model) and `group="2"` on the two `head_link` geoms (the aperture fix is a geom attribute; a CSG hole or a second perception-only mesh were considered and rejected — see the sensor-model source doc's own alternatives analysis). Proven dynamics-neutral by a dedicated test (V0: identical `nbody`/`ngeom`/`nq`/`nv`, zero mass/inertia delta, zero contype/conaffinity delta between the edited and pre-edit compiled models). `src/` was originally on the must-not-edit list; this is a deliberate, justified, and narrowly-scoped exception, not a precedent for editing `src/` freely.

No other existing file is edited. `dpcbf/`, `deploy/`, `scripts/`, both reference packages, and the vendored `simulate/mujoco/` tree remain untouched — the non-negotiable oracle deletion-safety is structural, not procedural.

**Mode ladder** (selector-owned, `config mode:`):

1. oracle-only (P3)
2. pipeline-running-not-consumed (P12)
3. shadow (logged) (P12)
4. comparison (both evaluated; duplicate side-effect-free `DpcbfSafetyFilter` instance per P11 — safe because each instance owns its OSQP state `[FACT pimpl]`)
5. estimated drives the duplicate only
6. estimated drives live filter with automatic oracle fallback
7. estimated in allow-listed scenarios
8. estimated default
9. oracle permanently selectable

**Fallback semantics** `[RECOMMENDATION]`: per-frame, not per-obstacle — mixing oracle and estimated obstacles in one QP creates untestable hybrids; a frame falls back when `(now − PerceptionFrame.stamp > max_age) ∨ (frame flagged invalid) ∨ (pipeline heartbeat missed)`. Stale/invalid definition lives in safety/selector config. Synchronization: both paths stamped in sim time; paired deterministic runs = same seed + config manifest; oracle JSON/metrics in mode 1 remain byte-identical (P3 acceptance test).

---

## 17. DPCBF integration roadmap

- **Contract:** emit `dpcbf::ObstacleState` exactly (world XY, m, m/s, stable ids) `[FACT interface]`. Stable ids matter because the filter selects top-k constraints and the visualizer colors by constraint `[FACT]`; track ids map 1:1.
- **Radius policy:** feed conservative safety radius (P10) — the filter already applies its own `s=1.05` safety factor and `r_rob` `[FACT config]`, but it assumes exact radius; under-estimated radius is the one unrecoverable error (Q-fit experiment quantifies the needed margin) `[RECOMMENDATION: conservative enclosing over fitted, revisit with P8 bias data]`.
- **Velocity:** tracker KF rate states (upstream-equivalent `[FACT §4.1]`) with spike clamping in safety (fault-injection tested).
- **Threading:** bridge thread reads the `PerceptionFrame` double buffer (wait-free); `Filter()` untouched. The pre-existing potential race on `m`/`d` reads (§2.2) is out of scope but documented; perception does not add reads on the bridge thread.
- **Rates:** scans at 10 Hz vs filter at joystick-callback rate — the selector serves the latest valid frame with its timestamp; DPCBF sees piecewise-constant obstacle states, exactly as the oracle path already behaves between snapshots `[INFERENCE]`.
- **Evaluation:** P11 evaluator + P15 campaigns produce the §12 DPCBF metrics; promotion gates on them.

---

## 18. Risks and mitigations

Format: likelihood/impact → detection → mitigation → fallback → resolve-by-phase.

| # | Risk | L/I | Detection | Mitigation | Fallback | Phase |
|---|---|---|---|---|---|---|
| R1 | ROS2 coupling creep into core | M/H | CMake link-set review; core builds w/o rclcpp in CI matrix | §8 forbidden deps; contracts-only adapter | delete adapter | P0-P13 |
| R2 | Third-party message leakage | M/M | grep gate on `sensor_msgs`/`obstacle_detector` includes outside adapters/tests | §15 policy | re-port offending code | continuous |
| R3 | `obstacle_detector` assumptions vs Mid-360 (non-uniform sampling) | H/H | P8 fixture failures | uniform-grid projector before detector (§4.2) | tune `distance_proportion`; disable `circles_from_visibles` (upstream-documented workaround `[FACT]`) | P7/P8 |
| R4 | Ordered-scan construction from rosette loses info / aliases | H/M | projection info-loss metrics | bin-count sweep ablation; height-band tuning | multi-band scans (deferred) | P7 |
| R5 | Motion distortion before projection | M/H | deskew-off ablation vs oracle centers | P5 deskew; scan window ≤ 100 ms | shorter windows | P5 |
| R6 | Ground points contaminating bins | M/H | segmentation metrics; bin-level ground-label audit | z-band + height-band double gate | raise `min_height` | P6/P7 |
| R7 | Self-hit contamination | L/M | self-hit rate metric (bodyid retained) | `bodyexclude` at source + defensive filter | widen self cylinder | P4/P6 |
| R8 | Partial-visibility center drift & short-arc radius bias (√3/3 rule) | H/H | P8 bias experiment | conservative enclosing radius in safety stage; visibility flags damp updates | inflate `k_σ` | P8/P10 |
| R9 | Cluster merge/split instability, occlusion track swaps | M/M | ID-switch & fragmentation metrics | ported fusion/fission + gating; scenario tests | shorter confirm, longer coast | P8/P9 |
| R10 | Velocity from differentiated noisy centers | H/H | velocity RMSE vs oracle | KF rate estimation (not finite diff); measurement σ from fit residual; spike clamp | fallback mode gate on velocity quality | P9/P10 |
| R11 | Tracker-state/timing model mismatch (upstream timer-driven) | M/M | regression divergences | deliberate REPLACE with measurement-driven dt (P9) | n/a | P9 |
| R12 | Physics/LiDAR/policy/DPCBF timing mismatch; stale data | M/H | staleness counters; fault injection | single sim-time base; `max_age` gate; per-frame fallback | oracle fallback | P10/P12/P15 |
| R13 | Frame inconsistency (yaw-aligned vs world) | M/H | static-scene world-frame consistency test | frame-tagged contracts; single transform owner | — | P5-P8 |
| R14 | Hidden per-frame allocations / raycast cost in physics loop | M/M | alloc counters; physics-step budget metric | preallocation; rays spread across steps | lower point rate | P4/P16 |
| R15 | Optional-dep build failures (Armadillo/PCL/rclcpp absent) | H/L | CI matrix all-options-OFF default | options OFF by default; feature tests | skip targets | P0 |
| R16 | Copied third-party divergence | L/M | pinned snapshot + regression suite on update | §15 no-modification rule | re-pin | continuous |
| R17 | Excessive edits to existing files / oracle removal | L/H | diff gate: only 2 files may change; P3/P12 bit-exact oracle tests | §16 hook discipline | revert | every phase |
| R18 | Data race on `m`/`d` between physics and bridge threads (pre-existing) | M/M | TSAN run in P12 | perception reads only under lock or via snapshots | document; propose upstream fix separately | P12 |

---

## 19. Open questions

**Blocking before Phase 1:** none technical; owner decisions: (Q1) commit the two reference packages as-is vs submodules (§15); (Q2) confirm `perception/` top-level placement (§9).

**Blocking before algorithm implementation (P5-P9):** (Q3) exact Mid-360 pattern fidelity — parametric rosette vs sampled real pattern CSV; affects P4 config only, pipeline is pattern-agnostic `[OPEN—experiment]`. **Status (§1.1): still open.** The shipped bring-up uses a uniform 360×32 azimuth/elevation grid (11520 rays/frame) rather than the real ~20000 pts/frame non-repetitive rosette; confirmed pattern-agnostic downstream (`GenerateUniformGrid` is the only place fidelity would change). One new dependency: the shipped bring-up assigns one timestamp per azimuth column (a rotating-multi-beam analogue) for deskew's `time_offset_s`/`ray_index` — resolve Q3 before P5 if rosette fidelity will matter, since deskew calibration against the current pattern would otherwise need redoing; (Q4) scan window: 100 ms frames vs shorter sub-frames for latency; ablation in P7/P12; (Q5) single height-band vs multi-band reduction into one scan (start single; revisit with R4 data); (Q6) nearest-return per bin confirmed adequate? (default yes, matching upstream `[FACT]`; ablation P7); (Q7) how deskew interacts with binning when window is long (deskew-first is the design; verify residuals P5).

**Blocking before ROS2 integration (P13):** (Q8) ROS2 distro availability and workspace arrangement on the dev machine `[OPEN—inspection: no ROS2 found in repo; environment unknown]`; (Q9) in-process publisher thread vs separate executable reading dumps; (Q10) sim-time policy (`/clock` vs direct sim-stamps).

**Blocking before closed-loop (P15):** (Q11) fitted vs conservative enclosing circle feed (P8/P10 evidence); (Q12) inflation calibration `k_σ`, latency term (P10 coverage tests); (Q13) unknown-radius handling when arcs stay short (candidate: radius floor at config `radius_range` min 0.2 m `[FACT config]` — owner decision); (Q14) is KF-rate velocity adequate for DPCBF at 10 Hz scans (P11 command-delta data); (Q15) multi-obstacle association validation at count=90 `[FACT, dpcbf/config/dpcbf_config.yaml — corrected from an earlier "count=20" mis-citation during P1]` densities.

**Non-blocking research:** (Q16) TRAVEL benefit on non-flat scenes; (Q17) alternative circle fitters (algebraic Taubin fit) vs ported rule; (Q18) feeding perception-derived noise stats back into the RL training noise model (§12); (Q19) tracker upgrade to joint 2D KF with cross-covariance.

**New from the P4 bring-up (§1.1), to resolve before/at P8–P10:** (Q20) **RESOLVED** — merge the live scan view into `DpcbfVisualizer` was deferred to P8–P10, when `CircleObservation`/`SafetyObstacle` exist to overlay against ground truth, rather than shipping a raw-point-only interim merge now (owner decision, 2026-07-30); (Q21) whether to invest in patching the vendored `simulate.cc` passive-mode `user_scn` append limitation (§13) to unlock a native in-viewer 3D overlay later, vs. staying on the OpenCV channel permanently; (Q22) isolate the populated-arena raycast cost (~50 ms/scan observed with ~166 geoms, contention-polluted, not a clean measurement) from physics-thread contention — needed before trusting any O6-style budget number in a real scenario; (Q23, mirrors extrinsic doc §7.1) is `mid360_link` the optical origin O or the mounting base — still unresolved by any simulation-only test (a front-wall test cannot discriminate, differs by only 2.4 mm in range vs. 47 mm in 3-D); the one hardware test that would settle it (flat-ground height fit: 1.2654 m ⇒ O, 1.2185 m ⇒ mounting base) is out of scope for this repo until real hardware is available; (Q24) rigid-attachment/pose-tracking evidence to date (V5, V6) never exercises an actual walking-policy trajectory, only static poses and a synthetic kinematic sweep — revisit if the deployed ONNX velocity policy's specific gait dynamics ever need to be validated against the sensor path; (Q25) **re-scoped at P2:** add keyboard left/right steering as a workaround for this sandbox's pre-existing no-joystick crash (`use_joystick: 1` is now correctly the tracked default; the crash is a real, perception-unrelated bug that a keyboard input path would sidestep for interactive local driving) — still backlogged, not yet scoped as its own phase task; (Q26) the one-physics-step scan/kinematics lag (~4.7 mm at fall speed, §1.1 follow-up) is deliberately unfixed pending P5 deskew — confirm at P5 that the deskew design actually subsumes it rather than assuming so.

**Also resolved-by-audit** (formerly open): `obstacle_detector` separates cleanly from ROS (yes, minus Armadillo `[FACT]`); its tracker reuse (port with rescheduling, §4.2); original p2l behavior appropriateness (yes, post-deskew with height band, §5); internal projector as canonical path (yes, §5.2).

---

## 20. Recommended implementation order

Authoritative order = `P0 → P1 → P2 → P3 → {P4 ∥ P5 ∥ P6 ∥ P7 ∥ P8-P9 chain} → P10 → P11 → P12 → {P13 ∥ P14 ∥ P15-prep} → P15 → P16`, mapped to the requested categories: scaffolding (P0), contracts (P1-P2), simulation adapters + oracle comparison (P3), raw sensor validation (P4), motion compensation (P5), segmentation (P6), projection + p2l compatibility (P7), `obstacle_detector` reference integration + repo-owned detection + circle fitting (P8), tracking (P9), safety-state (P10), DPCBF shadow (P11-P12), ROS2 visualization (P13), TRAVEL (P14), closed-loop activation (P15), promotion (P16).

---

## 21. Parallelizable workstreams

Once P1 contracts freeze:

- **(A) MuJoCo sensor track** (P3-P4-P5) — needs MuJoCo, not algorithms.
- **(B) Algorithm track** (P6-P7-P8-P9) — develops entirely on synthetic fixtures + dumps, independent of MuJoCo (as required: `obstacle_detector` behavioral tests independent of ray simulation; p2l equivalence independent of TRAVEL).
- **(C) Infrastructure track** (P2 dumps, replay app, upstream-oracle harness).
- **(D) ROS2 adapter** (P13) any time after P12's frame buffer — visualization never blocks headless validation.

Serialization points: P10 needs A+B outputs; P12 needs A+B+C; P15 needs P11+P12 evidence.

---

## 22. What must not be implemented too early

| Deferred item | Why wait | Evidence required first |
|---|---|---|
| Replacing/removing the oracle path | non-negotiable principle; also the regression baseline | never removed (mode 9) |
| ROS2 in core / rclcpp in default build | destroys headless determinism & deletability | n/a — permanent rule |
| Copying `obstacle_detector` wholesale into production | ROS1+Armadillo dependency for ~800 portable lines | n/a — rejected by audit |
| `sensor_msgs/LaserScan` as canonical type | third-party leakage; HW-migration coupling | n/a |
| One monolithic pipeline class | kills backend swap & per-stage testing | n/a |
| TRAVEL integration | arena is flat `[INFERENCE]`; z-band likely sufficient | P6 metrics failing on target scenes |
| Online parameter tuning / dynamic reconfigure | frozen-config determinism first | stable P15 campaigns |
| Probabilistic/multi-model tracking (IMM, JPDA), elaborate covariance propagation | ported KF may already meet §12 targets | P9/P11 RMSE misses |
| Aggressive perf optimization, zero-copy ROS2, GPU raycast | budgets likely met at 20k pts/frame | P12 timing data violating budgets |
| Real Livox hardware drivers | sim-first; adapter seam already reserved | closed-loop sim success |
| Full RViz dashboard beyond §13 topics | viz must not drive architecture | user need |
| Automatic fallback heuristics beyond staleness/validity | premature cleverness hides estimator faults | P15 fallback-frequency data |
| DPCBF closed-loop activation | safety gate | all §12 DPCBF thresholds green |
| Removing reference backends / intermediate debug outputs | they are the oracles & forensics channel | subsystem declared stable (post-P16) |

---

## 23. Final architectural decision summary

- **Option D/B hybrid:** repository-owned Eigen/STL perception core is the production path; internal scan projector is canonical; original ROS packages are reference + regression baselines only. Option C is factually impossible (ROS1 vs ROS2).
- **`obstacle_detector`:** PORT the ROS-free algorithmic core (grouping, split-merge, fitting, √3/3 circle rule, merging, per-axis-KF tracker) to Eigen behind `IDetector2D`/`ITracker2D`; upstream compiles only in an optional Armadillo test target as regression oracle; tracker scheduling deliberately changed to measurement-driven.
- **`pointcloud_to_laserscan`:** PORT the ~60-line binning core; exact-equality equivalence tests; optional live-node baseline.
- **DPCBF interface unchanged**; `dpcbf/` receives zero edits; validity/staleness/inflation handled upstream in `core/safety` + adapter; conservative radius feed by default pending P8 bias data.
- **Oracle path permanent**, formalized via `ObstacleSourceSelector` with a 9-step mode ladder and per-frame fallback; paired oracle/estimate evaluation built in P11 before any closed-loop use.
- **Exactly three existing files edited** (`simulate/CMakeLists.txt`, `simulate/src/main.cc` — marked hook regions — and `src/assets/robots/unitree_g1/xmls/scene_g1.xml` for the MJCF-only `<site>`/`group="2"` facts, amended in §1.1), everything else new under `perception/`; deleting `perception/` + hooks + the `scene_g1.xml` edits restores today's repository.
- **Deterministic, headless-first:** sim-time everywhere, seeded raycasting, dump-based fixtures; ROS2/RViz strictly optional and stateless.

The document above is grounded in five parallel source audits of `dpcbf/`, `simulate/`+`deploy/`, `src/tasks/navigation/`, `obstacle_detector/`, and `pointcloud_to_laserscan/`; every `[FACT]` carries its file/line origin from those audits. The two highest-leverage findings that shaped the design: the ROS1/ROS2 split between the reference packages (eliminating the external-node architecture), and the realization that a uniform-grid internal projector placed before the detector neutralizes all of `obstacle_detector`'s Livox-hostile assumptions — making a faithful port both safe and cheap.
