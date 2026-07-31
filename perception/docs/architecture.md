# DPCBF perception subsystem — module boundaries

Frozen at Phase 0. This is the working reference for the code in this directory; the
full analysis, option trade-offs and phase roadmap live in the repository-root
`DPCBF_Perception_Subsystem_Architecture.md` (§6–§10 and §22 in particular).

**Status:** Phases 0–3 complete, plus a narrow Phase-4 vertical slice — Mid-360 ray
simulation and validation at the official extrinsic. P3 formalized the ground-truth obstacle
path: the inline `DynamicObstacle → dpcbf::ObstacleState` conversion has MOVED out of
`main.cc`'s axis-filter lambda into `OracleProviderMj` + `adapters/dpcbf` +
`ObstacleSourceSelector` **without changing** — proven bit-for-bit, through the real
`DpcbfSafetyFilter`, against fixtures captured before the new code existed. Everything from
deskew onwards is still directory structure and a frozen contract, not code.

---

## 1. What exists today

```
perception/
  include/perception/
    core/contracts/     validation.h                (Validate()/IsValid() convention)
                        frames.h, ray_pattern.h, timed_point_cloud.h,
                        frame_transform_snapshot.h  (the bring-up five)
                        point_clouds.h              (DeskewedPointCloud,
                                                     GravityAlignedCloud)
                        labeled_point_cloud.h       (PointLabel, GroundPlane,
                                                     GroundSegmentationResult,
                                                     LabeledPointCloud)
                        projected_scan.h            (ProjectionStats, ProjectedScan)
                        detection_2d.h              (Cluster2D, FittedPrimitive2D,
                                                     CircleObservation)
                        tracking_2d.h               (TrackState2D, TrackPrediction2D)
                        obstacles.h                 (PerceptionObstacle, SafetyObstacle,
                                                     OracleObstacleState)
                        diagnostics.h               (PipelineStage, StageTiming,
                                                     MatchedPairError,
                                                     EstimationDiagnostics)
                        perception_frame.h          (PerceptionFrame + capacities)
    core/diagnostics/   dump_format.h               (versions, RecordType, header/footer,
                                                     RetainedStages bitmask, provenance)
                        byte_io.h                   (fixed-width little-endian primitives)
                        contract_fields.h           (THE field list, one per contract)
                        binary_codec.h, json_codec.h, json_value.h
                        dump_file.h                 (DumpWriter / DumpReader)
                        run_manifest.h, replay_report.h, config_hash.h
    core/geometry/      rpy.h                       (rotation conventions)
    interfaces/         i_raycaster.h
    integration/        perception_config.h, mid360_bringup.h, dump_recorder.h,
                        obstacle_source_selector.h  (what DPCBF is fed; oracle mode only)
    adapters/mujoco/    ray_pattern_generator.h, frame_provider_mj.h,
                        raycaster_mj.h, extrinsic_guard.h, live_scan_overlay.h,
                        oracle_provider_mj.h        (Snapshot() + robot pose ->
                                                     OracleObstacleState)
    adapters/dpcbf/     oracle_to_dpcbf.h           (the ONLY module that names
                                                     dpcbf::ObstacleState)
    adapters/opencv/    live_scan_view.h            (the live window)
    core/projection/    scan_projector.h            (the ported p2l binning core;
                                                     LabeledPointCloud -> ProjectedScan)
    core/{pipeline,segmentation,detection,tracking,safety}/
                        reserved, empty — the CONTRACTS are frozen, the STAGES are not
  src/                  mirrors the include tree
    adapters/{ros2,travel}/                         reserved, empty
  apps/                 lidar_bench.cc              (headless raycast harness)
                        perception_replay.cc        (headless dump inspection; no MuJoCo)
  cmake/                GenerateBuildInfo.cmake     (build-time git provenance)
  tests/unit/           geometry_rpy_test.cpp       (no MuJoCo)
  tests/contract/       check.h, contracts_test.cpp, config_test.cpp
                                                    (no MuJoCo; contracts_test links
                                                     perception_contracts ONLY)
                        dump_roundtrip_test.cpp     (no MuJoCo; links
                                                     perception_diagnostics ONLY)
  tests/adapter/        mid360_bringup_test.cpp     (V0–V5 + T1, needs MuJoCo)
                        mid360_pose_tracking_test.cpp (V6, needs MuJoCo)
                        dump_bringup_test.cpp       (real-data dump -> perception_replay,
                                                     needs MuJoCo)
                        oracle_equivalence_test.cpp (P3 acceptance: oracle path bit-exact
                                                     through the real DpcbfSafetyFilter)
                        oracle_golden_run.h         (the one definition of the golden run)
                        oracle_golden_capture.cc    (the PRE-CHANGE tap that produced the
                                                     committed fixtures; not a test)
  tests/fixtures/       synthetic_contracts.h       (hand-authored fixtures for all 24
                                                     dumpable record types)
                        oracle_golden/              (committed pre-change golden: 2700
                                                     OracleObstacleState dump records +
                                                     the exact dpcbf::ObstacleState log)
  tests/{integration,regression}/                   reserved, empty
  configs/              perception.yaml             (the FULL typed tree, all sections)
  docs/                 this file
  third_party_notices.md
```

### Contracts and config: frozen, not implemented

Every section-7 contract and every section-14 config section now exists, is validated, and
is covered by `tests/contract/`. None of the pipeline STAGES that will consume them exists.
That split is deliberate: a later phase adds a stage against an already-frozen type and an
already-typed config field, instead of amending the schema and the contract set on every
commit. Config sections whose module is missing (`segmentation`, `detection`, `tracking`,
`dpcbf_adapter`, `mode`, `ros2`, `metrics`, `profiling`) carry documented defaults and are
cross-validated against the sections that do exist. `dumps` is no longer one of them: it now
drives the real dump subsystem described in §5a.

Validation convention: contracts added here expose `const char* Validate()` (nullptr when
well-formed) plus `IsValid()`. `RayPattern::IsConsistent` and `ScanStats::IsBalanced`
predate it and keep their names. Every fraction-valued accessor states its DENOMINATOR in
the comment beside it, and `contracts_test.cpp` asserts the denominator rather than only
the value — the "self-hit fraction" ambiguity is not allowed to recur silently.

## 2. Build targets — the link sets *are* the dependency enforcement

| Target | Kind | May depend on | Must never depend on |
|---|---|---|---|
| `perception_contracts` | INTERFACE | Eigen, STL | MuJoCo, ROS2, dpcbf, yaml-cpp |
| `perception_core` | STATIC | `perception_contracts` | MuJoCo, ROS2, dpcbf, yaml-cpp |
| `perception_diagnostics` | STATIC | `perception_contracts` **only** | MuJoCo, ROS2, dpcbf, yaml-cpp |
| `perception_config` | STATIC | `perception_contracts`, `perception_diagnostics`, yaml-cpp | MuJoCo, ROS2, dpcbf |
| `perception_dpcbf_headers` | INTERFACE | dpcbf's include path, no link | everything else |
| `perception_dpcbf_adapter` | STATIC | `perception_contracts`, `perception_dpcbf_headers` | **MuJoCo**, ROS2, yaml-cpp |
| `perception_mujoco` | STATIC | core, config, **MuJoCo**, `perception_dpcbf_headers`, `dpcbf_dynamic_obstacles` | ROS2, `dpcbf::ObstacleState` |
| `perception_opencv_view` | STATIC | contracts, config, **OpenCV** | MuJoCo, ROS2, dpcbf |
| `perception_integration` | STATIC | `perception_mujoco`, `perception_dpcbf_adapter`, `perception_opencv_view` | ROS2 |
| `perception_lidar_bench` | EXE | `perception_integration` | — |
| `perception_replay` | EXE | `perception_diagnostics` **only** | MuJoCo, ROS2, dpcbf, yaml-cpp |
| `perception_contracts_test` | EXE | `perception_contracts` **only** | everything else |
| `perception_config_test` | EXE | `perception_config` | MuJoCo, ROS2, dpcbf |
| `perception_dump_roundtrip_test` | EXE | `perception_diagnostics` **only** | everything else |
| `perception_dump_bringup_test` | EXE | `perception_integration` | — |
| `perception_oracle_equivalence_test` | EXE | `perception_integration`, `dpcbf_safety_filter` | — |
| `perception_oracle_golden_capture` | EXE (not a test) | `perception_diagnostics`, `dpcbf_dynamic_obstacles` | — |

**The one rule a link set cannot enforce.** `perception_dpcbf_headers` grants an include path,
and an include path cannot tell `dpcbf/dynamic_obstacles.h` from `dpcbf/dpcbf_safety_filter.h`.
The split is therefore stated at the target that grants it and reviewed:
`perception_mujoco` may include **only** `dynamic_obstacles.h` (for `OracleProviderMj`, the one
sanctioned MuJoCo-plus-dpcbf module) and must never name `dpcbf::ObstacleState`;
`perception_dpcbf_adapter` may include **only** `dpcbf_safety_filter.h` and must never include a
MuJoCo header. Everything else — contracts, core, diagnostics, config — deliberately does not
link `perception_dpcbf_headers` at all, which is what keeps "core must not depend on dpcbf" a
build-time fact rather than an intention. `perception_dpcbf_adapter` sits outside the MuJoCo
block and is built and tested in the no-MuJoCo configuration, which is what proves its half.

`perception_contracts_test` links `perception_contracts` and nothing else on purpose: if a
contract ever grows a MuJoCo, ROS2, dpcbf or yaml-cpp dependency, that target stops linking
and the dependency rule fails at build time rather than at review time. `perception_replay`
and `perception_dump_roundtrip_test` do the same job for `core/diagnostics`, which is the one
part of `core/` that does file I/O and must still reach nothing but the contracts and the
standard library.

`perception_opencv_view` is guarded by `find_package(OpenCV QUIET ...)`, exactly as
`dpcbf_visualizer` is, and carries `PERCEPTION_HAVE_OPENCV=1`. Without OpenCV everything
else still builds and the live window is simply unavailable.

`perception_integration` carries `PERCEPTION_ENABLED=1` as a PUBLIC compile definition.
That is the switch the application's hook regions key off, so `PERCEPTION_ENABLE=OFF`
removes them at preprocessing time rather than at link time.

### CMake options

| Option | Default | Effect |
|---|---|---|
| `PERCEPTION_ENABLE` | ON | Build the subsystem at all. OFF makes `perception/CMakeLists.txt` return immediately. |
| `PERCEPTION_WITH_ROS2` | OFF | Reserved for the optional ROS2 visualization adapter. Currently errors if ON. |
| `PERCEPTION_WITH_TRAVEL` | OFF | Reserved for the optional TRAVEL segmenter. Currently errors if ON. |
| `PERCEPTION_BUILD_UPSTREAM_ORACLE` | OFF | Compiles `obstacle_detector`'s ROS-free headers and links Armadillo, building `perception_detection_oracle_test` — the upstream-parity suite for the ported detector. OFF by default because Armadillo is a dependency the rest of the repository does not have. Located via `PERCEPTION_OBSTACLE_DETECTOR_ROOT`, defaulting to `../obstacle_detector`. |
| `PERCEPTION_BUILD_TESTS` | ON | Build and register the CTest targets. |

The two still-reserved options (`PERCEPTION_WITH_ROS2`, `PERCEPTION_WITH_TRAVEL`) fail loudly
rather than silently doing nothing, so nobody concludes a feature is present because the flag
was accepted. `PERCEPTION_BUILD_UPSTREAM_ORACLE` became real in P8 and now adds one CTest
target: 11 tests with it OFF, 12 with it ON.

MuJoCo is located via `PERCEPTION_MUJOCO_ROOT`, defaulting to `../simulate/mujoco`. When
absent, `perception_core`, `perception_diagnostics`, `perception_config`, `perception_replay`
and the four MuJoCo-free tests still build; the adapter, facade, bench, the end-to-end dump
test and the validation suite are skipped. That is a requirement, not a convenience: replaying
a dumped fixture has to work on a machine that cannot run the simulator, or the dumps are not
really the regression format for the later phases.

`perception_build_info` is a build-time (not configure-time) custom target that captures the
git sha, `git describe` and a dirty flag into a generated header for the dump provenance. A sha
captured when CMake last ran names the wrong commit as soon as anyone commits, and a manifest
field naming the wrong commit is worse than one saying `unknown`. The generator rewrites the
header only when its contents change, so it does not force a rebuild every time.

## 3. Who may know what

| Dependency | Only reachable from |
|---|---|
| MuJoCo | `adapters/mujoco/*` |
| ROS2 / rclcpp | `adapters/ros2/*` (optional, unbuilt) |
| `dpcbf::` headers | `adapters/dpcbf/*` (unbuilt) |
| yaml-cpp | `src/integration/perception_config.cpp`, nowhere else |
| TRAVEL / PCL | `adapters/travel/*` (optional, unbuilt) |
| upstream `obstacle_detector` / `pointcloud_to_laserscan` | test targets only, never production |
| Unitree G1 specifics | config defaults and `adapters/mujoco` body/geom/site names |

Forbidden edges, restated because they are load-bearing: `core → MuJoCo`, `core → ROS2`,
`core → dpcbf`, `core → yaml-cpp`, `core → the application`, `dpcbf adapter → MuJoCo`,
`ros2 adapter → owning or mutating perception state`, `oracle provider → estimator`,
`evaluator → feeding the estimator`.

## 4. Integration hooks outside `perception/`

Five regions, all marked `PERCEPTION HOOK n/5` in `simulate/src/main.cc`, plus one guarded
block in `simulate/CMakeLists.txt`:

| # | Where | What | Why it cannot live inside `perception/` |
|---|---|---|---|
| 0 | `main.cc` include block | `#include`s behind `#ifdef PERCEPTION_ENABLED` | — |
| 1 | `main.cc` globals | the facade object, the obstacle-source selector, `RebindPerception()` | needs `mj::Simulate` to hand over `user_scn` |
| 2 | `main.cc` ×3 model-load sites | `RebindPerception(sim)` | the three sites are where `m`/`d` are replaced |
| 3 | `main.cc` physics loop | `MaybeScan(d->time, !sim.run)` | the only place a consistent `mjData` + `sim.mtx` exist together |
| 4 | `main.cc` `main()` | `LoadConfig(...)` + `ObstacleSourceSelector::Bind(...)` | the app owns startup ordering, and binding before any thread starts is what removes the race |
| 5 | `main.cc` axis-filter lambda | `obstacle_states = selector.GetObstacleStates(d->time)` | the lambda body is the sole DPCBF input site and a lambda cannot be interposed from outside |

Hook 5 is an `#ifdef`/`#else` pair whose `#else` arm is the *original* inline conversion text,
unchanged. That is deliberate on two counts: it is what keeps the `PERCEPTION_ENABLE=OFF` binary
byte-identical to a repository that never had perception in it, and it makes the P3 rollback a
deletion of the `#ifdef` arm rather than a rewrite from memory.

A third file outside `perception/` is also edited —
`src/assets/robots/unitree_g1/xmls/scene_g1.xml` — for two things that are *asset* facts
and have no code representation: the `lidar_mid360` site at the official extrinsic, and
the two `head_link` geoms moved to visibility group 2 to model the optical aperture.
`dpcbf/`, `deploy/` and every other tree take **zero** edits.

To remove the subsystem entirely: delete `perception/`, set `PERCEPTION_ENABLE=OFF` (or
drop the guarded block), and optionally revert the two `scene_g1.xml` edits.

## 4a. Live visualization — why the OpenCV window and not the 3D viewer

The MuJoCo viewer *does* expose a user-owned scene (`Simulate::user_scn`), and
`adapters/mujoco/live_scan_overlay.h` fills it with ray lines, hit markers and the sensor
axes. But in MuJoCo **3.3.6** the vendored app only appends those geoms into the render
scene in **passive** mode:

```c++
// simulate/mujoco/simulate/simulate.cc, ~line 2861
if (!is_passive_) {
  Sync();                       // <-- our path: fills user_scn_geoms_, never uses it
} else if (m_passive_ && d_passive_) {
  mjv_updateScene(...);
  // add user geoms to scene
  int nusergeom = user_scn_geoms_.size();
  ...                           // <-- the only append, passive-only
}
```

`simulate/src/main.cc` constructs `Simulate` with `is_passive_ = false`, so the overlay
would build geoms that are silently discarded. `visualization.viewer_overlay` therefore
defaults to **false** and warns loudly if switched on.

The working live view is `adapters/opencv/live_scan_view.h` — a separate window on its own
thread, fed copies of each scan. It shows a top-down XY panel and an azimuth × elevation
range image.

If you want the 3D overlay, the vendored fix is to move the append out of the passive-only
branch so it runs after `Sync()` too. That edits third-party code, so it is deliberately
**not** applied here.

## 5. Threading

| Thread | Touches |
|---|---|
| physics | `RaycasterMj::Scan` and `LiveScanOverlay::Update`, both inside the existing `sim.mtx` scope |
| render | copies `user_scn` geoms under `sim.mtx` (vendored `Simulate::Sync`) |
| bridge | `ObstacleSourceSelector::GetObstacleStates` — **no new `m`/`d` read**, see below |
| main | `ObstacleSourceSelector::Bind`, before either other thread starts |

The pre-existing potential race on `m`/`d` between the physics and bridge threads is
documented in the root architecture doc (§2.2, R18) and is **not** worsened here.

The bridge-thread entry needs care, because it is the one place perception now runs on the
DPCBF control path. It touches `m`/`d` **not at all**: in oracle mode the selector reads only
`DynamicObstacleManager::Snapshot()`, which takes that manager's own mutex and returns a copy —
and it is the *same* `Snapshot()` call the axis-filter lambda already made inline before P3, not
an additional one. The sim time it stamps with is passed in by the caller (`d->time`, already
dereferenced at that site) rather than read again. `ReadRobotGroundTruth` was deliberately left
in `main.cc` instead of being routed through `OracleProviderMj`, precisely so that this phase
adds zero mjData dereferences to this thread; the provider's robot-pose leg exists for the P11
evaluator and is called from the model-load path, not from the filter callback.

There is still no perception thread, no queue and no runner — that is P12. The selector runs
synchronously on whatever thread asks.

## 5a. Debug dumps and headless replay

The dump subsystem is the regression-fixture format for every later phase AND the headless
inspection channel that keeps RViz off the critical path (root architecture doc §13). Both uses
require that a file found on disk months later can say what it is, what produced it, and what
was *not* in it.

**One mechanism, not twenty-four writers.** Each contract declares its fields exactly once, in
a `VisitFields` overload in `core/diagnostics/contract_fields.h`. The four codecs (binary
write/read, JSONL write/read) are visitors over those declarations, so a writer and its reader
cannot drift apart — they walk the same list. Adding a field is a one-line change plus a dump
schema version bump.

**Two schema versions, never merged.** `kDumpSchemaVersion` versions the byte layout of dump
files. `PerceptionConfig::schema_version` versions the YAML key tree. They change for different
reasons — adding a tracking parameter bumps the config schema and leaves every dump readable;
adding a field to `TrackState2D` bumps the dump schema and leaves every YAML valid — so both are
recorded in every file header and in the manifest. Conflating them would make one of the two
numbers a lie the first time either changed alone. A mismatched dump schema version is refused
outright on open, with an error that names both.

**`RetainedStages` is in every header.** An empty stage section has two completely different
meanings, and conflating them turns a broken pipeline into an idle one. The nine retention flags
travel as a bitmask in the file header *and* in the manifest, which lets `perception_replay`
give a three-way verdict per stage:

| Verdict | Meaning |
|---|---|
| `NOT RECORDED` | Not retained and no file: the dump says nothing about this stage. |
| `RECORDED, EMPTY` | Retained, present, zero records: the stage ran and produced nothing. A real finding. |
| `RECORDED` | Retained, present, with records. |
| `INCONSISTENT` | The flag and the files on disk disagree; neither reading is trustworthy. |

**Bit-exactness.** Floats travel as their IEEE-754 bit patterns in binary, and at 17 (double) /
9 (float) significant digits in JSONL — the shortest widths that round-trip every value of those
types exactly. So `-0.0`, denormals and NaN all survive both formats. That matters concretely:
an empty `ProjectedScan` bin *is* a NaN, and a codec that lost it would make a never-measured bin
indistinguishable from a zero-range one. JSONL spells non-finite values as the bare tokens `NaN`
/ `Infinity` / `-Infinity` (Python's `json` spelling), which is recorded in the JSONL header's
`jsonl_number_note` field because it is not standard JSON.

**File layout.** One record type per file, `<record_type>.bin` / `.jsonl`, plus a sibling
`run_manifest.json` and `resolved_config.txt`. Per-type files make the census a directory
listing, let a consumer read only the stage it cares about, and keep `wc -l` meaningful on JSONL.
Every file ends with a footer carrying the record count: a reader that hits end-of-data without
one reports the dump as **truncated** rather than as a complete-but-shorter run, so an
interrupted run cannot leave behind a fixture that looks finished.

**Manifest vs per-frame dump.** The manifest is a sibling index, not a container. Every dump file
also carries the full provenance block in its own header, so a single `.bin` found on its own is
still self-describing — fixtures get copied around individually, and a fixture that cannot say
which config produced it is not a fixture. What the manifest adds is the cross-file view: which
record types the run produced at all, the `frames_seen` / `frames_recorded` pair (so a decimated
dump cannot read as a run that produced fewer scans than it did), and the pointer to the
resolved-config text whose FNV-1a/64 is the `config_hash` in every header. Its JSON style mirrors
`dpcbf_rollout_evaluator`'s own output — flat object, two-space indent, 17 significant digits on
every real — plus `tune_dpcbf`'s seed/resolved-config reproducibility fields and the git identity
that §14 asks for and no existing artefact in this repository records.

The `config_hash` covers the **entire** resolved tree, including the `dumps:`/`metrics:`/
`profiling:` sections. Turning dumps on therefore changes it. That is deliberate: the hash
identifies a run's configuration, not a subset of it judged to be behaviour-affecting, and such a
judgement would itself need versioning. Hashing the resolved tree rather than the YAML bytes
means two files differing only in comments hash the same, and a run using built-in defaults with
no file at all still gets a real hash.

**What is wired to live data today.** The `dumps:` config section gates real dumping of what the
P4 bring-up slice genuinely produces: `ScanStats` and `FrameTransformSnapshot` on every recorded
scan, and the raw `TimedPointCloud` only when `dumps.retain_stage_clouds` is set. The other eight
stages do not exist, so their retention flags stay false and a replay reports them as `NOT
RECORDED` — never as stages that ran and produced nothing. Writing happens on the physics thread
inside the caller's lock scope, like everything else in `Mid360BringUp`, which is why it is off by
default and why the expensive part is separately gated. A write failure does not abort the scan or
throw: the simulator must keep running when a disk fills up, so the first error is remembered and
reported by `FinishDumps`.

`FinishDumps` must be called explicitly; neither `DumpRecorder` nor `DumpWriter` finalises from a
destructor. A manifest written during stack unwinding would describe a run that failed as one that
finished, and a dump directory with no manifest is how an interrupted run announces itself.

**Whole-file replay.** `DumpReader` loads a file into memory at `Open()`. Records are
variable-length and replay is an offline, human-timescale activity, so a streaming reader would
need per-record length prefixes for no present benefit. This is a known forward limitation — a
long campaign's `PerceptionFrame` dump will not fit comfortably — and is the first thing the
later phases will need changed here.

## 6. Deliberately not implemented yet

Deskew, gravity alignment, ground segmentation, the ESTIMATED half of the DPCBF adapter,
every obstacle-source mode above `oracle`, the perception thread and its queues, and the ROS2
topic set. (Tracking landed in P9 and safety-state generation in P10 — see §8 and §9; both are
still called by nothing, for the same reason projection and detection are.) (Dump/replay is
built — §5a. Scan projection landed in P7: `core/projection/scan_projector` is implemented
and proved bin-for-bin identical to upstream `pointcloud_to_laserscan`, but nothing CALLS it
yet — there is no pipeline, and its input `LabeledPointCloud` has no producing segmenter
until P6, so it is exercised only from synthetic fixtures. Detection landed in P8:
`core/detection/{line_fit, segment_circle_detector}` behind `IDetector2D`, likewise called by
nothing — see §7 for what it measured. The oracle half of
`adapters/dpcbf` and mode 1 of the ladder landed in P3;
Sixteen of its twenty-four record types have no producing stage yet and are covered by
hand-authored fixtures in `tests/fixtures/synthetic_contracts.h`; the format is correct for
them now because it is the format their own regression fixtures will be written in.) The root
architecture doc §22 explains why each waits, and what evidence unblocks it. The
`Mid360BringUp` facade is explicitly *not* `PerceptionRunner`, and `ObstacleSourceSelector`
is explicitly *not* a runner either — it has no thread, no queue and no double buffer, and
refuses to start in any mode above `oracle` rather than degrading to one.

**One knob pair that must not be "fixed" by tidying.** `dpcbf_adapter.max_obstacles: 20` and
`drop_invalid: true` are validated config today and are deliberately NOT applied on the oracle
path. They belong to the P11 estimated-path adapter. The shipped `dpcbf_config.yaml` runs 90
obstacles, and capping to 20 was measured to change `DpcbfSafetyFilter`'s output on 30/30
probed samples — `DpcbfSafetyFilter` does its own top-k selection
(`default_num_constraints: 10`) from whatever it is given, so the unfiltered list is the
correct input. `perception_oracle_equivalence_test` asserts the full uncapped count passes
through, and that an obstacle failing `Validate()` is still emitted.

Every one of those now has its INPUT and OUTPUT types and its config section already
frozen, so each is a matter of writing the stage — not of designing the interface it plugs
into. What is still deliberately absent is any logic beyond construction and validation:
no stage reads a `ProjectedScan`, nothing fills a `TrackState2D`, and `PerceptionFrame` is
never published because nothing produces one yet.

## 7. Detection (P8) — the ported `obstacle_extractor`, and what measuring it found

`core/detection/segment_circle_detector` implements `IDetector2D::Detect(const ProjectedScan&)`
→ `{Cluster2D[], FittedPrimitive2D[], CircleObservation[]}`, faithful to the audited
`obstacle_extractor` semantics with Armadillo replaced by `core/detection/line_fit`. It holds
per-frame scratch only, reuses it across calls, and allocates nothing in steady state. Nothing
calls it: there is no pipeline until P12, and its input has no producing projector chain until
P5–P7 are wired, so it is exercised from `tests/fixtures/detection_scenes.h` and from P7's
committed golden `ProjectedScan` corpus.

### The Armadillo→Eigen substitution, and the tolerance policy it settled

The open question was whether `arma::pinv` and a QR-based substitute diverge on degenerate
clusters. Measured over 55 real cluster fits plus 12 constructed degenerate cases
(`perception_detection_oracle_test`, section A), the boundary is **not** rank deficiency —
the port reproduces Armadillo's own truncation rule, so a rank-1 fit agrees to 1e-15. The
boundary is the conditioning of upstream's *endpoint projection*, which divides by
`D = A² + B²`:

| Class | Definition | Worst endpoint delta | Policy |
|---|---|---|---|
| 1. well-posed | ≥3 points, fitted line within 1e3× the data extent of the origin | **1.3e-14 m** | ≤1e-9 asserted. Contains 100% of the fits the pipeline performs. |
| 2. two-point group | exactly 2 points | 3.8e-6 m at cond 1e11 | Bounded by `cond(X)·eps`, not by the solver. Unreachable at `min_group_points: 5`; documented, not tolerated. |
| 3. ill-posed projection | `D → 0`: the fitted line passes through the sensor | 6.7e+17 m | Indeterminate in **both** implementations. Counted by `DetectionStats::ill_conditioned_projections`; the radius cap stops such a segment becoming a circle. |

So the answer is **(a) with a named exception list**: ≤1e-9 is achievable, the Armadillo
oracle is a regression convenience, and the two exceptions are properties of upstream's
formulation that no substitution could have avoided. Comparing *coefficients* instead of
endpoints would have produced a spectacular false alarm — on a near-radial cluster they differ
by 9e+04 while the endpoints agree to 1e-16, because the projection divides the magnitude back
out. End-to-end parity over 21 scans: worst segment endpoint 1.07e-14 m, circle centre
1.00e-14 m, radius 4.4e-16 m, identical counts.

### Four findings the later phases need

1. **The √3/3 circle rule misses the centre gate, and it is the rule, not the data.** Worst
   full-arc centre error **0.115 m** against §12's 0.05 m target; half-arc **0.219 m** against
   0.15 m. The radius gate is met full-arc (0.028 m ≤ 0.05) and missed half-arc (0.161 m ≤ 0.10
   is false). An algebraic circle fit on the *same points* recovers centre and radius to machine
   precision, so the entire gap is the fitting rule. The rule is a circumcircle standing on the
   chord — its centre sits behind the arc by construction — and it was ported unchanged and not
   retuned, as the phase required.
2. **The short-arc radius bias is real and points the unsafe way.** Mean true-radius bias
   −0.006 m full-arc versus **−0.145 m half-arc**: shorter arcs *under*-estimate, which is the
   one unrecoverable direction for a safety radius. `CircleObservation` already carries
   `radius_enclosing_m` alongside `radius_fitted_m` for exactly this choice.
3. **Split-and-merge barely fires on upstream's default path.** With `use_split_and_merge: true`
   the divider search is seeded with the least-squares line, and for an L-shaped cluster that
   line makes the cluster's own *first* point the largest deviation — so `split_index` is 1 and
   the `min_group_points` guard rejects the split. A closed square gives 1 split with the
   least-squares seed and 4 with the chord seed. Wall corners therefore survive as single
   over-long segments, which the radius cap harmlessly rejects, but the stage is much weaker
   than its name suggests.
4. **The ±π seam is a coverage hole directly astern.** The ordered point list runs bin 0 → B−1
   and does not wrap, so an object centred behind the robot arrives as two clusters. Segment
   merging rejoins them when both halves clear `min_group_points` (measured identical to the
   head-on view), and the object vanishes entirely when they do not — a 0.25 m cylinder at 5 m
   astern is undetected while the same cylinder at 5 m ahead is found.

Also measured, and not a defect: at 360 bins a 0.20–0.30 m cylinder falls under
`min_group_points` beyond roughly 4.6–6.9 m, halving with a half-visible arc. That is the
angular-resolution floor of the shipped `projection.bins`, and it bounds the useful detection
range for this obstacle class.

### Two upstream defects, corrected by default

Both were found by transcription, both are reproduced exactly under `UpstreamQuirks::Exact()`
so the oracle comparison is bit-for-bit rather than approximate, and both are corrected in the
shipped default. Their combined effect on the corpus is measured, not argued: 4 of 22 scans
change, worst circle-centre shift **5.2e-2 m**.

* `groupPoints` writes `input_points_.begin()++`, which evaluates to `begin()`, so the first
  point is compared against itself and counted twice. `fitSegment` then reads one point past
  the first group — mixing the next cluster's first point into the fit — and reads past `end()`
  outright when the whole scan is one group.
* a split records its second half as `num_points - split_index`, one short of the span it owns,
  because the cloned divider point is not counted. The half's last point is excluded from the
  fit while still being projected onto as an endpoint.

The port also represents the split's cloned point as a *shared index* between the two halves
rather than by inserting into a list, which is exactly equivalent and removes the aliasing
hazard `Cluster2D` was defined to remove.

### Config

One key was added under `detection`: `discard_converted_segments: true`, upstream's own name and
default. It is a ported extractor parameter — a segment that became an accepted circle leaves
the segment list — and exposing it keeps the port and its oracle tunable from one set of
numbers. `radius_enlargement_m` is deliberately still upstream's 0.25 m; retuning it is a later
decision that finding 2 above is the input to.

The two measurement sigmas (`sigma_center_m`, `sigma_radius_m`) are the only numbers in this
module that were **chosen** rather than ported: a per-point residual reduced by point count and
inflated by the `1/(1 − cos α)` geometric dilution of a centre estimated from a short arc, with
a residual floor and a dilution cap named as constants in the header. They are provisional and
the tracking phase is what calibrates them against measured innovation statistics.

---

## 8. Tracking (P9) — the ported `obstacle_tracker`, rescheduled onto measurement time

`core/tracking/kf_circle_tracker.cpp` behind `interfaces/i_tracker_2d.h`, with the per-axis filter
in `core/tracking/axis_kalman.h`. Three independent 2-state `[value, rate]` Kalman filters per
obstacle (x, y, radius), a Euclidean association gate over (x, y, r), fusion, fission, and a
tentative → confirmed → coasting lifecycle. This is the first stage in the pipeline that owns
state across scans; everything above it is a pure function of one frame, by `IDetector2D`'s
contract.

### Upstream parity, and what it cost

Agreement with upstream's own `utilities/kalman.h` (compiled and linked against Armadillo, not
transcribed) is **exact — worst |Δ| = 0.0e+00** across 200 steps of upstream's own configuration,
150 steps of variable dt/R with dt-scaled Q, and 20 predict-only steps, over the full state and
all four entries of P. The 1e-12 gate is never approached. There was no numerical substitution to
make here, so unlike P8 there was no tolerance *policy* to settle: the same five expressions run
in the same order.

### Three upstream defects, found by transcription

Each is asserted by `tests/regression/tracking_oracle_test.cpp` against the real upstream header,
because each is a fact the port had to make a decision about.

* **D1 — upstream's timer does not coast.** `TrackedObstacle::updateState()` calls `predictState()`
  then `correctState()`, and `KalmanFilter::correctState()` reads `y`, which is written *only* by
  `TrackedObstacle::correctState(const CircleObstacle&)`. So the 100 Hz timer re-applies the **last
  received measurement** at full Kalman gain, roughly ten times per scan. Measured: after 1.0 s of
  ticks a track seeded at 1.0 m/s has advanced **8.6 mm** instead of 1.0 m, and its velocity
  estimate has decayed **1.000 → 0.085 m/s**. Risk R11 records the timer model as a determinism
  problem; this is the stronger reason, and it was not in the register.
* **D2 — `initKF` never initialises P.** Every upstream track begins at `P = eye` from the
  `KalmanFilter` constructor: 1.0 m² of claimed positional variance about a position it has just
  measured, and an arbitrary 1.0 (m/s)² about a velocity it has not measured at all.
* **D3 — the association penalty is dead code.** `obstacleCostFunction` computes a
  direction-rotated Mahalanobis-style penalty and then returns `cost / 1.0`, with
  `// return cost / penalty;` commented out above a TODO. The effective upstream cost already *is*
  a plain Euclidean distance over (dx, dy, dr) — which is what licenses the port's cost function to
  be one. The dead code additionally builds `distribution` with **variances** on its diagonal and
  then uses it as an **information** matrix, so a more uncertain track would score a smaller
  penalty; recorded so that nobody revives it as written.

### Five deliberate replacements

| # | Upstream | Port | Why |
|---|---|---|---|
| R11 | timer-driven update at `loop_rate`, A carrying `1/loop_rate` | measurement-driven dt from consecutive `Update()` stamps | tick count between measurements is a wall-clock scheduling fact, which breaks the determinism the dumps and the same-seed gate require — plus D1 |
| (a) | Q a stored per-tick constant | `Q(dt) = diag(process_variance·dt, process_rate_variance·dt)`, still diagonal | a fixed per-call Q would divide injected process noise by the scan-rate ratio, re-introducing rate dependence through the covariance |
| (b) | one fade counter of `loop_rate · tracking_duration` ticks | tentative/confirmed/coasting counted in **scans** | upstream has no confirmation gate at all, so a single spurious detection is published on its second scan |
| (c) | separate `untracked_obstacles_` list, cost matrix indexed over `[tracked \| untracked]` | tentative tracks are ordinary members of one track list | see the defect this removes, below |
| (d) | fusion resets P to the identity | the information-weighted merged covariance is kept | upstream computes the merge for the *means* and discards the covariance, so a track fused from four confident estimates emerges less certain than any parent and no longer comparable in the next scan's cost matrix. Measured: 5.6e-4 m² kept versus upstream's 1.0 m² |
| (e) | one global `measurement_variance` for every axis of every track | per-observation `sigma_center_m` / `sigma_radius_m` from the detector | a short-arc observation must be distinguishable from a full-arc one, which is the point of having measured the bias |

**The defect change (c) removes.** `fissionObstacleUsed` skips any match whose index lands in the
untracked half (`row_min_indices[idx] >= T`). When two new observations both fall nearest the same
untracked seed, fission therefore declines to handle them — and the plain-match loop then builds a
`TrackedObstacle` from that seed for *each* of them, because it records the consumed observation in
`used_new_obstacles` but never records the consumed old obstacle in `used_old_obstacles`. Two tracks
are born on top of each other from one seed. With tentative tracks in the single list there is no
exclusion to skip and no double-spawn path to reproduce; the port's plain-match loop also marks the
track it consumed, closing the same hole from the other side.

### The NIS calibration, which endorsed P8's sigmas and overruled upstream's Q

P8 left `sigma_center_m`/`sigma_radius_m` provisional and asked this phase to calibrate them
against measured innovation statistics. Running that check moved a different number than expected.

* **P8's sigmas are sound and are left untouched.** Over the P8 detection corpus the emitted
  `sigma_center_m` averages **0.145 m** (range 0.067–0.216 m) against an actual centre error of
  **0.130 m RMS, 0.219 m worst** — an implied NIS of **0.8**, marginally conservative, the safe
  direction. The prior expectation was the opposite: that a fit-residual statistic would badly
  under-state a bias-dominated error. The `1/(1 − cos α)` dilution term supplies the missing
  magnitude on exactly the short arcs where the bias lives. `measurement_sigma_scale` is therefore
  **1.0 by evidence**, and `measurement_sigma_floor_m = 0.030` is only a guard against the zero
  sigma `CircleObservation::Validate()` admits — the detector's smallest emitted sigma is 0.067 m,
  so the floor never binds in operation.
* **Upstream's process noise had to go.** Only the Q/R *ratio* reaches a Kalman gain, and change
  (e) replaces upstream's flat R of 1.00 m² with variances 20–200× smaller. Holding Q at upstream's
  magnitude across that substitution does not preserve upstream's filter; it produces one in which
  process noise dominates the measurement. Measured mean NIS on the corpus was **0.18–0.41** —
  badly under-confident. Sweep (floor lifted so only the reported sigma acts):

  | `process_variance` | `process_rate_variance` | NIS (const-vel / occlusion) | vel RMSE (const-vel / occlusion) |
  |---|---|---|---|
  | 0.01 (upstream-scaled) | 0.10 | 0.305 / 0.358 | 0.068 / 0.069 |
  | 0.01 (upstream-scaled) | 0.30 | 0.289 / 0.339 | 0.105 / 0.105 |
  | 0.001 | 0.03 | 0.608 / 0.724 | 0.062 / 0.058 |
  | **0.0001** | **0.03** | **0.688 / 0.821** | **0.068 / 0.062** |
  | 0.00003 | 0.03 | 0.695 / 0.831 | 0.068 / 0.062 |

  Shipped: `process_variance: 0.0001`, `process_rate_variance: 0.03`. The position channel of a
  constant-velocity model needs no independent process noise — position evolves exactly as
  `p + v·dt` — so 1e-4 m²/s is a numerical-health floor admitting (3.2 mm)² per scan, below the
  projector's own range quantisation. NIS does not reach 1.0 because the corpus has *exactly*
  constant velocity, so any Q > 0 reads as conservative; driving Q to zero to chase it would
  overfit a manoeuvre-free fixture, which is why a **bounce** stream (a full 0.8 → −0.8 m/s
  reversal) was added beyond §10's four: it sizes the rate channel from below. Recovery to within
  the 0.10 m/s gate takes **7 scans (0.7 s)**.

### The association gate, and P8's finding 2

Upstream's cost weights a metre of radius disagreement exactly as much as a metre of position
disagreement. It could afford that because it never separated full-arc from half-arc error. P8 did,
and measured the de-enlarged radius biased small by 0.006–0.145 m as a function of visible arc — and
because the same short arc also drives the centre off by up to 0.15 m, the two error terms **peak
together**, so upstream's cost adds them in quadrature exactly when the position residual is
already largest.

The response is a weight, not a wider radius gate: the channel is *biased*, so it should inform
identity **less**, rather than keep its full vote and forgive larger disagreements. A weight also
keeps one threshold instead of introducing a second, and `association_radius_weight: 1.0` recovers
upstream's cost exactly, which is what pins the port. Shipped **0.25**. Measured worst-case cost
against the 0.30 m gate:

| Stream | w = 1.0 (upstream) | w = 0.25 (shipped) |
|---|---|---|
| visibility change mid-track | 0.232 m — **77% of gate** | 0.173 m — 58% |
| gap then visibility change | 0.200 m — 67% | 0.153 m — 51% |
| no radius bias present | 0.103 m | 0.101 m |

A note on what down-weighting gives up, recorded rather than asserted away: a 0.75 m size
difference now contributes 0.188 m of the 0.30 m gate, so a co-located object of a wildly different
size is no longer rejected on radius alone. That is intended — at 10 Hz two obstacles do not swap
sizes between scans, so radius is a weak identity cue and position is the strong one, and a genuine
one-to-many is the fission path's job.

**A correction to how finding 2 must be read.** The bias is on the **de-enlarged** radius
(`radius_fitted_m − radius_enlargement_m`, upstream's `true_radius`), which is how P8 measured it.
The raw `radius_fitted_m` the tracker associates on is that plus the constant 0.25 m enlargement, so
against ground truth it *over*-estimates (+0.089 to +0.278 m on the corpus) — measuring it without
de-enlarging first reports the enlargement as an error. None of that changes the design: the
enlargement is a constant and cancels in the difference between two frames, so the frame-to-frame
variation of the fitted radius **is** the variation of the bias, and that variation is what the gate
sees.

### P8's finding 4 (±π seam) against the coasting lifecycle — verified, not assumed

The finding-4 geometry reproduced exactly: a 0.25 m cylinder at 5 m astern, swept past the seam at
1.2 m/s, occupies ~6 bins at 1° resolution and splits across the non-wrapping ordered point list.

* **The hole is real: 2 of 40 scans undetected**, and it is bracketed by frames where only one half
  clears `min_group_points` — so the object reappears as a *partial arc*, with the radius biased.
  Findings 2 and 4 are not independent hazards; the seam manufactures the visibility change.
* **Coasting survives it comfortably.** 2 scans against `delete_misses: 10`, **0 ID switches**, a
  track present in 38 of 40 frames. Worst unweighted reacquisition cost 0.159 m against the 0.30 m
  gate.
* Fitted-radius swing across the crossing: **0.056 m peak-to-peak, 0.056 m in a single scan**, from
  a one-bin change in arc length with the object unchanged. Worst de-enlarged under-estimate
  0.056 m, inside P8's 0.006–0.145 m range.

The gap is a *mid-range* phenomenon: closer objects subtend enough bins that each half still clears
`min_group_points`, and farther ones fall under it whether split or not.

### §12 tracking gates

All met. Position and velocity RMSE are asserted on the zero-mean-error streams; the two visibility
streams inject a deliberate 0.15 m **systematic** offset, and no estimator removes a bias it is not
told about — conservative inflation is P10's job, not this phase's.

| Gate | Target | Measured |
|---|---|---|
| velocity RMSE @ ≤ 0.8 m/s | ≤ 0.1 m/s | 0.065–0.078 m/s |
| position RMSE | ≤ 0.05 m | 0.024–0.031 m |
| ID switches per crossing pair | ≤ 1 | **0** |
| confirmation delay | ≤ 3 scans | exactly 3 |
| deletion delay | ≤ `tracking_duration` (2.0 s) | 1.0 s (`delete_misses` binds first) |
| stale-output rate | 0 | **0** across all 7 streams |

Mean NIS by stream: 0.61 (constant velocity), 0.61 (crossing), 0.69 (occlusion), 0.94 (radius
jitter), 1.03 (visibility change), 0.77 (reacquisition), 1.62 (bounce — the manoeuvre the model
cannot represent, as expected).

### Lifecycle detail worth carrying forward

A **tentative track dies on its first miss**, which is upstream's semantics for the same object
(`untracked_obstacles_` is cleared and reassigned every callback). The cost is that confirmation
requires `confirm_hits` *consecutive* scans, so the confirmation delay is a hard 3 scans rather than
a distribution, and an obstacle that flickers on its first three scans never confirms. Flagged for
P10 rather than papered over.

### Config

Four keys added under `tracking`, all of them numbers this phase is answerable for:
`association_radius_weight`, `measurement_sigma_scale`, `measurement_sigma_floor_m`,
`initial_rate_variance`. Two existing keys changed value — `process_variance` and
`process_rate_variance`, per the sweep above — and they are no longer labelled UPSTREAM.
`measurement_variance` keeps upstream's 1.00 with a narrowed role: the fallback R for an
observation reporting a zero sigma, which stops a zero R driving the gain to 1 and the covariance
to 0.

---

## 9. Safety-state generation (P10) — gating, conservative inflation, and three things the doc got wrong

`core/safety/safety_state_generator.cpp`, consuming a whole `Tracking2DResult` and emitting
`SafetyObstacle`s. Stateless between frames by design: everything with memory lives in the
tracker, and a safety stage that remembered anything could disagree with the estimator about
what is being tracked.

### Reconciliation — what the seam actually looks like

The architecture doc's §6 formula is
`radius ← max(fitted, enclosing) + k_σ·σ_r + latency·|v|·k_lat`. Locating each symbol in the
shipped code found that **neither named radius is where the formula implies**.

* **`enclosing` is not reachable.** `radius_enclosing_m` lives on `CircleObservation`, a
  *detection*-stage contract. Neither `TrackState2D` nor `PerceptionObstacle` carries it, so it
  does not survive tracking and `max(fitted, enclosing)` has no second operand at this seam.
* **`fitted` is not the fitted radius.** `PerceptionObstacle::radius_true_m` is
  `TrackState2D::radius_m`, which is the Kalman filter driven by `radius_fitted_m` — and that
  observation already includes `detection.radius_enlargement_m`. In upstream's vocabulary the
  field carries `CircleObstacle::radius`, not `true_radius`. **The comment on the contract said
  the opposite and has been corrected**; believing it and "restoring" the true radius by
  subtracting 0.25 m would have removed the only term covering P8's short-arc under-estimate.
  Measured over-statement on the P8 corpus: **+0.089 m to +0.278 m**.
* **`k_lat` does not exist as a config key.** `safety.latency_inflation_s` *is* the coefficient.

### Q11 — resolved, and against the doc's own recommendation

Architecture doc §17 recommends "conservative enclosing over fitted". **Measured over all 20
fitted circles in the P8 corpus, the enclosing radius is SMALLER than the fitted one in 20 cases
out of 20, by 0.254–0.287 m.** It is never once the larger. The cause is geometric: the enclosing
radius is measured from the √3/3 centre, which sits behind the visible arc by construction
(P8 finding 1 put that offset at 0.09–0.19 m), and only the sensor-facing arc is ever hit — so
the farthest contributing point is barely past the true radius, while the fitted radius carries a
flat +0.25 m. Following the recommendation would under-estimate every radius by 0.25–0.29 m,
which is the doc's own definition of the one unrecoverable error.

Consequently no fourth filtered channel was added to carry the enclosing radius through the
tracker: it would provably never bind. `safety.use_enclosing_radius` is retained (frozen schema)
as a documented no-op, with a test asserting both settings produce identical output.

### The implemented rule, and two terms the doc's formula lacks

```
base   = max(radius_true_m, min_radius_m)
radius = base + k_σ·σ_r + k_σ·σ_pos + fixed + (age_s + latency_inflation_s)·|v|
```

* **`k_σ·σ_pos`.** Containment means the true *disc* is inside the safety *disc*, so the centre
  error has to be covered too — and it is the larger of the two (worst 0.204 m against 0.169 m
  for the radius on P9's corpus). The contract already anticipated this:
  `SafetyObstacle::position_inflation_m` exists and is reported separately.
* **`age_s` in the drift horizon.** The doc's fixed latency covers only generation→consumption.
  A coasting track admitted at `max_age_s` (0.30 s) has *already* drifted up to 0.24 m at the
  arena's 0.8 m/s — twice what the 0.15 s term covers. Using `age_s` is conservative by
  construction, not by tuning: `age_s` is measured from the *measurement* stamp while the
  published centre is the estimate at the (later or equal) *scan* stamp, so the real
  extrapolation is ≤ `age_s`. In the nominal case `age_s` is 0 and the two rules coincide.

Staleness is `now − PerceptionObstacle::last_update_stamp_s`, which P9 defines as the last
*matched measurement* time, not an advanced-to frame time. A stale track is **dropped**, never
emitted with a shrunk radius (§11's safety-dominance policy) — asserted both ways, since "the
stale one is gone" and "nothing stale was published smaller" are different claims.

`safety.min_track_hits` forced a design decision: the hit count is **not on**
`PerceptionObstacle` and is not recoverable from it (`confidence` is a ratio, `track_age_s` is a
duration). The stage therefore consumes the whole `Tracking2DResult` and joins against
`TrackState2D` by id. Without that join the key would be silently inert whenever it exceeded
`tracking.confirm_hits`.

### Q12 — the calibration sweep, and what it actually found

k_σ swept over {0, 1, 2, 3, 3.72, 5, 8, 12} on three corpora: **A** the shipped pipeline
(ray-cast moving cylinders → real detector → real tracker, 171 samples), **B** P9's seven streams
with their radius channel rebased onto the detector's convention (312), **C** P9's streams
verbatim, i.e. with the enlargement absent (312).

| k_σ | A containment / worst margin | B | C |
|---|---|---|---|
| 0.00 | 100 % / +0.069 m | 100 % / +0.085 m | **87.18 %** / −0.116 m |
| 1.00 | 100 % / +0.306 m | 100 % / +0.126 m | **88.14 %** / −0.075 m |
| **2.00** | **100 % / +0.509 m** | **100 % / +0.166 m** | **93.91 %** / −0.034 m |
| 3.00 | 100 % / +0.572 m | 100 % / +0.207 m | 100 % / +0.006 m |
| 3.72 | 100 % / +0.572 m | 100 % / +0.236 m | 100 % / +0.036 m |

**k_σ is retained at 2.0**, and the sweep's real product is not that number. The shipped pipeline
meets 99.9 % containment from k_σ = 0 upward, so k_σ is *not the binding term* — 2.0 is kept for
headroom on a 171-sample corpus, not because it is required. What the sweep exposed is that
containment was resting on a **detection** parameter: remove the enlargement and the shipped
config falls to 93.91 %.

That shortfall is a **systematic bias**, and a bias is covered by a fixed term, not by a
variance — raising `safety.radius_inflation_fixed_m` to 0.20 m restores corpus C to 100 % at the
*unchanged* k_σ. So rather than double-paying for margin the shipped configuration already has,
P10 added a cross-field constraint:

> `detection.radius_enlargement_m + safety.radius_inflation_fixed_m ≥ 0.20 m`

0.20 m is P9's 0.169 m worst case rounded up. The shipped config satisfies it at 0.30 m; zeroing
the enlargement now fails to load instead of silently losing containment. `config_test` covers it.

### The χ² check §12 asks for — and it is a rejection

Centre NEES and radius NIS, normalized by the tracker's own per-axis variances, over corpus B:

| statistic | measured | χ² prediction |
|---|---|---|
| centre NEES mean | 9.13 | 2.00 (2 dof) |
| centre NEES 99.9 % quantile | 100.30 | 13.82 |
| radius NIS 99.9 % quantile | 179.16 | 10.83 |

The model is rejected outright. The errors this pipeline makes are bias-dominated — P8's
short-arc centre offset and radius bias are systematic, not noise — so the filter's covariance
does not describe them. **This is why k_σ was calibrated empirically rather than read off a
quantile**; √χ²₂(0.999) = 3.72 would be the Gaussian answer and it is neither necessary nor
sufficient on its own. Reporting the rejection *is* the calibrated-coverage result.

### Q13 — the short-arc radius floor, decided on numbers

The doc's candidate was a hard floor at `safety.min_radius_m` (0.20 m). The competing candidate
was that `k_σ·σ_r` already scales up on short arcs via the detector's `1/(1 − cos α)` dilution.
**Neither is the answer, and the floor is kept anyway for a different job.**

* The floor **binds on 0 of 171 emissions**. The tracked radius carries the 0.25 m enlargement,
  so it runs 0.36–0.55 m where the floor is 0.20 m. No emitted state under-estimates the true
  radius at all, so there is nothing for a floor to rescue. It is retained as a **degenerate-fit
  guard** — a radius filter driven toward zero — and does exactly that in its own test.
* `k_σ·σ_r` **is** large on the shipped pipeline (worst 0.283 m) but *not because it tracks the
  bias*. The detector computes `fit_residual_m` against the **enlarged** radius, so the residual
  and every σ derived from it carry the same 0.25 m constant. Same detector, same scenes, only
  the enlargement changed: **mean `sigma_radius_m` 0.128 m at 0.25 m enlargement, 0.024 m at
  zero** — a 5.4× artefact. Remove the enlargement and the term collapses while the bias it
  appeared to cover does not move.

What actually covers the short-arc bias is the enlargement itself, which is why de-enlarging is
forbidden in the safety stage and why the cross-field constraint above exists.

### The confirmation-fragility question, answered by scenario

**Tentative tracks are not emitted.** A one-scan track has a zero velocity (the birth prior) and
a radius from a single unvalidated fit; publishing it feeds DPCBF a stationary constraint at a
possibly-artefactual location, and a spurious constraint is not a free safety win — it can push
the QP toward infeasibility or steer the robot into a real hazard to avoid an imaginary one. The
designed net for "something is out there and perception has not caught up" is §16's **per-frame**
fallback, which switches the whole frame rather than mixing a half-trusted estimate into a
trusted set. `SafetyStats::tentative_suppressed` keeps the decision visible.

The stress case was built rather than reasoned about: a **new** obstacle whose first appearance
is already inside the ±π seam hole (P8 finding-4 geometry, 0.25 m cylinder at 5 m, entering
within 0.06 rad of the seam). Measured over 40 frames: detected in 37, longest detection gap
**3 scans**, **1 tentative track killed by a miss** — so the fragility is genuinely exercised —
tracker confirmed in 34 frames, safety emitted in 34. First detection frame 0, first publication
frame 6: **0.6 s**, exactly `confirm_hits` (3) + `min_track_age_s` (2 scans) + the 3-scan seam
gap, and nothing else. Whenever it is published it contains the object (worst margin +0.639 m).

### §12 safety-generation gates

Measured on corpus A — the shipped pipeline, 171 samples across four moving ray-cast scenes.
Containment means `‖c_truth − c_safety‖ + r_truth ≤ r_inflated`, the conjunction, not either half.

| Gate | Target | Measured |
|---|---|---|
| conservative containment | ≥ 99.9 % | **100.0000 %** (worst margin +0.509 m) |
| radius under-estimation rate | ≤ 0.1 % | **0.0000 %** |
| position/velocity uncertainty coverage (χ² test) | calibrated | calibrated empirically; **χ² model rejected** (above) |
| latency inflation correctness (analytic) | — | exact to **1e-15 m** over a speed × age grid |

Containment also holds at **100 %** with the latency term zeroed, so the uncertainty terms carry
it without the drift term paying for them.

**The cost, which P11 and P15 need.** Mean inflation 0.444 m, decomposing as elapsed drift
0.009 m + k_σ terms 0.293 m + fixed 0.050 m + latency drift 0.105 m. A 0.25 m arena cylinder is
therefore presented to DPCBF at roughly **0.94 m** before the filter's own `s = 1.05` and
`r_rob`. Containment says nothing about whether the constraint set is still *usable*; this is the
number the §12 intervention-rate and QP-feasibility gates will be decided by.

### Velocity spike clamping (R10, doc §17)

In the safety stage, not the tracker: the tracker's job is to report what its filter believes,
and a quietly clipped rate state is an estimator whose NIS statistics stop meaning anything (P9
depends on those). The clamp preserves direction and scales magnitude to `max_speed_mps`; the
obstacle is **not** dropped, because a spike is evidence of a real object tracked badly. The
inflation uses the *unclamped* speed — the clamp bounds what DPCBF integrates, the inflation
bounds where the object might be — with `max_radius_m` keeping that finite.

Fault-injected, and getting the fault in took one correction worth recording: **a single 3 m
displacement produces no spike at all**, because the association cost exceeds
`min_correspondence_cost_m` and the tracker simply drops the observation and coasts. Only
displacements that stay *inside* the gate reach the rate channel — an association *drag*, which
is precisely what R10 names. Sustained 0.25 m/scan for 14 scans drove the KF rate state to
**3.225 m/s**; every emitted state came out at exactly 1.600 m/s, the clamp firing on 13 of 37
emissions. A 0.80 m/s velocity — the arena maximum — passes through untouched. A second finding
falls out: this filter damps a velocity fault hard, and a short spike does not survive it.

### Config

No new keys. `k_sigma`, `fixed`, `latency` and `min_radius_m` keep their shipped values; what
changed is the commentary, which now states the implemented rule rather than the doc's formula,
and one new **cross-field constraint** (the short-arc bias budget above). `use_enclosing_radius`
is documented as a no-op with a test to keep it one.

## 10. Two corrective passes over §9's constants — what moved, and what it cost

§9 is left as P10 wrote it, because it is the record of what P10 actually measured. Two later
passes changed constants that §9's numbers depend on, so the statements below **supersede** the
ones they name. Both passes changed configuration and commentary only — no algorithm, no
contract, no new key.

### Pass 1 — `fit_residual_m` was measured against the enlarged radius

The detector computed the circle residual as the RMS distance from `radius_fitted_m`, which
already carries `detection.radius_enlargement_m`. Every residual therefore carried the
enlargement as an additive constant, and `fit_residual_m`'s only consumer is the measurement-sigma
formula. Corrected to measure against `radius_unenlarged`.

Superseded from §9:

* "mean `sigma_radius_m` 0.128 m at 0.25 m enlargement, 0.024 m at zero — a 5.4× artefact" —
  the artefact is gone. Both readings are now **0.0239 m**, and `sigma_radius_m` is asserted
  invariant under the enlargement (safety §H).
* "k_σ terms 0.293 m" in the inflation attribution → **0.0739 m**, a 75 % reduction.
* QP feasibility, which P11 measured at 0.9750 against the oracle's 1.0000, now **meets**.
* The Q12 sweep table's corpus-C row at k_σ = 2.00 — P10 recorded **93.91 %**; the smaller sigmas
  took it to **90.38 %**, and pass 2's larger safety-side fixed term then lifted it to
  **97.76 %**. Three values for one cell, which is why the config's commentary now names the
  corpus and the config rather than quoting a percentage. The *conclusion* it supports is
  unchanged and is what matters: without the enlargement the shipped k_σ does not reach 99.9 %.

### Pass 2 — `radius_enlargement_m` was still upstream's default

P1 kept upstream's 0.25 m "pending the P8 short-arc-bias experiment (Q11)". P8 ran that
experiment and nobody closed the loop. Re-derived here:

| measurement | corpus | value |
|---|---|---|
| worst de-enlarged radius under-estimate | P8 §I, 18 matched cylinders (12 full-arc, 6 half-arc) | **0.16127 m** |
| — full-arc population | n = 12 | mean −0.0061 m, worst under 0.0200 m |
| — half-arc population | n = 6 | mean −0.1446 m, worst under 0.1613 m |
| worst under-estimate surviving the filter to the safety input | P10 corpus C, 312 samples | **0.1668 m** |

`detection.radius_enlargement_m` = **0.17 m** — the smallest 0.01 m-granular value covering both
on its own. Shipping 0.16127 m would assert that an 18-cylinder corpus with 6 half-arc samples
located the population's worst case; 0.01 m is finer than that population's own spread
(0.1358–0.1613 m).

`safety.radius_inflation_fixed_m` **0.05 → 0.08 m**, carrying the remainder of a 0.25 m flat
total. The split is free and the total is not, and that is measured rather than assumed: with
`min_radius_m` out of the way, corpus C carrying the whole budget on the safety side and corpus B
carrying the shipped split agree to **1e-9 m**. (With the floor active they do not, because the
floor binds on the enlargement-free corpus and not on the shipped one — so corpus C at the full
safety-side budget is *conservative* relative to the real split, not equivalent. Asserted in that
direction.)

### Why the flat budget stopped at 0.25 m and not at the 0.20 m cross-field floor

This is the finding of pass 2, and it contradicts what pass 1's report projected (a reachable
saving of ~0.10 m).

§9's strict-form check — same-instant containment with `latency_inflation_s` zeroed, because
crediting a consumption latency that has not been consumed lets one term's margin pay for
another's shortfall — needs a **flat total of 0.2395 m**. Its worst margin is exactly
`total − 0.2395` m across the swept range, and the 1:1 relation is now asserted rather than
inferred from one point. At the 0.20 m cross-field floor the gate reports **90.64 %** against a
99.9 % target. So:

* realised saving **0.05 m** (0.30 → 0.25 m of flat terms), not 0.10 m;
* the cross-field constraint stays at 0.20 m, where its own derivation (the bias) puts it — it is
  a load-time check on one measured quantity, and folding a second, differently-derived
  requirement into the same constant would make neither traceable;
* §9's "containment also holds at 100 % with the latency term zeroed" still holds, but the margin
  went from **+0.0605 m to +0.0105 m**. `safety.latency_inflation_s` is no longer inert.

### §12 gates after both passes

| metric | target | P11 | after pass 1 | after pass 2 |
|---|---|---|---|---|
| command-delta RMSE | ≤ 0.100 m/s | 0.3237 | 0.1890 | **0.1732** — still missed, by 1.73× |
| QP feasibility | ≥ oracle (1.0000) | 0.9750 | 1.0000 | **1.0000** |
| velocity RMSE | ≤ 0.10 m/s | 0.1122 | 0.0479 | **0.0479** |
| intervention-rate difference | ≤ 0.200 | 0.0875 | 0.0437 | **0.0375** |
| min-clearance difference | ≥ −0.05 m | — | 0.0914 | **0.0728** |
| collisions (lookahead proxy) | 0 | — | 0 | **0** |
| end-to-end latency | ≤ 150 ms | — | 100.10 | **100.06 ms** [PARTIAL] |
| containment (corpus A, 171) | ≥ 99.9 % | — | 100.0000 % | **100.0000 %** |
| radius under-estimation (corpus A) | ≤ 0.1 % | — | 0.0000 % | **0.0000 %** |
| containment (corpus C at the shipped flat budget) | ≥ 99.9 % | — | — | **100.0000 %** |

Radius handed to the QP: **0.4776 → 0.4282 m** over truth on average. A 0.25 m cylinder reaches
the filter at ~0.69 m rather than ~0.74 m (P10: ~0.94 m).

### The open finding

Two constants have now been re-derived from measurements and the command-delta gate is still
missed by 1.73×. The ablation is unambiguous about where the miss lives — substituting the true
radius gives 0.0192 m/s, substituting true centres and velocity gives 0.1564 m/s — so it is still
the radius. But the flat terms are done, and what is left at the top of the attribution is
`safety.latency_inflation_s`: 0.15 s, 0.1095 m, 26 % of the over-statement, and still a
**JUDGEMENT** value. It cannot be re-derived from anything this repository measures, because the
§12 latency figure is a 100 ms scan window plus sub-millisecond compute with the queueing and
thread-handoff terms explicitly absent until P12.

That two independent, correctly-derived reductions moved the gate from 3.2× to 1.7× and stopped
is the result — not a third constant to reach for.

**And the target is not reachable this way at all**, which the pass measured rather than
inferred, because "which constant next" is the wrong question if the answer is none of them. Two
deliberately unshippable configurations were run through the same paired probe:

| configuration | command-delta RMSE |
|---|---|
| shipped | 0.1732 m/s |
| `latency_inflation_s` = 0 and the flat budget at the strict-form minimum (0.2395 m) — the best a perfect latency measurement could do while still containing the obstacle | **0.1312 m/s** |
| the same, and k_σ zeroed as well — abandons containment on corpus C, included only as a lower bound | **0.1124 m/s** |

So even a latency measurement that came back as zero leaves the gate missed by 1.31×, and giving
up containment entirely still misses by 1.12×. The remaining distance is a property of presenting
DPCBF a conservatively inflated radius at all, not of any one constant's value — the ablation's
first row (true radius → 0.0192 m/s) is the same statement from the other side. Whatever closes
this gap is a change to *what is published*, not a smaller number in the config. Both floor rows
are asserted, so if a later change makes either reach the target this conclusion goes red rather
than quietly persisting.

### One detection-side consequence, recorded because it changed a test's answer

The enlargement is added **before** `max_circle_radius_m` is applied, so shrinking it widens what
the detector will report. One scene in the corpus crosses that threshold: two overlapping 0.25 m
cylinders at 2 m merge into a circle measuring 0.549 m at the shipped enlargement and 0.629 m at
upstream's 0.25 m, against a 0.60 m cap. At 0.25 m the cap dropped it and the scene reported **no
obstacle at all** while a `circles.size() <= 1` assertion passed vacuously; at 0.17 m it is
emitted. The check now asserts the count from both sides and verifies the mechanism.

Everything else in P8's fit-accuracy report is byte-identical — cluster precision/recall/purity,
both arc populations' centre and radius errors, the seam findings, and pass 1's §K residual
invariance — as it must be, since the de-enlarged radius is what those measure.
