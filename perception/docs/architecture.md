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
    core/{pipeline,projection,segmentation,detection,tracking,safety}/
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
| `PERCEPTION_BUILD_UPSTREAM_ORACLE` | OFF | Reserved for the Armadillo-linked `obstacle_detector` regression oracle. Currently errors if ON. |
| `PERCEPTION_BUILD_TESTS` | ON | Build and register the CTest targets. |

The three reserved options fail loudly rather than silently doing nothing, so nobody
concludes a feature is present because the flag was accepted.

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

Deskew, gravity alignment, ground segmentation, scan projection, detection, tracking,
safety-state generation, the ESTIMATED half of the DPCBF adapter, every obstacle-source mode
above `oracle`, the perception thread and its queues, and the ROS2 topic set. (Dump/replay is
built — §5a. The oracle half of `adapters/dpcbf` and mode 1 of the ladder landed in P3;
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
