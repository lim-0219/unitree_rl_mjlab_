# Third-party notices

## Reference packages (not compiled, not linked, not shipped)

Two ROS packages sit untracked at the repository root as **reference implementations and
regression oracles**. Neither is on any production path, and no upstream source file is
compiled into any shipped target.

| Package | Location | License | Status in this subsystem |
|---|---|---|---|
| `obstacle_detector` | `obstacle_detector/` | BSD-3-Clause (Poznan University of Technology, 2017; Mateusz Przybyla) | **Algorithms ported** (detection phase) — see the file mapping below. Its ROS-free headers compile **only** inside the optional Armadillo-linked test target (`PERCEPTION_BUILD_UPSTREAM_ORACLE`, default OFF), where they serve as the regression oracle. ROS1/catkin, so it can never enter production here. |
| `pointcloud_to_laserscan` | `pointcloud_to_laserscan/` | BSD-3-Clause | **Algorithm ported** (projection phase): `pointcloud_to_laserscan_node.cpp:178-229` → `src/core/projection/scan_projector.cpp`, with a verbatim transcription of the upstream loop in `tests/regression/p2l_reference.h` as the exact-equality oracle. ROS2/ament; no upstream file is compiled. |

### Ported file mapping — `obstacle_detector` → `perception/`

Every entry is an algorithm re-implemented in Eigen/STL against this subsystem's own
contracts. No upstream file is included by, or linked into, any production target; the
divergences are enumerated in the ported files' own headers.

| Upstream | Port | Notes |
|---|---|---|
| `src/obstacle_extractor.cpp:158-403` (`groupPoints`, `detectSegments`, `mergeSegments`, `detectCircles`, `mergeCircles`) | `src/core/detection/segment_circle_detector.cpp` | Pipeline order, thresholds and literal constants preserved, including the truncated `0.5773502` circumcircle factor. Two upstream point-counting defects are corrected by default and reproduced exactly under `UpstreamQuirks::Exact()`; both are documented in `segment_circle_detector.h`. |
| `include/obstacle_detector/utilities/figure_fitting.h:54-164` (`fitSegment`, via `arma::pinv`) | `src/core/detection/line_fit.cpp` | Armadillo replaced by a streaming Givens QR plus a closed-form 2×2 pseudo-inverse that reproduces Armadillo's own rank-truncation rule (`op_pinv_meat.hpp:135`). |
| `utilities/point.h`, `utilities/segment.h`, `utilities/circle.h`, `utilities/point_set.h` | inlined into `src/core/detection/segment_circle_detector.cpp` | Only where upstream's arithmetic differs from the obvious Eigen equivalent (division by zero returning a zero point, `normalized()` of a zero vector); each such case is commented at its use. |
| the same files, unmodified | `tests/regression/upstream_extractor_reference.h` (test-only) | The pipeline transcribed verbatim, defects included, as the oracle. Compiled only when `PERCEPTION_BUILD_UPSTREAM_ORACLE=ON`. |
| `utilities/kalman.h` (2-state KF), `utilities/tracked_obstacle.h` (three-axis wiring), `src/obstacle_tracker.cpp:156-353,405-454` (association, fusion, fission) | `src/core/tracking/kf_circle_tracker.cpp`, `include/perception/core/tracking/axis_kalman.h` | The five Kalman expressions are expanded to explicit 2x2 scalar arithmetic in the same operation order, so agreement with the Armadillo original is exact rather than approximate. Five deliberate replacements (timer-driven dt, per-tick Q, the fade counter, the untracked side list, fusion's covariance reset) and three upstream defects are enumerated in `kf_circle_tracker.h` and `docs/architecture.md` section 8. |
| `utilities/kalman.h`, unmodified | `tests/regression/tracking_oracle_test.cpp` (test-only) | Upstream's own Kalman header COMPILED AND LINKED against Armadillo as the regression oracle - not transcribed. `TrackedObstacle` and `obstacleCostFunction` are transcribed in `tests/regression/upstream_tracker_reference.h` instead, because those two reach ROS message and `tf` headers. Built only when `PERCEPTION_BUILD_UPSTREAM_ORACLE=ON`. |

Policy, unchanged from the root architecture document §15:

* Both trees are **read-only**. Any needed change happens in the port, never upstream.
* When code is ported, the upstream license block is preserved in the ported file and the
  table above is updated with the specific file mapping.
* Upstream types (`sensor_msgs/LaserScan`, `obstacle_detector::*`) are barred from being
  the canonical internal model. `perception/core/contracts/*` are canonical.

## Documentation sources cited in code and configuration

These are documents, not code, and nothing is redistributed from them. They are listed
because numeric constants in this subsystem are transcribed from them and a future reader
needs to know where to check.

| Source | Used for |
|---|---|
| Unitree Robotics `unitree_ros`, `robots/g1_description/*.urdf` | The `mid360_joint` extrinsic, transcribed verbatim. |
| Livox / DJI, *Livox Mid-360 User Manual* v1.0 (2023-01) | Vertical/horizontal FOV, blind zone, frame rate, range and angular error, point rate, body dimensions, mounting notes. |
| Google DeepMind, MuJoCo 3.3.6 (`engine_ray.c`, API reference) | `mj_ray` / `mj_multiRay` semantics, including the `geomgroup` mask and the batched-path pruning approximation. Vendored under `simulate/mujoco/` with its own `THIRD_PARTY_NOTICES`. |

Repository-local analyses that these constants come from, and which should be read before
changing any of them: `g1_mid360_extrinsic.md` (authoritative for the mount) and
`g1_mid360_sensor_model.md` (sensor model, aperture, exact-vs-batched raycasting).
