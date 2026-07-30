# Third-party notices

## Reference packages (not compiled, not linked, not shipped)

Two ROS packages sit untracked at the repository root as **reference implementations and
future regression oracles**. Neither is on any production path today, neither is linked by
any target in `perception/CMakeLists.txt`, and no code has been ported from either yet.

| Package | Location | License | Status in this subsystem |
|---|---|---|---|
| `obstacle_detector` | `obstacle_detector/` | BSD-3-Clause (Poznan University of Technology, 2017) | Reference only. Its ROS-free algorithmic core is scheduled for porting in the detection/tracking phases; the upstream headers would then compile **only** inside an optional Armadillo-linked test target (`PERCEPTION_BUILD_UPSTREAM_ORACLE`, default OFF). ROS1/catkin, so it can never enter production here. |
| `pointcloud_to_laserscan` | `pointcloud_to_laserscan/` | BSD-3-Clause | Reference only. Its ~60-line binning core is scheduled for porting in the projection phase, with exact-equality equivalence tests. ROS2/ament. |

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
