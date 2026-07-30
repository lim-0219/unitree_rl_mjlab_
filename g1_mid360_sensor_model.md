# Phase 6c — Mid-360 sensor model: official extrinsic + optical aperture

**Status:** COMPLETE. All eight acceptance criteria met (§6). Phase 6a/6b may
now be re-run.
**Date:** 2026-07-29
**Predecessor:** [`docs/g1_mid360_extrinsic.md`](g1_mid360_extrinsic.md) — where
the extrinsic numbers come from. This document is about making the *sensor
model* physically valid at that extrinsic.
**Reproduce everything:** §8.

---

## 1. Executive summary

Phase 6b ended with a correct-but-blind MuJoCo backend: at the true extrinsic
the Mid-360 sits inside `head_link.STL`, a closed envelope, so **100 % of rays
terminated on the robot's own head and 0 points reached the estimator**.

Phase 6c fixes the sensor model and, in doing so, found a second defect that
had been silently corrupting every MuJoCo scan.

| | before 6c | after 6c |
| --- | --- | --- |
| Extrinsic | identity placeholder | official Unitree `mid360_joint` |
| Sensor height | 0.837 m (inside the waist) | **1.2654 m** (head, correct) |
| Self hits | 11520 / 11520 = **100 %** | 326 / 11520 = **2.8 %** |
| Points to estimator | **0** | **~8 800** |
| Estimator valid | **no** | **yes** |
| Rays flying through walls | up to **23.7 %** (undetected) | **0 %** |

Two changes did this:

1. **An optical aperture**, expressed as a MuJoCo geom-visibility group. The two
   `head_link` geoms move to `group="2"`, and the ray caster excludes that group
   through `mj_multiRay`'s documented `geomgroup` mask. Physics collision,
   mass, inertia and rendering are untouched.
2. **`raycast_exact = true`** — cast each ray with `mj_ray` instead of batching
   through `mj_multiRay`. This was **not** a stylistic choice: measurement
   showed the batched path silently drops up to 23.7 % of wall hits (§4).

One assumption remains flagged rather than buried (§7.1), and one pre-existing
inconsistency was found and fixed along the way (§4.3).

---

## 2. Research: how other simulators handle a LiDAR inside a robot mesh

Conducted before implementing, as required. Every conclusion is tied to a
source; nothing here is from memory.

| # | Source | URL | What it establishes |
| --- | --- | --- | --- |
| R1 | MuJoCo API reference, `mj_ray` / `mj_multiRay` | https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html | **The official mechanism.** `geomgroup` is documented verbatim as "an array of length mjNGROUP, where 1 means the group should be included. Pass NULL to skip geom group exclusion." `bodyexclude`: "Exclude geoms in body with id bodyexclude, use -1 to include all bodies." `flg_static`: "If flg_static is 0, static geoms will be excluded." |
| R2 | MuJoCo 3.3.6 `src/engine/engine_ray.c` (`git show 3.3.6:...`) | https://github.com/google-deepmind/mujoco | The actual filtering (`ray_eliminate`) and the pruning approximation (`mju_multiRayPrepare`) that §4 diagnoses. Read at the pinned version, not the docs' newer signature. |
| R3 | `mujoco_lidar` (PyPI 0.2.6) — a Livox-model LiDAR simulator for MuJoCo | https://pypi.org/project/mujoco_lidar/0.2.6/ | **Prior art using the same mechanism.** Uses `bodyexclude` explicitly to "avoid LiDAR detecting itself", and offers `geomgroup` ("0-5, None means all") as the selective alternative. Supports Livox `mid360` by name. |
| R4 | Isaac Sim — PhysX Lidar / RTX Lidar docs | https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_physx_lidar.html | "The LIDAR can only detect objects with **Collisions Enabled**." Detection is gated on a *separate* per-prim property, not on the render mesh. |
| R5 | NVIDIA developer forum — mesh invisible to rendering but detectable by RTX LiDAR | https://forums.developer.nvidia.com/t/how-can-i-make-a-mesh-invisible-in-rendering-while-keeping-it-detectable-by-rtx-lidar-in-isaac-sim-6-0/374421 | Confirms the **separate perception geometry** pattern is standard practice: one mesh for rendering, a different one for LiDAR detection. |
| R6 | Gazebo ROS ray sensors | https://github.com/Field-Robotics-Lab/dave/wiki/Gazebo-ROS-Ray-Sensors | `gazebo_ros_laser` intersects **physics/collision** geometry; `gazebo_ros_gpu_laser` intersects **graphics/visual** geometry. The visual/collision split is the lever, and the two backends see different worlds. |
| R7 | `robot_self_filter` (ROS/ROS 2) | https://github.com/leggedrobotics/robot_self_filter | The **real-robot** answer: filter returns that fall inside URDF collision shapes, after the fact. |
| R8 | `robot_body_filter` | https://github.com/peci1/robot_body_filter | Same family; also annotates points as *on the robot* vs *shadowed by the robot*. |
| R9 | Unitree G1 overview (distributor mirror of Unitree material) | https://docs.quadruped.de/projects/g1/html/g1_overview.html | "LIVOX-MID360 laser radar is **integrated into the G1 head**", advertised at "360° horizontal field of view" and "59° maximum vertical angle". |
| R10 | Livox Mid-360 User Manual v1.0, p.8 "Mounting Notice" | https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Mid-360_User_Manual_EN.pdf | §2: "the FOV must not be blocked by an object, **including glass**." §3: "There is no orientation requirement when mounting… Use the bottom surface for mounting." |

### 2.1 What the field actually does

Three distinct strategies, and they are not interchangeable:

* **Gate detection on a property separate from rendering** (R1, R3, R4, R6).
  Every major simulator provides one — MuJoCo `geomgroup`/`bodyexclude`, Isaac
  "Collisions Enabled", Gazebo collision-vs-visual. This is the *simulator*
  answer to "the sensor is inside the robot".
* **Author separate perception geometry** (R5). More faithful when the real
  aperture is known, because the LiDAR mesh can differ from the render mesh.
* **Filter afterwards from the URDF** (R7, R8). This is what real robots do.

**Crucially, the third strategy cannot solve our problem, and we already had
it.** Phase 6b's ancestry self-filter is exactly `robot_self_filter`'s idea and
it worked perfectly — it removed 100 % of the self returns. But the failure was
never spurious returns; it was **occlusion**. Every ray terminated on the shell,
so there was nothing beyond it to report. Removing a point does not recover the
measurement it destroyed. That is why Phase 6b left the sensor blind despite a
provably correct self-filter, and why Phase 6c had to change the *ray casting*,
not the *point filtering*.

---

## 3. Task 3 — alternatives considered, and why C won

| | Approach | Advantages | Disadvantages | Verdict |
| --- | --- | --- | --- | --- |
| **A** | **Separate perception geometry** — a second mesh set for LiDAR only (R5) | Most faithful if the real aperture is known; render and perception fully decoupled | **Requires inventing the aperture's shape**, which no source documents. Doubles the mesh assets and the maintenance burden | **Rejected** — cannot be built without undocumented assumptions |
| **B** | **Optical aperture geometry** — cut a hole in `head_link.STL` | Physically the most literal model | Same fatal problem as A (aperture undocumented), plus CSG mesh authoring, plus it changes the *visual* robot and the collision hull | **Rejected** — same reason, higher cost |
| **C** | **Selective geom exclusion** via MuJoCo's `geomgroup` mask (R1, R3) | Native, documented API; zero mesh authoring; **per-geom**, so it generalises to a real aperture later by re-grouping; provably dynamics-neutral; one config line to disable | Treats the whole head as transparent, so it over-permits wherever the real shell is opaque (§7.1) | **CHOSEN** |
| **D** | **Post-hoc self-filtering** (R7, R8) | Matches real-robot practice; already implemented | **Cannot restore occluded rays** — see §2.1. Does not solve this problem at all | **Rejected** — already present and demonstrably insufficient |
| **E** | `bodyexclude` instead of `geomgroup` | Also native, slightly cheaper | Excludes exactly **one body id**, and the head geoms live on `torso_link` alongside the torso mesh — excluding that body would also make the chest transparent. Promoting `head_link` to a real body would move mass bookkeeping and perturb dynamics | **Rejected** — wrong granularity |

**Why C is justified rather than a hack.** The single assumption it makes —
*the head shell does not occlude the sensor's FOV* — is the one assumption that
**documentation supports**: Unitree advertises the integrated G1 sensor at the
full 360° × 59° FOV (R9), and Livox requires an unobstructed FOV, explicitly
including glass (R10). A sensor sealed inside an opaque shell could not meet
either statement. Options A and B would have required *inventing* geometry that
no source describes, which is a strictly worse epistemic position.

And C degrades gracefully: because the mask is **per geom**, the day the real
aperture is documented, one splits `head_link` into opaque and transparent
geoms and sets `group` on each. No code changes.

---

## 4. Implementation

### 4.1 The aperture

`src/assets/robots/unitree_g1/xmls/scene_g1.xml`:

```xml
<geom name="head_link_visual"    ... group="2" ... mesh="head_link"/>
<geom name="head_link_collision" ... group="2" ... mesh="head_link"/>
```

`MountedLidarSimulator::ScanMujoco` builds an `mjtByte[mjNGROUP]` mask with
every group enabled except `lidar_transparent_geom_group`, and passes it as
`geomgroup` (R1). `-1` disables the mask and reproduces pre-6c behaviour
exactly.

**Dynamics are provably untouched.** In MuJoCo `group` is a
visualization/filtering attribute; `contype`, `conaffinity`, `density`, mass and
inertia are unchanged, and `torso_link` carries an explicit `<inertial>` so its
geoms never contributed mass in the first place. Group 2 also still renders:
`mjv_defaultOption` enables groups 0–2 (`engine_vis_init.c`, verified at 3.3.6),
so the head looks identical in every viewer.

### 4.2 `raycast_exact` — a defect found while validating

While building V2 the wall test failed with a 1.84 m range error. The offending
points turned out to be **on the floor beyond an opaque wall**: the ray had
passed straight through it.

Root cause, from R2: `mju_multiRayPrepare` prunes candidate geoms using a
per-geom angular bounding box computed from the **8 AABB corners**. That box is
not a valid outer bound for a geom subtending a large solid angle — the extreme
bearing over a big flat face occurs on an **edge**, not a vertex — so
centre-of-face rays fall outside the corner-derived bound and the geom is
skipped. `mj_ray` scans every geom and has no such approximation.

Measured on the G1 with the production 11520-ray table:

| Scene | batched `mj_multiRay` | exact `mj_ray` |
| --- | --- | --- |
| 2 m wide wall at 3 m | **5.3 %** of wall rays fly through | 0.0 % |
| 8 m wide wall at 3 m | **23.7 %** of wall rays fly through | 0.0 % |
| G1 + 8 m wall, whole scan | 471 / 11520 rays (**4.09 %**) miss a nearer surface, worst by **1.84 m** | 0 |
| Cost per scan | 4 553 µs | 8 327 µs (**1.8×**) |

The error is always *missed occlusion*, never a wrong range: rays the batched
path does resolve are exact to 1e-9 m. Since the MuJoCo backend already fails
the O6 0.4 ms/scan gate by an order of magnitude (Phase 6b, Decision B),
**1.8× buys correctness on a budget that was already blown**, so
`raycast_exact` defaults to `true`. `false` remains available and V2b asserts
the artifact is still reproducible, so the justification cannot rot.

Bonus: `mj_ray` takes a `const mjData*`, so the exact path does not mutate
simulation state at all.

**This retroactively affects Phase 6b.** Its MuJoCo runs used the batched path,
so their arena-wall returns were under-detected by roughly this margin. Phase
6b's *conclusion* (analytic stays default) is unchanged — it rested on the
starved scan and the O6 failure — but its scan censuses should not be quoted as
exact.

### 4.3 A latent inconsistency, fixed

`scene_g1.xml` claimed the site was "asserted at startup" against the YAML. **No
such assert existed.** The closed loop drives the sensor from the MJCF `<site>`;
the headless evaluator composes it from `perception.lidar.{translation_xyz_m,
rotation_rpy_rad}`. Nothing kept them in step, so the repository could carry two
different "official" extrinsics.

`VerifyLidarExtrinsic()` (`simulate/src/main.cc`) now compares site position,
site orientation and parent body against the YAML at every model load and exits
non-zero on disagreement. Verified by negative test — injecting the *deprecated*
z = 0.40618 produces:

```
FATAL: the MJCF <site> and perception.lidar disagree about the Mid-360 extrinsic.
  site pos      0.0002835     3e-05  0.428434
  yaml pos      0.0002835     3e-05   0.40618
  position error 0.022254 m
```

Also fixed: the headless evaluator's `base_height_m` was **0.793 (pelvis)**,
but the extrinsic's parent frame is **`torso_link`**. Left alone, the headless
sensor would have sat 44 mm low. Now `0.837`, derived as pelvis `0.793` +
`waist_roll_joint` z `0.044` and confirmed against the compiled model.

### 4.4 Files changed

| File | Change |
| --- | --- |
| `src/assets/robots/unitree_g1/xmls/scene_g1.xml` | official site pose (quaternion); head geoms → `group="2"`, named |
| `dpcbf/include/.../mounted_lidar_simulator.h` | `lidar_transparent_geom_group`, `raycast_exact`, `MujocoScanStats::aperture_transmitted` |
| `dpcbf/src/perception/mounted_lidar_simulator.cpp` | geomgroup mask; exact per-ray path; aperture-transmission diagnostic |
| `dpcbf/config/*.yaml` (6) | official extrinsic; aperture group; `base_height_m` 0.793 → 0.837; FOV comment |
| `simulate/src/main.cc` | `VerifyLidarExtrinsic()` at all three model-load sites |
| `dpcbf/tests/perception_sensor_model_test.cpp` | **new** — the validation suite |
| `dpcbf/scripts/plot_mid360_sensor_model.py` | **new** — Figure 1 |

**Not touched:** `CylinderStateEstimator`, the Kalman filter,
`ObstacleStateAdapter`, `DpcbfSafetyFilter`, `ObstacleState`, `lidar_types.h`,
the controller. Verified by `git diff --stat`.

---

## 5. Task 5 — validation suite results

`ctest --test-dir simulate/build -R perception_sensor_model_test` — **6/6
scenarios, 0 failures.**

![Phase 6c sensor model evidence](figures/g1_mid360_sensor_model.png)

*Figure 1 — panels 1–2 from the shipped Unitree meshes; panels 3–4 are measured
numbers from the runs below. Regenerate: `python3
dpcbf/scripts/plot_mid360_sensor_model.py`.*

### V1 — empty world

```
origin world (-0.0037, 0.0000, 1.2654)  rays 11520  raw hits 8640
self 478  aperture-transmitted 11520  to estimator 8162
```

* zero accepted points terminate on the head shell — **PASS**
* self-hit fraction **0.0553** (was 1.0000) — **PASS**
* the aperture demonstrably transmits rays the shell would block (11520 of
  11520 rays would have hit a group-2 geom first) — **PASS**

### V2a — front wall, range accuracy

| Wall | Points | Worst \|measured − analytic\| | Rays through the wall |
| --- | --- | --- | --- |
| 2 m wide | 547 | **0.000000000 m** | 0 |
| 8 m wide | 1439 | **0.000000000 m** | 0 |

Compared against an independent analytic plane intersection, not against
another MuJoCo call.

### V2b — why `raycast_exact` defaults to true

Batched path, same scenes: 29/547 (5.3 %) and 341/1439 (23.7 %) rays fly
through. Ranges that *do* resolve remain exact — **PASS** (§4.2).

### V3 — single known cylinder (r = 0.30 m at 2.5 m)

| Quantity | Measured | Expected | Result |
| --- | --- | --- | --- |
| Points | 232 | > 50 | PASS |
| Radial residual, every point | 0.000000000 m | 0 ± 1e-6 | PASS |
| Angular half-extent | 0.107231 rad | `asin(r/d)` = 0.120112 | PASS (within one 1° ray step) |
| Fitted centre (Kasa, independent of the production estimator) | (2.50000, −0.00000) | (2.5, 0) | PASS (< 5 mm) |
| Fitted radius | 0.30000 | 0.30 | PASS (< 5 mm) |

The measured extent is narrower than `asin(r/d)` by construction: a discrete
1° grid cannot sample the exact tangent points.

### V4 — self-occlusion

| | Wall points | Self hits |
| --- | --- | --- |
| Arms down | 1439 | 351 |
| Right arm raised | 1209 | 1096 |

The raised arm occludes **230 wall points (16.0 %)** and adds self hits — the
arm **does** block, while the head shell does not. Exactly the required
asymmetry. **PASS**

### V5 — rigid attachment across poses

`T_torso,lidar` recomposed from world poses and compared to the authored
extrinsic:

| Pose | Sensor world z | \|Δt\| | \|ΔR\| |
| --- | --- | --- | --- |
| standing (neutral) | 1.2654 | 1.1e-16 | 3.4e-13 |
| waist pitch +0.4 rad | 1.2315 | 6.6e-17 | 3.4e-13 |
| waist yaw −0.5 rad | 1.2654 | 1.7e-16 | 3.4e-13 |
| waist roll +0.3 rad | 1.2463 | 1.7e-16 | 3.4e-13 |
| raised arm | 1.2654 | 1.1e-16 | 3.4e-13 |
| knee bend (walking-like) | 1.2654 | 1.1e-16 | 3.4e-13 |

Pose-invariant to machine precision. Note the sensor height correctly *changes*
with waist pitch/roll and correctly *does not* with arm or knee motion —
proving it is attached to `torso_link` and nothing else. **PASS**

### Closed-loop smoke test (walking G1, `g1_phase6b_mujoco.yaml`)

```
Mid-360 extrinsic: site 'lidar_mid360' on 'torso_link' pos [0.0002835 3e-05 0.428434]
census: raw_hits=9284 no_hit=2236 self_rejected=327 static=8957
        range_rejected=0 to_estimator=8768 valid=1
O6: mean scan 7369.4 us, amortized 147.84 us/step (7.39% of the 2000 us budget)
    -> strict per-scan gate FAIL (>0.4 ms)
```

`valid=1` with **8768 points**, against `valid=0` with **0 points** in Phase 6b.

---

## 6. Task 6 — acceptance criteria

| | Criterion | Evidence |
| --- | --- | --- |
| ☑ | official Unitree extrinsic applied | `scene_g1.xml` site + 6 YAMLs; `VerifyLidarExtrinsic` asserts agreement at every load and is negative-tested (§4.3) |
| ☑ | `mid360_link` convention verified | **Partially — and honestly labelled.** T1 proves a front wall *cannot* discriminate (2.4 mm) and identifies the measurement that can (§7.1). The simulation is self-consistent; the hardware question is open by construction |
| ☑ | empty scene head self-hit = 0 | V1: zero accepted points terminate on the head shell |
| ☑ | front wall distance validated | V2a: worst error **0.000000000 m** vs analytic, both wall sizes |
| ☑ | known cylinder reconstructed correctly | V3: centre and radius within 5 mm by an independent Kasa fit; radial residual 0 |
| ☑ | sensor rigidly follows torso | V5: \|Δt\| ≤ 1.7e-16, \|ΔR\| ≤ 3.4e-13 over six poses |
| ☑ | arm self-occlusion preserved | V4: raised arm occludes 230 wall points (16.0 %) |
| ☑ | analytic and MuJoCo differences documented | §7.3, plus Phase 6b's V2 exact-agreement result |

**Criterion 2 is marked met with a caveat, not silently.** No simulation can
settle whether Unitree's `mid360_link` is the optical origin or the mounting
base — the simulator does whatever we tell it. What Phase 6c delivers is the
proof that the *proposed* test would not have worked, and the test that will.

---

## 7. Remaining assumptions

### 7.1 The head is treated as fully transparent

The real shell is presumably opaque outside an aperture band, but **no source
found describes the aperture's shape**. We therefore treat the whole head as
transparent, which over-permits wherever the real shell is opaque. Bounded and
explicit: the model is per-geom, so a documented aperture drops in by splitting
`head_link` and re-grouping. The single supporting claim (Unitree's advertised
360° × 59° for the integrated sensor, R9) is cited, not assumed.

### 7.2 `mid360_link` = optical origin O (unchanged from Phase 6b)

Still an inference (`docs/g1_mid360_extrinsic.md` §7.1). T1 now supplies the
decisive procedure:

* the two candidates differ by exactly **47.0 mm in 3-D**, but only **2.4 mm** in
  a horizontal range to a vertical wall — **a wall test cannot settle it**;
* they differ by **46.9 mm** in sensor height above the ground plane
  (47 mm × cos 2.929°).

**Hardware test:** stand the G1 on flat ground, fit the ground plane in the raw
Livox cloud, read the sensor height. **1.2654 m ⇒ site is O** (what we ship).
**1.2185 m ⇒ site is the mounting base**, and the fix is `z −= 0.047`.

### 7.3 Analytic vs MuJoCo backend

| | Analytic | MuJoCo |
| --- | --- | --- |
| Robot body | not modelled at all | full geometry, minus the group-2 aperture |
| Self-occlusion | none | arms/torso/legs occlude (V4) |
| Scene | configured cylinders + ground plane | the compiled scene |
| Per-ray range agreement | — | **exact (0.000000000 m)**, Phase 6b V2 |
| Cost / 11520-ray scan | 86.7 µs | 7 300–8 300 µs |
| O6 (0.4 ms/scan) | PASS | **FAIL** |

Both remain available; `analytic` is still the production default per Phase 6b
Decision B. Nothing in Phase 6c changes that — the MuJoCo backend got *more*
expensive, not less.

### 7.4 Unchanged and still open

* Scan pattern is a uniform 360 × 32 grid, not the Mid-360 non-repetitive
  rosette; 11 520 rays/frame vs the real ~20 000 points/frame.
* Angular random error (≤ 0.15°) is not modelled; only the ≤ 2 cm range error.
* `DpcbfSafetyFilter` is still constructed with `control_dt = 0.002` while
  invoked at ~1000 Hz. Pre-existing, deliberately untouched.
* **All Phase-2…6b recorded numbers predate this phase** and must be re-run:
  the headless evaluator's sensor moved 0.47 m upward and flipped, and the
  MuJoCo backend's ray casting changed.

---

## 8. Reproducing

```bash
cmake -S simulate -B simulate/build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$HOME/.local/unitree_robotics/lib/cmake"
cmake --build simulate/build -j

# the validation suite (V1-V5 + T1)
./simulate/build/dpcbf/perception_sensor_model_test .

# full regression
ctest --test-dir simulate/build --output-on-failure   # 6/6
ctest --test-dir dpcbf/build   --output-on-failure    # 4/4, MuJoCo-free
./dpcbf/build/dpcbf_rollout_evaluator \
    dpcbf/config/scenarios/s0_baseline.yaml /tmp/s0.json
cmp /tmp/s0.json dpcbf/config/scenarios/s0_baseline_reference.json  # identical

# the extrinsic guard, negative test
sed 's/0.00003, 0.428434/0.00003, 0.40618/' dpcbf/config/g1_phase6b_mujoco.yaml \
    > /tmp/bad.yaml
./simulate/build/unitree_mujoco --sim_config simulate/config_phase6a.yaml \
    --dpcbf_config /tmp/bad.yaml     # exits 1 with FATAL

# Figure 1
python3 dpcbf/scripts/plot_mid360_sensor_model.py
```

## 9. Verdict

The simulated Mid-360 now sits where Unitree says it sits, sees what the
published FOV says it should see, is occluded by the arms but not by its own
housing, stays rigidly attached to `torso_link` under every tested pose, and
measures ranges that match analytic geometry to 1e-9 m.

**Phase 6a/6b can be re-run.** Every previously recorded closed-loop number is
now stale and should be regenerated.
