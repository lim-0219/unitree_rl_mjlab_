# Livox Mid-360 extrinsic on the Unitree G1 — reconstruction from primary sources

**Status:** RESOLVED for the `rev_1_0` / `mode_*` G1 (the variant this repository
models). One inference remains flagged (§7.1).
**Date:** 2026-07-29
**Scope:** replaces the `nominal mount — extrinsic UNRESOLVED` placeholder used
throughout Phases 0–6b (`dpcbf/README.md` §"UNRESOLVED — Mid-360 extrinsic
measurement").
**Reproduce every number in this document:** see §8.

---

## 1. Executive summary

Unitree publishes the Mid-360 mount directly in the official G1 URDFs as a fixed
joint `mid360_joint`, parent `torso_link`. It was never a missing measurement —
it was a source we had not located.

```yaml
perception:
  lidar:
    parent_body: "torso_link"
    translation_xyz_m: [0.0002835, 0.00003, 0.428434]
    rotation_rpy_rad:  [3.141592653589793, 0.05112069379091391, 0.0]
```

Three facts dominate everything else:

1. **The sensor is mounted upside down.** `roll = π`. Its +Z axis points *down*
   in the robot frame. Confirmed independently by Unitree's URDF and by two
   community LiDAR-odometry projects that had to set `roll = 180°` to make
   FAST-LIO work on this robot.
2. **Therefore the published `-7°…+52°` vertical FOV becomes `+4.07°…-54.93°`
   in the robot frame** (after the additional 2.929° forward-down pitch). The
   sensor looks *down*, which is what makes it useful on a humanoid. Getting the
   roll wrong inverts the sensor's whole purpose.
3. **Unitree ships two mutually inconsistent values.** The disagreement is
   resolved in §5.2: the alternative comes only from URDFs that Unitree's own
   README marks **Deprecated**.

Sanity check on absolute scale: this places the LiDAR origin at **1.2654 m**
above the floor in the default standing pose, 58.2 mm below the head crown,
against Unitree's published standing height of 1.320 m (model crown: 1.3236 m,
+3.6 mm).

> **Not applied to the repository by this document.** Adopting these values
> changes every Phase-6a/6b closed-loop result, so the edit is deliberately left
> as an explicit, reviewed change. The exact one-line edits are in §6.4.

---

## 2. Source table

| # | Title | URL | Date / version | Publisher | Why it is trustworthy |
|---|---|---|---|---|---|
| S1 | `unitree_ros`, `robots/g1_description/g1_29dof_with_hand_rev_1_0.urdf` | https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_with_hand_rev_1_0.urdf | retrieved 2026-07-29, `master` | Unitree Robotics | **Primary.** The manufacturer's own kinematic description of the exact variant this repo models. Contains `mid360_joint` verbatim. |
| S2 | `unitree_ros`, remaining 20 `g1_description` URDFs | https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description | retrieved 2026-07-29 | Unitree Robotics | **Primary.** Cross-variant agreement/disagreement census (§5.2). |
| S3 | `unitree_ros`, `robots/g1_description/README.md` | https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/README.md | retrieved 2026-07-29 | Unitree Robotics | **Primary.** The variant table that marks `g1_23dof`, `g1_29dof`, `g1_29dof_with_hand`, `g1_29dof_lock_waist` as *Deprecated*. This is what breaks the tie. |
| S4 | Livox Mid-360 User Manual v1.0 | https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Mid-360_User_Manual_EN.pdf | 2023-01 | Livox / DJI | **Primary.** Sensor-side ground truth: point-cloud coordinate definition, dimension drawing, specification table. |
| S5 | G1 meshes shipped in this repository | `src/assets/robots/unitree_g1/xmls/assets/*.STL` | vendored 2026-07-28 | Unitree Robotics | **Primary CAD.** Unitree's own geometry; used for the direct measurements in §3 and the figure in §5. |
| S6 | `deepglint/FAST_LIO_LOCALIZATION_HUMANOID` | https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID | retrieved 2026-07-29 | DeepGlint | **Secondary, independent.** A working G1+Mid-360 LiDAR-odometry stack. Corroborates the upside-down mount from field practice, not from Unitree's files. |
| S7 | DeepWiki summary of S6, "Livox Mid360 and G1 Robot Setup" | https://deepwiki.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/4.1-livox-mid360-and-g1-robot-setup | retrieved 2026-07-29 | DeepWiki (third party) | **Tertiary.** Used only to locate S6 and to extract the LiDAR↔internal-IMU offset that is *not* our transform (§7.4). Not relied on for any recommended number. |
| S8 | Unitree G1 overview (Quadruped/QRE docs mirror of Unitree material) | https://docs.quadruped.de/projects/g1/html/g1_overview.html | retrieved 2026-07-29 | Quadruped GmbH (Unitree distributor) | **Secondary.** Source of the statement that the "LIVOX-MID360 laser radar is **integrated into the G1 head**", which decides the enclosure question in §5.4. |
| S9 | Unitree G1 product specifications (standing height 1320 mm) | https://shop.unitree.com/products/unitree-g1 · https://robotsguide.com/robots/unitree-g1 | retrieved 2026-07-29 | Unitree Robotics / IEEE Robots Guide | **Secondary.** Independent absolute-scale check (§5.5). |
| S10 | MuJoCo 3.3.6 `engine_ray.c` | `git show 3.3.6:src/engine/engine_ray.c` (github.com/google-deepmind/mujoco) | tag 3.3.6 | Google DeepMind | **Primary.** Needed to interpret the self-hit measurements in §5.6 correctly. |

Sources deliberately **not** used as evidence: retailer product pages
(robotsinternational, robostore, ghostysky, futurology, stemfinity, rbtx). They
were read and contained no mounting geometry. They are listed here only so a
future reader knows the ground was covered and need not re-tread it.

---

## 3. Measurement table

### 3.1 Robot-side (Unitree)

| Quantity | Value | Unit | Source | Confidence | Notes |
|---|---|---|---|---|---|
| `mid360_joint` origin x | 0.0002835 | m | S1, S2 (16 files) | **Certain** | Verbatim URDF attribute. 0.28 mm ⇒ on the centreline; CAD rounding, not a real offset. |
| `mid360_joint` origin y | 0.00003 | m | S1, S2 (16 files) | **Certain** | 0.03 mm ⇒ zero. |
| `mid360_joint` origin z | 0.428434 | m | S1, S2 (16 files) | **Certain** | In `torso_link`. |
| `mid360_joint` roll | 3.141592653589793 | rad | S1, S2 (16 files) | **Certain** | = π. Upside-down mount. Corroborated by S6. |
| `mid360_joint` pitch | 0.05112069379091391 | rad | S1, S2 (16 files) | **Certain** | = 2.929°, nose-down. |
| `mid360_joint` yaw | 0 | rad | S1, S2 (16 files) | **Certain** | |
| `mid360_joint` parent | `torso_link` | — | S1, S2 (all 21 files) | **Certain** | Not `head_link`, not `pelvis`, and there is **no** `base_link` in the G1 URDF. |
| `mid360_joint` type | `fixed` | — | S1 | **Certain** | No articulation between torso and sensor. |
| `imu_in_torso` origin | (−0.03959, −0.00224, 0.14792) | m | S1 | **Certain** | Byte-identical to this repo's `scene_g1.xml` ⇒ frames are directly compatible. |
| `head_link` origin | (0.0039635, 0, −0.044) | m | S1 | **Certain** | Fixed on `torso_link`; matches this repo's head geom offset exactly. |
| `d435_link` origin | (0.0576235, 0.01753, 0.42987) | m | S1 | **Certain** | Depth camera, 1.4 mm from the LiDAR in z — a shared sensor-deck height (§5.3). |
| G1 standing height | 1320 | mm | S9 | High | Independent scale check. |

### 3.2 Sensor-side (Livox)

| Quantity | Value | Unit | Source | Confidence | Notes |
|---|---|---|---|---|---|
| Body size | 65 × 65 × 60 | mm | S4 p.19 & p.20 | **Certain** | 73.0 mm overall including the connector. |
| Point-cloud origin **O** above mounting base | 47.0 | mm | S4 p.19 (dimension drawing, side view) | **Certain** | The single most important sensor-side number. |
| ⇒ O below the top of the body | 13.0 | mm | derived: 60.0 − 47.0 | **Certain** | |
| Mounting interface | 4 × M3 ↧5, 48.0 × 36.0 mm pattern, bottom face | mm | S4 p.19 | **Certain** | Bottom face is the mounting surface. |
| Vertical FOV | −7 … +52 | deg | S4 p.20 (Specifications) | **Certain** | Matches this repo's `vertical_fov_deg: [-7.0, 52.0]`. |
| Horizontal FOV | 360 | deg | S4 p.20 | **Certain** | |
| Close-proximity blind zone | 0.1 | m | S4 p.20 | **Certain** | Matches `min_range_m: 0.10`. |
| Frame rate | 10 | Hz | S4 p.20 | **Certain** | Matches `scan_rate_hz: 10.0`. |
| Distance random error (1σ) | ≤ 2 cm @ 10 m | m | S4 p.20 | **Certain** | Matches `range_noise_std_m: 0.02`. |
| Angular random error (1σ) | ≤ 0.15 | deg | S4 p.20 | **Certain** | Not currently modelled. |
| Point rate | 200 000 | pts/s | S4 p.20 | **Certain** | ⇒ 20 000 points/frame at 10 Hz; we simulate 11 520 rays/frame (§7.5). |
| Built-in IMU offset in point-cloud coords | (11.0, 23.29, −44.12) | mm | S4 p.13 | **Certain** | **Not our transform** — see §7.4. |
| Mounting orientation constraint | "There is no orientation requirement… Use the bottom surface for mounting." | — | S4 p.8 ("Mounting Notice" §3) | **Certain** | Upside-down mounting is explicitly permitted by Livox. |
| FOV obstruction constraint | "the FOV must not be blocked by an object, including glass" | — | S4 p.8 ("Mounting Notice" §2) | **Certain** | Decisive for §5.4. |

### 3.3 Measured directly from Unitree CAD (this work, source S5)

Measured by loading the shipped binary STLs and expressing every vertex in
`torso_link` (script: `dpcbf/scripts/plot_mid360_extrinsic.py`).

| Quantity | Value | Unit | Confidence | Notes |
|---|---|---|---|---|
| `head_link.STL` z-span | 0.2808 … 0.4866 | m | **Certain** | Head crown at 0.4866. |
| `torso_link_rev_1_0.STL` z-span | −0.0089 … 0.3116 | m | **Certain** | Torso ends well below the LiDAR. |
| Clearance, mounting base plane → head crown | 11.1 | mm | **Certain** | 0.4866 − (0.428434 + 0.047). |
| Nearest head-shell surface from O, forward | 55.6 | mm | **Certain** | vs 32.5 mm sensor half-width ⇒ fits. |
| Nearest head-shell surface from O, up | 58.0 | mm | **Certain** | vs 47 mm to the flipped base ⇒ fits, 11 mm spare. |
| Nearest head-shell surface from O, down | 33.3 | mm | **Certain** | vs 13 mm to the flipped body top ⇒ fits, 20 mm spare. |
| Is O inside `head_link.STL`? | **yes** (7/7 ray-parity votes) | — | **Certain** | Control: `imu_in_torso` correctly tests inside torso, outside head. |

---

## 4. Coordinate frame definition

### 4.1 Robot frame

Unitree's G1 URDF has **no `base_link`**. The kinematic root is **`pelvis`**.
`head_link` exists but is a fixed decorative link — it is *not* the LiDAR parent.

* `torso_link`: x forward, y left, z up. Its origin is the **waist-pitch joint
  frame**, not the visible torso centre — this is why z = 0.428434 (a number
  larger than the torso mesh) is correct rather than suspicious.

### 4.2 Sensor frame

Livox Mid-360 point-cloud frame `O-XYZ` (S4 p.12–13), right-handed, origin at
point **O**, 47.0 mm above the mounting base on the body centreline. Livox
defines point clouds in Cartesian `(x, y, z)` or spherical `(r, θ, φ)` with
`X = r·sinθ·cosφ`, `Y = r·sinθ·sinφ`, `Z = r·cosθ`.

### 4.3 RPY convention

URDF `rpy` is **fixed-axis (extrinsic) X-Y-Z**:

```
R = Rz(yaw) · Ry(pitch) · Rx(roll)
```

This repository's `dpcbf::perception::RpyToRotation`
(`dpcbf/include/dpcbf/perception/lidar_types.h`) is

```cpp
AngleAxisd(rpy.z(), UnitZ) * AngleAxisd(rpy.y(), UnitY) * AngleAxisd(rpy.x(), UnitX)
```

which is the **same** composition. **The URDF numbers can therefore be pasted in
without conversion.** This was checked, not assumed.

### 4.4 Transformation chain

```
world
 └─[floating_base_joint · floating]────────────────────────────► pelvis
     └─[waist_yaw_joint   · revolute, axis (0,0,1)]────────────► waist_yaw_link
         └─[waist_roll_joint · revolute, axis (1,0,0)
                               xyz (−0.0039635, 0, 0.044)]────► waist_roll_link
             └─[waist_pitch_joint · revolute, axis (0,1,0)]───► torso_link
                 ├─[head_joint · fixed
                 │                    xyz (0.0039635, 0, −0.044)]──► head_link
                 ├─[imu_in_torso_joint · fixed
                 │                    xyz (−0.03959, −0.00224, 0.14792)]──► imu_in_torso
                 ├─[d435_joint · fixed
                 │              xyz (0.0576235, 0.01753, 0.42987)
                 │              rpy (0, 0.8307767239493009, 0)]──► d435_link
                 └─[mid360_joint · fixed
                                  xyz (0.0002835, 0.00003, 0.428434)
                                  rpy (π, 0.05112069379091391, 0)]──► mid360_link
```

Three revolute joints (waist yaw/roll/pitch) sit between `pelvis` and the
sensor. The extrinsic is therefore **only** valid relative to `torso_link`; the
pelvis→sensor transform is pose-dependent, which is exactly why the study mounts
on `torso_link` (`dpcbf/README.md` §"LiDAR mount decision").

### 4.5 Resulting rotation

```
R_torso,lidar =
  [  0.998693622   0.000000000  -0.051098431 ]
  [  0.000000000  -1.000000000   0.000000000 ]
  [ -0.051098431   0.000000000  -0.998693622 ]
```

Columns are the sensor axes in `torso_link`:

| Sensor axis | Direction in `torso_link` | Meaning |
|---|---|---|
| **+X** (forward) | (+0.998694, 0, −0.051098) | forward, **2.929° nose-down** |
| **+Y** (left) | (0, −1, 0) | points to the robot's **RIGHT** (flipped) |
| **+Z** (up) | (−0.051098, 0, −0.998694) | points **DOWN** (flipped) |

Quaternion `(w, x, y, z) = (0, 0.999673352, 0, −0.025557564)`, `|q| = 1`.

---

## 5. Evidence

### 5.1 The primary artefact

`g1_29dof_with_hand_rev_1_0.urdf`, lines 570–576 (S1), verbatim:

```xml
  <!-- mid360 -->
  <link name="mid360_link"></link>
  <joint name="mid360_joint" type="fixed">
    <origin xyz="0.0002835 0.00003 0.428434" rpy="3.141592653589793 0.05112069379091391 0"/>
    <parent link="torso_link"/>
    <child link="mid360_link"/>
  </joint>
```

**Why this file and not another variant.** This repository models the 29-DOF
`rev_1_0` G1. Four independent checks, all exact:

| Check | This repo (`scene_g1.xml`) | `..._rev_1_0.urdf` | deprecated `g1_29dof_with_hand.urdf` |
|---|---|---|---|
| torso/waist mesh files | `torso_link_rev_1_0.STL`, `waist_{roll,yaw}_link_rev_1_0.STL` | same | non-`rev_1_0` |
| `waist_pitch_joint` origin | `0 0 0` (no `pos` attribute) | `0 0 0` | `0 0 0.019` ✗ |
| `left_shoulder_pitch_link` | `0.0039563 0.10022 0.24778` | identical | — |
| `imu_in_torso` | `−0.03959 −0.00224 0.14792` | identical | — |

### 5.2 Resolving the two conflicting Unitree values

A census of all 21 `g1_description` URDFs (S2):

| `mid360_joint` origin | Files | Unitree status (S3) |
|---|---|---|
| `xyz 0.0002835 0.00003 **0.428434**`, `rpy **π** 0.05112069379091391 0` | **16** — `g1_23dof_mode_10`, `g1_23dof_rev_1_0`, `g1_29dof_mode_{11,12,13,14,15,16,18}`, `g1_29dof_rev_1_0`, `g1_29dof_rev_1_0_with_inspire_hand_DFQ`, `g1_29dof_with_hand_rev_1_0`, `g1_29dof_lock_waist_rev_1_0`, `g1_29dof_lock_waist_with_hand_rev_1_0`, `g1_comp` | **Up-to-date** |
| `xyz 0.0002835 0.00003 **0.40618**`, `rpy **0** 0.04014257279586953 0` | **5** — `g1_23dof`, `g1_29dof`, `g1_29dof_with_hand`, `g1_29dof_lock_waist`, `g1_dual_arm` | **Deprecated** ×4 + `g1_dual_arm` |

**Which is more trustworthy, and why.** The minority value appears *only* in the
four files Unitree's own README explicitly marks **Deprecated**, plus
`g1_dual_arm` — a legless dual-arm torso product with different hardware, out of
scope here. Every up-to-date legged variant agrees to the last digit. The two
values also differ in a physically coherent way (torso redesign between
revisions: `waist_pitch_joint` moved by 19 mm, `d435_link` by 10 mm, LiDAR by
22.25 mm, and the sensor was inverted), which is consistent with a real hardware
revision rather than a typo.

**Verdict: use the `rev_1_0` / `mode_*` value.** The deprecated value is also
*independently* falsified in §5.6.

### 5.3 Geometry verification (measured, not assumed)

![G1 head geometry with the Mid-360 to scale](figures/g1_mid360_extrinsic_geometry.png)

*Figure 1 — Source: Unitree G1 meshes (S5) + Livox dimension drawing (S4 p.19).
Regenerate with `python3 dpcbf/scripts/plot_mid360_extrinsic.py`.*

Panels 1–2 draw the 65 × 65 × 60 mm Mid-360 body to scale at the URDF pose,
flipped per `roll = π`, so the mounting base is 47 mm *above* O and the body top
13 mm *below* it. The sensor lands **inside the G1 head with its mounting base
11.1 mm below the crown** and 20–30 mm of lateral clearance on all sides. It
fits, snugly and sensibly — a component bolted to a plate just under the crown.

Note the *asymmetry* this explains: the head cavity offers 58.0 mm above O but
only 33.3 mm below it. Right-side-up, the 47 mm to the base would not fit in
33.3 mm. **The upside-down mount is what makes the sensor fit.** That is
independent physical corroboration of `roll = π`.

Panel 3 shows why the roll matters operationally: flipped, the FOV sweeps the
ground ahead of the robot; unflipped, it points at the ceiling.

### 5.4 Is the sensor enclosed?

Yes — and the shipped mesh does not model the window. S8 states the
"LIVOX-MID360 laser radar is **integrated into the G1 head**", and the geometry
above shows it fits entirely inside. But `head_link.STL` is a **closed solid
envelope**: a ray-parity test puts O firmly inside it (7/7 votes), and rays cast
outward cross exactly one surface in the lateral and upward directions
(downward they cross additional internal neck structure). There is no
modelled cavity or window.

A LiDAR cannot operate inside an opaque closed shell, and Livox explicitly
requires that "the FOV must not be blocked by an object, **including glass**"
(S4 p.8). The physical G1 head must therefore have an optical aperture that the
visual STL simply does not represent. **This is a simulation-asset limitation,
not an extrinsic error** — and it is exactly what Phase 6b measured (§5.6).

### 5.5 Absolute-scale cross-check

At the model's default standing pose (`pelvis` at z = 0.793, `torso_link` at
world z = 0.837):

| Landmark | `torso_link` z | World z |
|---|---|---|
| Mid-360 origin O | 0.428434 | **1.2654 m** |
| Mounting base plane | 0.475434 | 1.3124 m |
| Head crown (mesh) | 0.4866 | 1.3236 m |

Unitree's published standing height is **1.320 m** (S9). The model crown lands
at 1.3236 m — **+3.6 mm (0.27 %)**. An independent, end-to-end confirmation
that the mesh scale, the kinematic chain and the LiDAR height are mutually
consistent. A ~1.265 m eye height for a 1.32 m humanoid is exactly right.

### 5.6 Physical validation of the six required criteria

Tool: `dpcbf/tests/perception_extrinsic_probe.cpp`
(`./simulate/build/dpcbf/perception_extrinsic_probe .`). It compiles
`scene_g1.xml` with one 0.30 m obstacle at (2.5, 0), places the sensor at a
candidate extrinsic, and runs the production ray table through both backends.

| Criterion | A: current placeholder (identity) | **B: documented (rev_1_0)** | C: deprecated URDF |
|---|---|---|---|
| Origin, world | (−0.004, 0.000, **0.837**) | (−0.004, 0.000, **1.2654**) | (−0.004, 0.000, 1.2432) |
| **1.** Outside the shell? | **No** — inside waist/torso, 6 mm to surface | **No** — inside the head shell, 33–62 mm | **No** — inside, 11–80 mm |
| **2.** Optical window clear? | No | No *(closed mesh; see §5.4)* | No |
| **3.** FOV blocked by | `waist_roll_link` 9432, `torso_link` 1772 | `torso_link` 11520 | `torso_link` 11520 |
| **4.** Geom AABBs containing origin | **4 of 73** | **2 of 73** | 2 of 73 |
| **5.** Self hits | 11204 / 11454 = **97.8 %** | 11520 / 11520 = **100 %** | 100 % |
| **6.** Obstacle visible (MuJoCo backend) | 0 rays, estimate invalid | 0 rays, estimate invalid | 0 rays, estimate invalid |
| **P.** Analytic backend (production default) | 861 pts, 117 inliers, err 0.0000 m | **8718 pts, 143 inliers, err 0.0000 m** | 319 pts, 65 inliers, err 0.0000 m |

Two conclusions, and they point in different directions — both matter:

* **On the production (analytic) path, B is decisively the best.** It yields
  **8718 scan points versus 319** for the deprecated pose C, because from a
  1.265 m eye height the *flipped* FOV sweeps the ground and the obstacle,
  whereas C looks upward and sees almost nothing. This is strong independent
  evidence for the flip, obtained without reference to any URDF.
* **On the MuJoCo backend, B does not fix the Phase-6b starvation.** It makes it
  formally worse (100 % vs 97.8 % self hits) because the sensor moves from
  inside the waist shell to inside the *head* shell. This is the closed-mesh
  limitation of §5.4, not a defect in the extrinsic. Phase 6b's conclusion
  (Decision B: analytic stays default) is unchanged, and its root-cause analysis
  is now *more* precisely stated: the blocker is the missing optical aperture in
  `head_link.STL`, not the placeholder pose.

---

## 6. Final recommended transform

```yaml
perception:
  lidar:
    parent_body: "torso_link"
    site_name:   "lidar_mid360"
    translation_xyz_m: [0.0002835, 0.00003, 0.428434]
    rotation_rpy_rad:  [3.141592653589793, 0.05112069379091391, 0.0]
```

### 6.1 Where each number comes from

| Number | Origin |
|---|---|
| `0.0002835` | Read verbatim from `<origin xyz=...>` of `mid360_joint` in 16 up-to-date Unitree URDFs (S1, S2). Not derived, not fitted. |
| `0.00003` | Same. |
| `0.428434` | Same. Cross-checked against CAD: places the flipped mounting base 11.1 mm under the head crown (§5.3) and the origin at 1.2654 m for a 1.320 m robot (§5.5). |
| `3.141592653589793` | Same (= π). Corroborated by S6 (`roll = 180°` required for FAST-LIO on this robot), by the CAD packaging asymmetry (§5.3), and by the 27× scan-yield advantage over the unflipped pose (§5.6). |
| `0.05112069379091391` | Same. 2.929° nose-down. |
| `0.0` | Same. |

Nothing here is estimated, averaged, fitted or inferred from photographs. Every
digit is a transcription from a manufacturer file, and every one has at least
one independent physical cross-check.

### 6.2 Equivalent representations

* Quaternion `(w, x, y, z)` = `(0, 0.999673352, 0, −0.025557564)`
* Rotation matrix: §4.5
* MJCF site (quaternion order `w x y z`):
  ```xml
  <site name="lidar_mid360" size="0.02" rgba="1 0.4 0 1"
        pos="0.0002835 0.00003 0.428434"
        quat="0 0.999673352 0 -0.025557564"/>
  ```

### 6.3 Consequence for the FOV configuration

`vertical_fov_deg: [-7.0, 52.0]` is correct **as it stands** and must **not** be
manually negated. It is expressed in the sensor frame (S4 p.20), and the
simulator builds rays in the sensor frame before applying
`pose.rotation_world`, so `roll = π` performs the flip. In the robot frame the
resulting coverage is **+4.071° … −54.929°**. Flipping the numbers *and* setting
`roll = π` would double-invert and silently point the sensor back at the sky.

### 6.4 Exact edits required (not applied by this document)

1. `src/assets/robots/unitree_g1/xmls/scene_g1.xml` — the `lidar_mid360` site:
   `pos="0 0 0" quat="1 0 0 0"` → `pos="0.0002835 0.00003 0.428434"
   quat="0 0.999673352 0 -0.025557564"`.
2. All six `dpcbf/config/*.yaml` carrying a `perception.lidar` block
   (`dpcbf_config.yaml`, `g1_phase6a_{oracle,estimated,estimated_viz}.yaml`,
   `g1_phase6b_mujoco{,_viz}.yaml`) —
   `translation_xyz_m` and `rotation_rpy_rad` as in §6.
3. `dpcbf/config/g1_phase6a_estimated_viz.yaml` and `g1_phase6b_mujoco_viz.yaml`
   — `camera_offset_lidar_m` exists only because the placeholder site was buried
   in the torso. With the real extrinsic it should be re-evaluated and most
   likely reset to `[0, 0, 0]`.
4. Re-run Phases 6a and 6b; every published closed-loop number changes.
5. Replace the `nominal mount — extrinsic UNRESOLVED` label — including the
   visualiser banner in `simulate/src/g1_perception_visualizer.cpp` — with a
   citation of this document.

---

## 7. Remaining uncertainty

Stated plainly; none of these is hidden in the numbers above.

### 7.1 Does `mid360_link` denote the optical origin O or the mounting base? *(the one real open question)*

Unitree never says. `mid360_link` is a bare frame with no visual or collision
geometry. We treat it as **the Livox point-cloud origin O**, on three arguments:

1. **Purpose.** The frame exists so users can TF Livox point clouds into the
   robot. `livox_ros_driver2` publishes in the point-cloud frame, whose origin
   is O by definition (S4 p.12–13). A frame that did not coincide with O would
   be actively misleading for its only use.
2. **Packaging.** With `mid360_link ≡ O`, the flipped mounting base lands 11.1 mm
   below the head crown — a plate bolted just under the crown. The alternative
   puts O at 0.3814 and leaves a 58 mm void above the base.
3. **Consistency.** `d435_link` sits at z = 0.42987, within **1.4 mm** of the
   LiDAR — a shared optical-deck height. Optical centres, not mounting faces.

**Confidence: high, but this is an inference, not a citation.** If it is wrong,
the correction is exactly one number: O would be at
`z = 0.428434 − 0.047 = 0.381434`, i.e. `translation_xyz_m: [0.0002835,
0.00003, 0.381434]`, rotation unchanged. **Resolve it in 30 seconds on real
hardware:** put the robot in front of a flat wall at a measured distance, and
compare the mean forward range in the raw Livox point cloud against the
tape-measured wall distance. A systematic 47 mm bias means `mid360_link` is the
mounting base.

### 7.2 Which physical G1 this repository corresponds to

The extrinsic is correct for the `rev_1_0` / `mode_*` hardware. It is
**wrong by 22.25 mm and a full 180° roll** for the deprecated pre-`rev_1_0` G1.
The evidence that this repo is `rev_1_0` is strong (§5.1) but is drawn from the
*model files*, not from the serial number of the machine in the lab. **Anyone
using this on real hardware should confirm the robot's `mode_machine` ID** (app:
Device → Data → Robot → Machine Type; S3) — expect 5 for
`g1_29dof_with_hand_rev_1_0`.

### 7.3 The optical aperture is not modelled

`head_link.STL` is a closed envelope with no window, so the MuJoCo ray-casting
backend sees 100 % self hits at the true extrinsic (§5.6). The real aperture's
size, shape and angular extent are **undocumented in every source found**. Until
that is resolved, the MuJoCo backend cannot be used at the true mount without
either excluding the head geoms or authoring an aperture. This is now the single
blocking item for Phase-6b fidelity work — a change from the previous belief
that the unresolved extrinsic was the blocker.

### 7.4 A near-miss that must not be confused with this transform

S6/S7 publish `extrinsic_T = [−0.011, −0.02329, 0.04412]` with identity
rotation. That is the **Livox-internal IMU → point-cloud** offset — the exact
negative of the manual's `(11.0, 23.29, −44.12) mm` (S4 p.13) — used to fuse the
sensor's own IMU inside FAST-LIO. It is **not** a robot mounting transform and
must never be substituted for one. Recorded here because the number is
superficially plausible and easy to misuse.

### 7.5 Items outside this document's scope, still open

* **Scan pattern.** We simulate a uniform 360 × 32 azimuth/elevation grid
  (11 520 rays/frame). The real Mid-360 is a *non-repetitive rosette* at
  ~20 000 points/frame (S4 p.20). Coverage statistics differ.
* **Angular noise.** The ≤ 0.15° angular random error (S4 p.20) is not modelled;
  only the ≤ 2 cm range error is.
* **Manufacturing tolerance.** The URDF is nominal CAD. Unit-to-unit mounting
  tolerance is not published and is not captured by any figure here.
* **Timing/latency.** Sensor→host latency is not characterised.
* **`control_dt` discrepancy.** Pre-existing and unrelated: `DpcbfSafetyFilter`
  is constructed with `control_dt = 0.002` but invoked at ~1000 Hz.

---

## 8. Reproducing this document

```bash
# 1. the primary artefact (16 up-to-date URDFs agree; 5 deprecated disagree)
curl -sSL https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/\
robots/g1_description/g1_29dof_with_hand_rev_1_0.urdf | grep -A4 'mid360_joint'

# 2. the sensor-side drawing (point O, 47.0 mm above the mounting base, p.19)
curl -sSL -o mid360.pdf https://terra-1-g.djicdn.com/\
65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Mid-360_User_Manual_EN.pdf
pdftoppm -f 21 -l 22 -r 150 -png mid360.pdf page     # doc p.19-20 = pdf p.21-22

# 3. CAD measurements + Figure 1
python3 dpcbf/scripts/plot_mid360_extrinsic.py

# 4. the six physical validation criteria (table in 5.6)
cmake --build simulate/build -j --target perception_extrinsic_probe
./simulate/build/dpcbf/perception_extrinsic_probe .
# ...or probe an arbitrary candidate:
./simulate/build/dpcbf/perception_extrinsic_probe . 0.0002835 0.00003 0.428434 3.14159265358979 0.0511206937909139 0
```

## 9. Acceptance criteria — answered

* **Why is this LiDAR mounted here?** It is packaged inside the G1 head, bolted
  upside down to a plate 11.1 mm below the crown, so its −7…+52° vertical FOV
  becomes +4.07…−54.93° and sweeps the ground ahead of the robot (§5.3, §5.6).
* **Where did these xyz/rpy numbers come from?** Transcribed verbatim from the
  `mid360_joint` fixed joint in Unitree's own G1 URDFs (§5.1).
* **Which official document supports them?** S1/S2 (Unitree `unitree_ros`
  `g1_description`), tie-broken by S3 (Unitree's variant table), with sensor-side
  geometry from S4 (Livox Mid-360 User Manual).
* **Could someone independently reproduce these values?** Yes — §8 is four
  commands, of which two hit the manufacturers' servers directly.
