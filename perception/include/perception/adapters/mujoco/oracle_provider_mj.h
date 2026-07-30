// OracleProviderMj - the ground-truth obstacle source, formalized.
//
// WHAT IT IS. The DPCBF stack in this repository has always been fed by an oracle:
// kinematic mocap cylinders owned by dpcbf::DynamicObstacleManager, converted to
// dpcbf::ObstacleState inline inside the simulator's joystick axis-filter lambda. This class
// is that read, given a name, a contract and a test surface. It reads
// DynamicObstacleManager::Snapshot() plus the robot's own ground-truth pose out of mjData and
// emits core::OracleObstacleState stamped with MuJoCo sim time.
//
// THE ONE DELIBERATE DEPENDENCY EXCEPTION. Per the architecture doc's dependency rules this
// is the only module in perception/ permitted to include BOTH a MuJoCo header and a dpcbf
// header, and the dpcbf header it may include is dpcbf/dynamic_obstacles.h and only that.
// It must never name dpcbf::ObstacleState - that type belongs to adapters/dpcbf - and it
// must never know the estimator exists (oracle -> estimator is a forbidden edge, because an
// oracle that can see the estimate is no longer a ground truth).
//
// LOCKING. Snapshot() takes DynamicObstacleManager's own mutex and returns a copy; that is
// the entire locking discipline for the obstacle leg, unchanged from the inline code this
// replaces. No second lock is introduced and none is needed. The mjData reads for the robot
// pose are the caller's responsibility to scope, exactly as they were before: in the
// simulator they happen on the bridge thread, which shares the pre-existing, documented
// unsynchronised-read hazard with the ReadRobotGroundTruth call sitting next to them. This
// phase does not add a single new mjData read on that thread - see the note on
// SampleObstacles below.
//
// PASS-THROUGH IS EXACT AND UNFILTERED. Every obstacle the snapshot returns is emitted: no
// cap, no validity filter, no reordering. The DPCBF filter does its own top-k constraint
// selection downstream. See obstacle_source_selector.h for why the dpcbf_adapter config
// knobs that look like they belong here deliberately are not wired in yet.
#ifndef PERCEPTION_ADAPTERS_MUJOCO_ORACLE_PROVIDER_MJ_H_
#define PERCEPTION_ADAPTERS_MUJOCO_ORACLE_PROVIDER_MJ_H_

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "dpcbf/dynamic_obstacles.h"
#include "perception/core/contracts/obstacles.h"
#include "perception/integration/perception_config.h"

namespace perception::adapters::mujoco {

// The robot's own ground-truth planar state, in the terms DPCBF and the evaluator think in.
//
// WHY THIS IS AN ADAPTER-LOCAL STRUCT AND NOT A core/contracts TYPE. Adding a contract means
// adding a dump record type, a codec, round-trip tests and a schema bump - that is P1/P2
// work, and this phase has no producer or consumer that needs it serialized. It is also
// deliberately NOT dpcbf::RobotState: naming that type here would put a
// dpcbf_safety_filter.h include in the one module that must not have one. The fields carry
// the same five numbers, computed identically. A later phase (the P11 evaluator, which does
// need to log oracle-vs-estimate robot-relative geometry) is the right place to promote this
// into a real contract.
struct OracleRobotPose {
  bool valid = false;
  double stamp_s = 0.0;  // MuJoCo sim time, mjData::time.

  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;

  // Body-local planar velocity, i.e. mj_objectVelocity with flg_local = 1.
  double sagittal_velocity_mps = 0.0;
  double lateral_velocity_mps = 0.0;
};

// One oracle observation: every obstacle plus the robot that observes them, at one sim time.
struct OracleSample {
  double stamp_s = 0.0;
  std::vector<core::OracleObstacleState> obstacles;
  OracleRobotPose robot;
};

class OracleProviderMj {
 public:
  OracleProviderMj() = default;

  // Attaches the obstacle source. Deliberately separate from BindModel and deliberately
  // model-free: in oracle mode the obstacle leg needs no mjModel at all, so it can be wired
  // once at startup before any thread runs, instead of racing the model-load path. Pass
  // nullptr to detach.
  void BindObstacleSource(const ::dpcbf::DynamicObstacleManager* manager) {
    manager_ = manager;
  }

  // Resolves the robot base body named by frames.base_body ("pelvis" by default - the same
  // body DPCBF uses, which is NOT the Mid-360's torso_link parent). Only the robot-pose leg
  // needs this; the obstacle leg works without it.
  bool BindModel(const mjModel* model, const integration::FramesConfig& frames,
                 std::string& error);

  // Drops the model binding only; the obstacle source survives a model reload, because it
  // never depended on the model in the first place.
  void UnbindModel() { base_body_id_ = -1; }

  bool has_obstacle_source() const { return manager_ != nullptr; }
  bool has_model_binding() const { return base_body_id_ >= 0; }
  int base_body_id() const { return base_body_id_; }

  // The obstacle leg. Appends nothing and allocates nothing beyond growing `out`, which is
  // cleared first. `stamp_s` is the caller's sim time - taken as a parameter rather than read
  // from mjData on purpose: this is the call the simulator's bridge thread makes at joystick
  // rate, and it must not add an mjData dereference to that thread. The simulator passes
  // mjData::time, which is what FrameProviderMj stamps its snapshots with too, so both legs
  // of the subsystem share one clock.
  void SampleObstacles(double stamp_s, std::vector<core::OracleObstacleState>& out) const;

  // The robot-pose leg. Requires BindModel. `data` must be consistent - the caller owns
  // whatever lock that takes.
  OracleRobotPose SampleRobot(const mjModel* model, const mjData* data) const;

  // Both legs at one stamp, for the dump/evaluator path. Uses data->time as the stamp.
  void Sample(const mjModel* model, const mjData* data, OracleSample& out) const;

 private:
  const ::dpcbf::DynamicObstacleManager* manager_ = nullptr;
  int base_body_id_ = -1;
};

}  // namespace perception::adapters::mujoco

#endif  // PERCEPTION_ADAPTERS_MUJOCO_ORACLE_PROVIDER_MJ_H_
