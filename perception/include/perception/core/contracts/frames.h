// Frame identity for every perception contract.
//
// See perception/docs/architecture.md, "Internal data contracts". Every value type that
// carries geometry names the frame it is expressed in, so a stage cannot silently consume
// a cloud from the wrong frame.
#ifndef PERCEPTION_CORE_CONTRACTS_FRAMES_H_
#define PERCEPTION_CORE_CONTRACTS_FRAMES_H_

namespace perception::core {

enum class FrameId {
  kUnknown = 0,
  kSensor,             // Livox Mid-360 point-cloud frame O-XYZ (upside-down on the G1).
  kBase,               // The extrinsic's parent body. On the G1 this is `torso_link`.
  kGravityAlignedBase, // kBase with roll/pitch removed. Not produced by the P4 slice.
  kWorld,              // MuJoCo world frame.
};

inline const char* ToString(FrameId frame) {
  switch (frame) {
    case FrameId::kSensor:             return "sensor";
    case FrameId::kBase:               return "base";
    case FrameId::kGravityAlignedBase: return "gravity_aligned_base";
    case FrameId::kWorld:              return "world";
    case FrameId::kUnknown:            break;
  }
  return "unknown";
}

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_FRAMES_H_
