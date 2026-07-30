#include "perception/core/diagnostics/dump_format.h"

namespace perception::core::diagnostics {

RecordType RecordTypeFromString(const std::string& name) {
  for (uint32_t value = 1; value < kRecordTypeCount; ++value) {
    const RecordType candidate = static_cast<RecordType>(value);
    if (name == ToString(candidate)) return candidate;
  }
  return RecordType::kUnknown;
}

bool IsKnownRecordType(uint32_t value) {
  return value != 0 && value < kRecordTypeCount;
}

uint32_t PackRetainedStages(const RetainedStages& stages) {
  uint32_t mask = 0;
  if (stages.raw_cloud) mask |= kRetainRawCloud;
  if (stages.deskewed_cloud) mask |= kRetainDeskewedCloud;
  if (stages.gravity_aligned_cloud) mask |= kRetainGravityAlignedCloud;
  if (stages.labeled_cloud) mask |= kRetainLabeledCloud;
  if (stages.projected_scan) mask |= kRetainProjectedScan;
  if (stages.clusters) mask |= kRetainClusters;
  if (stages.primitives) mask |= kRetainPrimitives;
  if (stages.observations) mask |= kRetainObservations;
  if (stages.tracks) mask |= kRetainTracks;
  return mask;
}

RetainedStages UnpackRetainedStages(uint32_t mask) {
  RetainedStages stages;
  stages.raw_cloud = (mask & kRetainRawCloud) != 0;
  stages.deskewed_cloud = (mask & kRetainDeskewedCloud) != 0;
  stages.gravity_aligned_cloud = (mask & kRetainGravityAlignedCloud) != 0;
  stages.labeled_cloud = (mask & kRetainLabeledCloud) != 0;
  stages.projected_scan = (mask & kRetainProjectedScan) != 0;
  stages.clusters = (mask & kRetainClusters) != 0;
  stages.primitives = (mask & kRetainPrimitives) != 0;
  stages.observations = (mask & kRetainObservations) != 0;
  stages.tracks = (mask & kRetainTracks) != 0;
  return stages;
}

const char* RetainedStageBitName(uint32_t bit_index) {
  switch (bit_index) {
    case 0: return "raw_cloud";
    case 1: return "deskewed_cloud";
    case 2: return "gravity_aligned_cloud";
    case 3: return "labeled_cloud";
    case 4: return "projected_scan";
    case 5: return "clusters";
    case 6: return "primitives";
    case 7: return "observations";
    case 8: return "tracks";
    default: break;
  }
  return "unknown";
}

RecordType RetainedStageBitRecordType(uint32_t bit_index) {
  switch (bit_index) {
    case 0: return RecordType::kTimedPointCloud;
    case 1: return RecordType::kDeskewedPointCloud;
    case 2: return RecordType::kGravityAlignedCloud;
    case 3: return RecordType::kLabeledPointCloud;
    case 4: return RecordType::kProjectedScan;
    case 5: return RecordType::kCluster2D;
    case 6: return RecordType::kFittedPrimitive2D;
    case 7: return RecordType::kCircleObservation;
    case 8: return RecordType::kTrackState2D;
    default: break;
  }
  return RecordType::kUnknown;
}

}  // namespace perception::core::diagnostics
