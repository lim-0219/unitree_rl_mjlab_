// Corpora for the P10 safety-state tests.
//
// THREE OF THEM, AND THEY ARE NOT INTERCHANGEABLE. A containment claim is a claim about an
// ABSOLUTE radius, which makes it much more sensitive to which stages a fixture actually runs
// than P9's claims were - P9's gates are about RMSE, identity and association, all of which are
// insensitive to a constant offset in the radius channel. Containment is not.
//
//   A. `MovingScenes()`  - ray-cast scans of MOVING cylinders, driven through the real
//      SegmentCircleDetector and the real KfCircleTracker. This is the shipped pipeline, and it
//      is the corpus the section-12 containment and under-estimation gates are measured on. It
//      is new in P10 because no earlier fixture moves anything through a real detector: P8's
//      scenes are static single frames and P9's streams bypass the detector entirely.
//
//   B. `RebasedTrackingStreams()` - P9's own seven streams, with their radius channel REBASED
//      onto the detector's convention by adding `detection.radius_enlargement_m`. P9's fixture
//      emits `radius_fitted_m = truth_radius + noise`, which no real detector ever produces:
//      the shipped extractor adds a flat 0.25 m (segment_circle_detector.cpp:460). P9 was right
//      to ignore that - the enlargement is a constant and cancels in every difference its gates
//      look at - but it makes the raw streams unusable as an absolute-radius bed, because they
//      describe a detector that is 0.25 m more accurate than the one that exists.
//
//      The rebase changes ONE number per observation and preserves every error realisation:
//      the same seeds, the same Gaussian draws, the same -0.145 m visibility bias, the same
//      occlusion gaps, the same bounce. This is P9's measured error corpus, not a fresh one.
//
//   C. `RawTrackingStreams()` - the same seven streams UNCHANGED, kept so the phase can report
//      what containment would be if the enlargement were ever configured away. That is not a
//      hypothetical: `detection.radius_enlargement_m` is a live config key, and the safety
//      stage's containment turns out to depend on it.
//
// GROUND TRUTH IS CARRIED, NOT RE-DERIVED, in all three - the same rule detection_scenes.h and
// tracking_scenes.h already follow.
#ifndef PERCEPTION_TESTS_FIXTURES_SAFETY_SCENES_H_
#define PERCEPTION_TESTS_FIXTURES_SAFETY_SCENES_H_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "detection_scenes.h"
#include "tracking_scenes.h"

namespace safety_scenes {

using perception::core::kPi;
using perception::core::ProjectedScan;

inline constexpr double kScanPeriodS = tracking_scenes::kScanPeriodS;  // 10 Hz.

// ---------------------------------------------------------------------------------------
// A. Moving ray-cast scenes.
// ---------------------------------------------------------------------------------------

// One cylinder's motion. Constant velocity in the world, which for these scenes is also the
// sensor frame: the sensor sits at the origin and does not move, so the detector's
// kGravityAlignedBase and the tracker's kWorld coincide and the transform between them is the
// identity. Stated rather than assumed - a scenario that silently relabelled a frame would be
// exactly the bug the frame tags exist to catch.
struct Mover {
  Eigen::Vector2d start{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  double radius = 0.25;

  // Frames in [occlude_from, occlude_to) emit no return for this cylinder, which is how a
  // coasting interval is produced without a second body to do the occluding. The safety stage's
  // staleness gate and its age-driven drift term are both only exercised by a coasting track.
  int occlude_from = -1;
  int occlude_to = -1;

  Eigen::Vector2d At(double t) const { return start + velocity * t; }
};

// The exact state a frame was generated from.
struct MovingTruth {
  std::uint32_t truth_id = 0;
  Eigen::Vector2d center{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  double radius = 0.0;
  bool observed = false;
};

struct MovingFrame {
  double stamp_s = 0.0;
  ProjectedScan scan;
  std::vector<MovingTruth> truth;
};

struct MovingScene {
  std::string name;
  std::string purpose;
  double arc_fraction = 1.0;
  std::vector<MovingFrame> frames;
};

inline MovingScene MakeMovingScene(std::string name, std::string purpose,
                                   const std::vector<Mover>& movers, int frame_count,
                                   double arc_fraction) {
  MovingScene scene;
  scene.name = std::move(name);
  scene.purpose = std::move(purpose);
  scene.arc_fraction = arc_fraction;
  scene.frames.reserve(static_cast<std::size_t>(frame_count));

  for (int frame = 0; frame < frame_count; ++frame) {
    MovingFrame out;
    out.stamp_s = static_cast<double>(frame) * kScanPeriodS;

    std::vector<detection_scenes::Cylinder> visible;
    for (std::size_t m = 0; m < movers.size(); ++m) {
      const Mover& mover = movers[m];
      const Eigen::Vector2d center = mover.At(out.stamp_s);

      MovingTruth truth;
      truth.truth_id = static_cast<std::uint32_t>(m + 1);
      truth.center = center;
      truth.velocity = mover.velocity;
      truth.radius = mover.radius;
      truth.observed = !(mover.occlude_from >= 0 && frame >= mover.occlude_from &&
                         frame < mover.occlude_to);
      out.truth.push_back(truth);

      if (truth.observed) {
        visible.push_back({center.x(), center.y(), mover.radius});
      }
    }

    out.scan = detection_scenes::CastScene(visible, {}, arc_fraction, out.stamp_s,
                                           static_cast<std::uint64_t>(frame + 1))
                   .scan;
    scene.frames.push_back(std::move(out));
  }
  return scene;
}

// A single cylinder crossing in front of the sensor at 0.72 m/s - the same speed P9's
// constant_velocity stream uses, so the two corpora are comparable where they overlap.
inline MovingScene CrossingFullArc() {
  Mover mover;
  mover.start = Eigen::Vector2d(-1.4, 2.2);
  mover.velocity = Eigen::Vector2d(0.6, 0.4);
  mover.radius = 0.25;
  return MakeMovingScene("crossing_full_arc",
                         "one 0.25 m cylinder crossing at 0.72 m/s, fully visible", {mover}, 40,
                         1.0);
}

// The same motion with every cylinder cut to half its visible arc: P8's finding 2 and finding 1
// both active, on a MOVING object, through the real fitter. This is the corpus the containment
// gate has to survive - the radius under-estimate and the centre offset peak together here.
inline MovingScene CrossingHalfArc() {
  Mover mover;
  mover.start = Eigen::Vector2d(-1.4, 2.2);
  mover.velocity = Eigen::Vector2d(0.6, 0.4);
  mover.radius = 0.25;
  return MakeMovingScene("crossing_half_arc",
                         "the same crossing with the visible arc halved: P8 findings 1 and 2 "
                         "active on a moving object",
                         {mover}, 40, 0.5);
}

// Two cylinders on converging paths, at the arena's radius extremes, so the corpus contains
// both the smallest and the largest object that exists in it.
inline MovingScene ConvergingPair() {
  Mover a;
  a.start = Eigen::Vector2d(-2.2, 1.2);
  a.velocity = Eigen::Vector2d(0.8, 0.0);
  a.radius = 0.20;

  Mover b;
  b.start = Eigen::Vector2d(2.2, 2.6);
  b.velocity = Eigen::Vector2d(-0.8, 0.0);
  b.radius = 0.30;

  return MakeMovingScene("converging_pair",
                         "0.20 m and 0.30 m cylinders converging at 0.8 m/s: the arena's radius "
                         "extremes",
                         {a, b}, 40, 1.0);
}

// A five-scan occlusion in the middle of the crossing. The only scene in which the safety
// stage's `age_s` is ever non-zero, so it is the only one that exercises the elapsed half of
// the drift horizon and the staleness gate.
inline MovingScene CrossingWithOcclusion() {
  Mover mover;
  mover.start = Eigen::Vector2d(-1.4, 2.2);
  mover.velocity = Eigen::Vector2d(0.6, 0.4);
  mover.radius = 0.25;
  mover.occlude_from = 18;
  mover.occlude_to = 23;
  return MakeMovingScene("crossing_with_occlusion",
                         "a five-scan measurement gap mid-crossing: the coasting path, and the "
                         "only scene where age_s is non-zero",
                         {mover}, 40, 1.0);
}

inline std::vector<MovingScene> MovingScenes() {
  return {CrossingFullArc(), CrossingHalfArc(), ConvergingPair(), CrossingWithOcclusion()};
}

// ---------------------------------------------------------------------------------------
// B / C. P9's streams, raw and rebased.
// ---------------------------------------------------------------------------------------

inline std::vector<tracking_scenes::Stream> RawTrackingStreams() {
  return tracking_scenes::AllStreams();
}

// P9's streams with the radius channel moved onto the real detector's convention. `enlargement`
// is `detection.radius_enlargement_m` from the SHIPPED config, passed in rather than hard-coded
// so the rebase cannot drift away from the number the detector actually adds.
//
// `radius_enclosing_m` is rebased with it purely to keep CircleObservation::Validate() happy and
// the two numbers in a plausible relation; nothing in the safety stage reads it (it does not
// survive tracking - see safety_state_generator.h), so its value cannot affect any result.
inline std::vector<tracking_scenes::Stream> RebasedTrackingStreams(double enlargement) {
  std::vector<tracking_scenes::Stream> streams = tracking_scenes::AllStreams();
  for (tracking_scenes::Stream& stream : streams) {
    stream.name += "_rebased";
    for (tracking_scenes::Frame& frame : stream.frames) {
      for (perception::core::CircleObservation& observation : frame.observations) {
        observation.radius_fitted_m += enlargement;
        observation.radius_enclosing_m += enlargement;
      }
    }
  }
  return streams;
}

// ---------------------------------------------------------------------------------------
// D. The seam-birth scenario - the case section 3 of the P10 brief asks about.
// ---------------------------------------------------------------------------------------
//
// P9 established that an EXISTING track survives a seam crossing: 0 id switches, present in
// 38 of 40 frames, a 2-scan detection gap that coasting covers. That is not the case that
// worries P10. The case that worries P10 is a track that has to be BORN while the object is
// already in seam-affected geometry, because birth needs `tracking.confirm_hits` CONSECUTIVE
// matched scans and a tentative track dies on its first miss - so a two-scan hole that a
// confirmed track shrugs off resets a tentative one to nothing.
//
// The geometry is P8's finding-4 geometry: a 0.25 m cylinder at 5 m subtends about 5.7 degrees,
// so at 1-degree bins its ~6 occupied bins split across the +/-pi seam into two runs and the
// detector drops both whenever neither run reaches `min_group_points`. The object ENTERS the
// scene already close to the seam, rather than sweeping into it from a long detected run, and
// `first_frame_bearing_offset_rad` is how close.
struct SeamBirthFrame {
  double stamp_s = 0.0;
  ProjectedScan scan;
  Eigen::Vector2d truth_center{0.0, 0.0};
  Eigen::Vector2d truth_velocity{0.0, 0.0};
  double truth_radius = 0.0;
  double bearing_rad = 0.0;
};

// The object appears for the first time at `pi - offset` and sweeps through the seam at
// `tangential_speed_mps`. There is nothing before frame 0: the tracker is fresh, so whatever
// the safety stage publishes it publishes about an object it has only just met.
inline std::vector<SeamBirthFrame> SeamBirth(double range_m, double radius_m,
                                             double tangential_speed_mps, int frame_count,
                                             double first_frame_bearing_offset_rad) {
  std::vector<SeamBirthFrame> frames;
  frames.reserve(static_cast<std::size_t>(frame_count));

  const double omega = tangential_speed_mps / range_m;  // rad/s of bearing sweep.
  const double start_bearing = kPi - first_frame_bearing_offset_rad;

  for (int frame = 0; frame < frame_count; ++frame) {
    SeamBirthFrame out;
    out.stamp_s = static_cast<double>(frame) * kScanPeriodS;
    const double bearing = start_bearing + omega * out.stamp_s;
    const double x = range_m * std::cos(bearing);
    const double y = range_m * std::sin(bearing);

    out.truth_center = Eigen::Vector2d(x, y);
    out.truth_radius = radius_m;
    out.bearing_rad = std::atan2(y, x);
    out.truth_velocity = Eigen::Vector2d(-tangential_speed_mps * std::sin(bearing),
                                         tangential_speed_mps * std::cos(bearing));
    out.scan = detection_scenes::CastScene({{x, y, radius_m}}, {}, 1.0, out.stamp_s,
                                           static_cast<std::uint64_t>(frame + 1))
                   .scan;
    frames.push_back(std::move(out));
  }
  return frames;
}

}  // namespace safety_scenes

#endif  // PERCEPTION_TESTS_FIXTURES_SAFETY_SCENES_H_
