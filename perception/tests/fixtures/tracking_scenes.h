// Synthetic detection streams for the P9 tracking tests.
//
// WHY THESE ARE SYNTHESISED RATHER THAN CAST. The detection fixtures next door
// (detection_scenes.h) ray-cast analytic geometry, because the question P8 had to answer was
// "does the fitter recover a circle from the bearings a real scan actually has". The question here
// is different: given a KNOWN true trajectory and a KNOWN measurement error distribution, does the
// estimator recover the trajectory and keep its identities straight. That needs error statistics
// chosen by the fixture, not error statistics that fall out of a raycast - otherwise the velocity
// RMSE gate is measuring the detector, and a regression in either stage would move it.
//
// The one scenario in this phase that DOES need the real detector - the +/-pi seam gap - is built
// in the test itself from detection_scenes.h, precisely because its whole subject is a detector
// behaviour and synthesising it would beg the question.
//
// GROUND TRUTH IS CARRIED, NOT RE-DERIVED. Every frame stores the exact state each obstacle was
// generated from, so the acceptance metrics compare against the generator's numbers rather than
// against something re-estimated from the same observations.
//
// ERROR MODEL. Each observation gets a zero-mean Gaussian centre error and a radius error that is
// Gaussian PLUS an optional systematic offset. The systematic part is not decoration: P8 measured
// the sqrt(3)/3 fitted radius to be biased SMALL by 0.006-0.145 m as a function of visible arc,
// always in the same direction, and a fixture with only zero-mean radius noise would let a tracker
// that cannot cope with bias pass every gate here.
//
// SIGMAS ARE REPORTED TRUTHFULLY unless a stream says otherwise: sigma_center_m and
// sigma_radius_m carry the standard deviation the noise was actually drawn from. That makes this
// corpus the control case for the NIS calibration - a filter fed honest sigmas over honest noise
// must show a mean NIS near 1.0, and any departure is the estimator's, not the fixture's. Note
// that a SYSTEMATIC radius offset is deliberately NOT folded into the reported sigma: a detector
// does not know its own bias, which is exactly the situation the calibrated floor exists for.
#ifndef PERCEPTION_TESTS_FIXTURES_TRACKING_SCENES_H_
#define PERCEPTION_TESTS_FIXTURES_TRACKING_SCENES_H_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "perception/core/contracts/detection_2d.h"

namespace tracking_scenes {

using perception::core::CircleObservation;
using perception::core::FrameId;
using perception::core::kPi;

inline constexpr double kScanPeriodS = 0.1;  // 10 Hz, sensor.scan_rate_hz.

// Deterministic Gaussian noise. splitmix64 plus Box-Muller, written out rather than taken from
// <random>: the standard library's distributions are not specified to produce identical values
// across implementations, and a fixture whose numbers depend on the standard library version
// cannot back a committed acceptance figure.
class Rng {
 public:
  explicit Rng(std::uint64_t seed) : state_(seed) {}

  double Uniform() {
    // splitmix64.
    std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z = z ^ (z >> 31);
    // 53 bits into [0, 1). The +0.5 keeps the value strictly inside the interval, because
    // Box-Muller takes a logarithm of it.
    return (static_cast<double>(z >> 11) + 0.5) * (1.0 / 9007199254740992.0);
  }

  double Gaussian() {
    const double u1 = Uniform();
    const double u2 = Uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
  }

 private:
  std::uint64_t state_;
};

// The exact state an observation was generated from.
struct TruthSample {
  std::uint32_t truth_id = 0;  // The generator's identity, NOT a track id.
  Eigen::Vector2d center{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  double radius = 0.0;
  bool observed = false;  // False during an occlusion or seam gap.
};

struct Frame {
  double stamp_s = 0.0;
  std::vector<CircleObservation> observations;
  std::vector<TruthSample> truth;
};

struct Stream {
  std::string name;
  std::string purpose;
  std::vector<Frame> frames;
};

// One obstacle's generated trajectory. Constant velocity, because that is the model the ported
// filter assumes and the corpus should test the estimator rather than a manoeuvre the estimator
// was never given a chance to represent.
struct Body {
  Eigen::Vector2d start{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  double radius = 0.25;

  // Reported measurement sigmas, and the distribution the errors are actually drawn from.
  // 0.03 m and 0.015 m are the training-time noise envelope's sigma_pos and sigma_r (section
  // 12), which is the right scale for this corpus for two reasons: it is what the RL policy was
  // trained to tolerate, and it sits at the low end of what the real detector actually emits
  // (0.067-0.216 m over the P8 corpus), so the fixture is optimistic-but-not-fictional rather
  // than claiming a precision the detector never reaches.
  double sigma_center_m = 0.03;
  double sigma_radius_m = 0.015;

  // Systematic radius offset, metres, applied from `bias_from_frame` onward. This is how a
  // mid-track visibility change is expressed: the object does not change, the amount of it that
  // is visible does.
  //
  // WHAT THIS MODELS, PRECISELY, because the sign is easy to get wrong. P8's finding 2 measured
  // the DE-ENLARGED radius - `radius_fitted_m - radius_enlargement_m`, upstream's `true_radius`
  // - to be biased small by 0.006-0.145 m as a function of visible arc. The raw
  // `radius_fitted_m` that the tracker actually associates on is that value plus the constant
  // 0.25 m enlargement, so against ground truth it OVER-estimates; measuring it without
  // de-enlarging first reports the enlargement as an error, which is exactly why P8 does not.
  //
  // None of that changes what the association gate sees. The enlargement is a constant, so it
  // cancels in the difference between two frames, and the frame-to-frame variation of
  // `radius_fitted_m` IS the variation of the bias. Applying the step to the emitted radius here
  // is therefore the faithful model, and -0.145 m is P8's worst measured case.
  double radius_bias_m = 0.0;
  int bias_from_frame = 0;

  // A velocity reversal at `bounce_frame`, which is what obstacles in this arena actually do at
  // a wall. Not in section 10's list of streams; it is here because the process-noise retune
  // needed a manoeuvre to size the rate channel against, and a corpus of nothing but exactly
  // constant velocity would have justified driving the rate variance to zero.
  int bounce_frame = -1;
  Eigen::Vector2d velocity_after{0.0, 0.0};

  // A systematic CENTRE offset applied over the same window. Short arcs move the centre estimate
  // as well as the radius estimate, and they do it at the same time, so a fixture that offset
  // only the radius would understate the association stress by exactly the amount that matters.
  Eigen::Vector2d center_bias_m{0.0, 0.0};

  // Frames in [drop_from, drop_to) produce no observation for this body.
  int drop_from = -1;
  int drop_to = -1;
};

inline CircleObservation MakeObservation(const Eigen::Vector2d& center, double radius,
                                        double stamp_s, int primitive_index, double sigma_center,
                                        double sigma_radius) {
  CircleObservation observation;
  observation.center = center;
  observation.radius_fitted_m = radius;
  // The enclosing radius is what the safety stage may prefer (open Q11); it is filled with a
  // plausible value rather than left at zero because CircleObservation::Validate() requires it
  // positive, and a fixture that only satisfied the fields under test would let a contract
  // regression through.
  observation.radius_enclosing_m = radius * 1.05;
  observation.arc_extent_rad = 0.2;
  observation.arc_length_m = radius * 1.0;
  observation.fit_residual_m = sigma_radius;
  observation.range_to_center_m = center.norm();
  observation.sigma_center_m = sigma_center;
  observation.sigma_radius_m = sigma_radius;
  observation.point_count = 12;
  observation.primitive_index = primitive_index;
  observation.front_visible = true;
  observation.back_visible = true;
  observation.stamp_s = stamp_s;
  // kWorld, not the detector's kGravityAlignedBase: the tracker's contract is world-frame, and
  // the transform is the pipeline's job (a later phase). Handing the tracker a base-frame
  // observation is a real failure mode, so it has its own negative test rather than being
  // smuggled in here.
  observation.frame = FrameId::kWorld;
  return observation;
}

inline Stream MakeStream(std::string name, std::string purpose, const std::vector<Body>& bodies,
                         int frame_count, std::uint64_t seed) {
  Rng rng(seed);
  Stream stream;
  stream.name = std::move(name);
  stream.purpose = std::move(purpose);
  stream.frames.reserve(static_cast<std::size_t>(frame_count));

  for (int frame = 0; frame < frame_count; ++frame) {
    Frame out;
    out.stamp_s = static_cast<double>(frame) * kScanPeriodS;

    for (std::size_t b = 0; b < bodies.size(); ++b) {
      const Body& body = bodies[b];

      Eigen::Vector2d truth_center = body.start + body.velocity * out.stamp_s;
      Eigen::Vector2d truth_velocity = body.velocity;
      if (body.bounce_frame >= 0 && frame >= body.bounce_frame) {
        const double bounce_stamp = static_cast<double>(body.bounce_frame) * kScanPeriodS;
        truth_center = body.start + body.velocity * bounce_stamp +
                       body.velocity_after * (out.stamp_s - bounce_stamp);
        truth_velocity = body.velocity_after;
      }

      TruthSample sample;
      sample.truth_id = static_cast<std::uint32_t>(b + 1);
      sample.center = truth_center;
      sample.velocity = truth_velocity;
      sample.radius = body.radius;

      const bool dropped =
          body.drop_from >= 0 && frame >= body.drop_from && frame < body.drop_to;
      sample.observed = !dropped;

      // The noise is drawn WHETHER OR NOT the observation is emitted, so that a dropped frame
      // does not shift every later draw and turn an occlusion-gap stream into a different
      // realisation of the same trajectory. Two streams differing only in their gap must remain
      // comparable.
      const double nx = rng.Gaussian();
      const double ny = rng.Gaussian();
      const double nr = rng.Gaussian();

      out.truth.push_back(sample);
      if (dropped) continue;

      const bool biased = frame >= body.bias_from_frame;
      Eigen::Vector2d measured = truth_center;
      measured.x() += nx * body.sigma_center_m;
      measured.y() += ny * body.sigma_center_m;
      if (biased) measured += body.center_bias_m;

      double measured_radius = body.radius + nr * body.sigma_radius_m;
      if (biased) measured_radius += body.radius_bias_m;
      // A radius must stay positive to be a legal observation at all; the clamp is far below any
      // value these streams generate and exists so a pathological draw cannot make the fixture
      // itself invalid.
      if (measured_radius < 0.05) measured_radius = 0.05;

      out.observations.push_back(MakeObservation(measured, measured_radius, out.stamp_s,
                                                 static_cast<int>(out.observations.size()),
                                                 body.sigma_center_m, body.sigma_radius_m));
    }
    stream.frames.push_back(std::move(out));
  }
  return stream;
}

// ---------------------------------------------------------------------------------------
// The streams. Speeds stay at or under 0.8 m/s and radii inside 0.20-0.30 m, matching
// dpcbf_config.yaml's `speed_range` and `radius_range`, so the corpus is this arena's problem.
// ---------------------------------------------------------------------------------------

inline Stream ConstantVelocity() {
  Body body;
  body.start = Eigen::Vector2d(3.0, -2.0);
  body.velocity = Eigen::Vector2d(0.6, 0.4);  // 0.72 m/s.
  body.radius = 0.25;
  return MakeStream("constant_velocity",
                    "one obstacle at 0.72 m/s: the velocity and position RMSE gates", {body}, 40,
                    0x51ED0001ULL);
}

// PERPENDICULAR crossing with a deliberate time offset, so the two tracks genuinely pass inside
// the association gate. Two parallel paths 0.6 m apart would test nothing: every cross-pair cost
// would exceed min_correspondence_cost and no swap would be POSSIBLE, which is not the same as a
// tracker declining to swap.
inline Stream CrossingPair() {
  Body a;
  a.start = Eigen::Vector2d(-1.6, 0.0);
  a.velocity = Eigen::Vector2d(0.8, 0.0);
  a.radius = 0.22;

  Body b;
  b.start = Eigen::Vector2d(0.30, -1.6);
  b.velocity = Eigen::Vector2d(0.0, 0.8);
  b.radius = 0.26;

  return MakeStream("crossing_pair",
                    "two obstacles crossing perpendicular paths within the association gate: the "
                    "ID-switch gate",
                    {a, b}, 44, 0x51ED0002ULL);
}

inline Stream OcclusionGap() {
  Body body;
  body.start = Eigen::Vector2d(-3.0, 1.0);
  body.velocity = Eigen::Vector2d(0.5, -0.3);
  body.radius = 0.28;
  body.drop_from = 15;
  body.drop_to = 20;  // Five scans, 0.5 s.
  return MakeStream("occlusion_gap",
                    "a five-scan measurement gap: coasting must survive it and reacquire the same "
                    "id",
                    {body}, 40, 0x51ED0003ULL);
}

inline Stream RadiusJitter() {
  Body body;
  body.start = Eigen::Vector2d(2.0, 2.0);
  body.velocity = Eigen::Vector2d(-0.4, -0.2);
  body.radius = 0.24;
  body.sigma_radius_m = 0.05;  // Far noisier than the training envelope's 0.015.
  return MakeStream("radius_jitter",
                    "heavy zero-mean radius noise: the radius filter must not destabilise the "
                    "position track",
                    {body}, 40, 0x51ED0004ULL);
}

// P8's FINDING 2, as a stream. Mid-track the visible arc shrinks: the fitted radius steps down by
// 0.145 m - the worst case P8 measured - and the centre estimate steps off by 0.15 m, section
// 12's own half-arc bound. Both at once, because both are driven by the same shortening arc.
// Nothing about the object changes.
inline Stream VisibilityChange() {
  Body body;
  body.start = Eigen::Vector2d(-2.5, 0.5);
  body.velocity = Eigen::Vector2d(0.8, 0.0);
  body.radius = 0.30;
  body.radius_bias_m = -0.145;
  body.center_bias_m = Eigen::Vector2d(0.106, 0.106);  // 0.15 m, split across both axes.
  body.bias_from_frame = 20;
  return MakeStream("visibility_change",
                    "arc visibility collapses mid-track: -0.145 m radius step and 0.15 m centre "
                    "step, no real change in the object",
                    {body}, 40, 0x51ED0005ULL);
}

// FINDING 2 AND FINDING 4 TOGETHER, which is the case neither one alone describes. The track
// loses five scans (a seam or occlusion gap), and when it comes back its arc visibility has
// changed - which is the physically expected pairing, since whatever moved the object out of
// view also changed which side of it faces the sensor. The reacquisition residual is then the
// coasting prediction error PLUS the centre bias, with the radius step on top.
inline Stream ReacquireWithVisibilityChange() {
  Body body;
  body.start = Eigen::Vector2d(-2.0, -1.0);
  body.velocity = Eigen::Vector2d(0.8, 0.0);
  body.radius = 0.30;
  body.drop_from = 18;
  body.drop_to = 23;
  body.radius_bias_m = -0.145;
  body.center_bias_m = Eigen::Vector2d(0.15, 0.0);
  body.bias_from_frame = 23;  // From the first scan after the gap.
  return MakeStream("reacquire_with_visibility_change",
                    "a five-scan gap followed by a full-magnitude visibility change: the worst "
                    "association case the two P8 findings imply together",
                    {body}, 40, 0x51ED0006ULL);
}

// A wall bounce: the velocity reverses in one scan interval. No Gaussian rate noise can predict
// that, so the point is not that the filter follows it - it is that the filter RECOVERS, and how
// many scans it takes. That recovery time is what sizes process_rate_variance, and it is the
// reason this stream exists: tuned against constant velocity alone, the rate channel's best value
// would be zero.
inline Stream Bounce() {
  Body body;
  body.start = Eigen::Vector2d(-1.5, 0.5);
  body.velocity = Eigen::Vector2d(0.8, 0.0);
  body.bounce_frame = 20;
  body.velocity_after = Eigen::Vector2d(-0.8, 0.0);  // A full reversal, the worst case.
  body.radius = 0.25;
  return MakeStream("bounce",
                    "a full velocity reversal mid-track: the recovery time that sizes the rate "
                    "process noise",
                    {body}, 45, 0x51ED0007ULL);
}

inline std::vector<Stream> AllStreams() {
  return {ConstantVelocity(), CrossingPair(),     OcclusionGap(),
          RadiusJitter(),     VisibilityChange(), ReacquireWithVisibilityChange(),
          Bounce()};
}

}  // namespace tracking_scenes

#endif  // PERCEPTION_TESTS_FIXTURES_TRACKING_SCENES_H_
