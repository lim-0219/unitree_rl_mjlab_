// Synthetic LabeledPointCloud scenes for the P7 projection tests and golden capture.
//
// WHY SYNTHETIC, AND WHY THEY CARRY LABELS ALREADY. P7 consumes the output of P6's
// segmenter, and P6 does not exist. These scenes are therefore hand-built clouds that
// already carry ground / non-ground / self / invalid labels - a fixture generator, not a
// segmenter. Nothing here decides what IS ground; every scene simply declares it, which is
// the whole point: the projector must be testable to completion before a segmenter exists,
// and it must be provable that it consumes the label rather than re-deriving it.
//
// DETERMINISM WITHOUT A TOOLCHAIN DEPENDENCY. The randomised scenes use a splitmix64
// generator written out in full below rather than <random>. P3's oracle fixture depends on
// libstdc++'s mt19937/uniform_real_distribution stream and its comment says so; that is a
// real, if small, liability - a libstdc++ change silently invalidates the fixture. These
// scenes have no such dependency: the bit sequence is defined by the code in this header
// and is identical on every conforming compiler, so a golden fixture captured on one
// machine is reproducible on any other. `SceneInputDigest` below then makes that checkable
// rather than merely claimed.
//
// The scenes are ordered from analytic (hand-placed points with known answers) to realistic
// (a sampled arena of cylinders and walls, which is what P8's detector port will actually
// be fed). Every scene satisfies LabeledPointCloud::Validate().
#ifndef PERCEPTION_TESTS_FIXTURES_PROJECTION_SCENES_H_
#define PERCEPTION_TESTS_FIXTURES_PROJECTION_SCENES_H_

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "perception/core/contracts/labeled_point_cloud.h"
#include "perception/core/contracts/validation.h"

namespace projection_scenes {

using perception::core::FrameId;
using perception::core::kPi;
using perception::core::LabeledPointCloud;
using perception::core::MotionCompensation;
using perception::core::PointLabel;
using perception::core::RawTimedPoint;

// ---------------------------------------------------------------------------------------
// splitmix64 - the whole generator, so the scenes do not depend on any standard library
// implementation choice. Reference: Steele/Lea/Flood, "Fast splittable pseudorandom number
// generators" (2014); this is the mixing function used as SplittableRandom's core.
// ---------------------------------------------------------------------------------------
class Rng {
 public:
  explicit Rng(std::uint64_t seed) : state_(seed) {}

  std::uint64_t NextU64() {
    state_ += 0x9E3779B97F4A7C15ull;
    std::uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }

  // [0, 1) from the top 53 bits, which is the standard exact construction.
  double NextUnit() {
    return static_cast<double>(NextU64() >> 11) * (1.0 / 9007199254740992.0);
  }

  double NextRange(double low, double high) { return low + (high - low) * NextUnit(); }

 private:
  std::uint64_t state_;
};

// ---------------------------------------------------------------------------------------
// Scene construction helpers. Points are appended with an explicit label; the census is
// recomputed once at the end so it can never drift from the labels.
// ---------------------------------------------------------------------------------------
struct SceneBuilder {
  LabeledPointCloud cloud;

  SceneBuilder() {
    cloud.frame = FrameId::kGravityAlignedBase;
    cloud.motion_compensation = MotionCompensation::kDeskewedToStamp;
  }

  void Add(float x, float y, float z, PointLabel label) {
    RawTimedPoint point;
    point.position = Eigen::Vector3f(x, y, z);
    // Provenance fields are carried but never read by the projector. They are filled with
    // recognisable values so a test that accidentally depends on one fails loudly.
    point.range_m = static_cast<double>(std::hypot(x, y));
    point.time_offset_s = 0.0f;
    point.ray_index = static_cast<std::int32_t>(cloud.points.size());
    point.geom_id = -1;
    point.body_id = -1;
    cloud.points.push_back(point);
    cloud.labels.push_back(label);
  }

  // Polar convenience: a non-ground return at (range, bearing) and height z.
  void AddPolar(double range, double bearing, double z, PointLabel label) {
    Add(static_cast<float>(range * std::cos(bearing)),
        static_cast<float>(range * std::sin(bearing)), static_cast<float>(z), label);
  }

  LabeledPointCloud Finish(double stamp_s, std::uint64_t sequence) {
    cloud.stamp_s = stamp_s;
    cloud.sequence = sequence;
    auto& census = cloud.segmentation;
    census = perception::core::GroundSegmentationResult{};
    census.total_points = static_cast<std::int32_t>(cloud.points.size());
    for (const PointLabel label : cloud.labels) {
      switch (label) {
        case PointLabel::kGround:    ++census.ground_points;     break;
        case PointLabel::kNonGround: ++census.non_ground_points; break;
        case PointLabel::kSelf:      ++census.self_points;       break;
        case PointLabel::kInvalid:   ++census.invalid_points;    break;
      }
    }
    // A z-band segmentation legitimately has no fitted plane; leaving `plane.valid` false
    // is the honest encoding of "these labels were declared, not fitted".
    return cloud;
  }
};

struct Scene {
  std::string name;
  std::string purpose;  // Why this scene exists. Printed by the capture tool.
  LabeledPointCloud cloud;
};

// ---------------------------------------------------------------------------------------
// The scenes.
// ---------------------------------------------------------------------------------------

// Four cardinal points at exact ranges. The -X point is the one that matters: with
// y = +0.0f, atan2 returns exactly +pi, which is the angle that walks upstream's index off
// the end of its buffer.
inline Scene AxisCross() {
  SceneBuilder b;
  b.Add(2.0f, 0.0f, 0.0f, PointLabel::kNonGround);    // bearing 0
  b.Add(0.0f, 3.0f, 0.0f, PointLabel::kNonGround);    // bearing +pi/2
  b.Add(-4.0f, 0.0f, 0.0f, PointLabel::kNonGround);   // bearing exactly +pi (y = +0.0f)
  b.Add(0.0f, -5.0f, 0.0f, PointLabel::kNonGround);   // bearing -pi/2
  b.Add(-6.0f, -0.0f, 0.0f, PointLabel::kNonGround);  // bearing exactly -pi (y = -0.0f)
  return {"axis_cross", "cardinal bearings; the -X point reaches atan2 == +pi exactly",
          b.Finish(1.0, 1)};
}

// One point at the centre of every 8th bin of a 360-bin scan, at a range that increases
// with the bin index so a mis-assignment is visible as a wrong VALUE, not just a wrong slot.
inline Scene BinCentres() {
  SceneBuilder b;
  const double increment = 2.0 * kPi / 360.0;
  for (int bin = 0; bin < 360; bin += 8) {
    const double bearing = -kPi + (static_cast<double>(bin) + 0.5) * increment;
    b.AddPolar(1.0 + 0.02 * bin, bearing, 0.0, PointLabel::kNonGround);
  }
  return {"bin_centres", "one return per 8th bin at a bin-dependent range", b.Finish(2.0, 2)};
}

// A sweep across the +pi seam. Every one of these has |bearing| within ~1e-13 rad of pi,
// which is the neighbourhood where upstream's truncated index evaluates to `bins`.
inline Scene AngleSeam() {
  SceneBuilder b;
  const float ys[] = {0.0f,      -0.0f,     1e-30f,    -1e-30f,   4.44e-16f, -4.44e-16f,
                      8.88e-16f, -8.88e-16f, 1e-14f,   -1e-14f,   1e-13f,    -1e-13f};
  for (const float y : ys) {
    b.Add(-2.5f, y, 0.0f, PointLabel::kNonGround);
  }
  // The mirror case at the bottom of the window: bearings just above -pi.
  b.Add(-2.5f, -1e-7f, 0.0f, PointLabel::kNonGround);
  b.Add(-2.5f, 1e-7f, 0.0f, PointLabel::kNonGround);
  return {"angle_seam", "bearings within 1e-13 rad of +/-pi; the upstream out-of-bounds case",
          b.Finish(3.0, 3)};
}

// Ranges exactly on and astride both bounds, for the shipped [0.10, 10.0] window.
inline Scene RangeEdges() {
  SceneBuilder b;
  const double bearings[] = {0.3, 0.9, 1.7, 2.4, -0.6, -1.9, -2.8, 1.1};
  const double ranges[] = {0.10, 0.09999, 0.100001, 10.0, 10.00001, 9.99999, 0.05, 20.0};
  for (int i = 0; i < 8; ++i) {
    b.AddPolar(ranges[i], bearings[i], 0.0, PointLabel::kNonGround);
  }
  return {"range_edges", "ranges on and astride range_min / range_max", b.Finish(4.0, 4)};
}

// Heights exactly on and astride both band edges, for the shipped [-0.70, 0.20] band.
inline Scene HeightBand() {
  SceneBuilder b;
  const double zs[] = {-0.70, 0.20, -0.7001, 0.2001, -0.69999, 0.19999, 0.0, -1.5, 2.0};
  for (int i = 0; i < 9; ++i) {
    b.AddPolar(3.0, -2.0 + 0.4 * i, zs[i], PointLabel::kNonGround);
  }
  return {"height_band", "heights on and astride the band edges", b.Finish(5.0, 5)};
}

// Twelve returns crowded into ONE bin, offered in an order that is neither ascending nor
// descending, so "keep the nearest" cannot pass by accident of arrival order.
inline Scene NearestReturn() {
  SceneBuilder b;
  const double increment = 2.0 * kPi / 360.0;
  const double bearing = -kPi + 100.5 * increment;  // Comfortably inside bin 100.
  const double ranges[] = {5.0, 2.0, 7.5, 1.25, 9.0, 3.5, 1.5, 8.0, 4.0, 1.75, 6.0, 2.5};
  for (const double range : ranges) {
    b.AddPolar(range, bearing, 0.05, PointLabel::kNonGround);
  }
  // A second bin whose nearest return arrives LAST, and a third whose nearest arrives FIRST.
  const double bearing_b = -kPi + 200.5 * increment;
  for (const double range : {9.5, 8.0, 6.0, 0.5}) b.AddPolar(range, bearing_b, 0.0,
                                                             PointLabel::kNonGround);
  const double bearing_c = -kPi + 300.5 * increment;
  for (const double range : {0.5, 6.0, 8.0, 9.5}) b.AddPolar(range, bearing_c, 0.0,
                                                             PointLabel::kNonGround);
  return {"nearest_return", "many returns per bin, nearest arriving first / last / middle",
          b.Finish(6.0, 6)};
}

// NaNs and infinities in every coordinate slot. Upstream rejects only the NaNs explicitly
// and lets the infinities fall out at the height or range gate; this scene is what proves
// the two paths still produce identical bins.
inline Scene NonFinite() {
  SceneBuilder b;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  b.Add(nan, 1.0f, 0.0f, PointLabel::kNonGround);
  b.Add(1.0f, nan, 0.0f, PointLabel::kNonGround);
  b.Add(1.0f, 1.0f, nan, PointLabel::kNonGround);
  b.Add(inf, 1.0f, 0.0f, PointLabel::kNonGround);
  b.Add(1.0f, -inf, 0.0f, PointLabel::kNonGround);
  b.Add(1.0f, 1.0f, inf, PointLabel::kNonGround);
  b.Add(1.0f, 1.0f, -inf, PointLabel::kNonGround);
  b.Add(nan, nan, nan, PointLabel::kNonGround);
  // Valid returns mixed in, so the scene cannot pass by producing an empty scan.
  b.AddPolar(2.0, 0.5, 0.0, PointLabel::kNonGround);
  b.AddPolar(3.0, -1.2, 0.1, PointLabel::kNonGround);
  return {"non_finite", "NaN and infinity in every coordinate, plus valid returns",
          b.Finish(7.0, 7)};
}

// All four labels, interleaved. The ground and self points are placed where they WOULD
// occupy bins if the projector ignored the label - so a projector that forgets to filter
// produces a visibly different scan rather than a subtly different one.
inline Scene MixedLabels() {
  SceneBuilder b;
  Rng rng(0xA5A5A5A5A5A5A5A5ull);
  for (int i = 0; i < 600; ++i) {
    const double bearing = rng.NextRange(-kPi, kPi);
    const double range = rng.NextRange(0.2, 9.0);
    const double z = rng.NextRange(-0.6, 0.15);
    // A quarter of the points are non-ground; the rest are placed NEARER, so if they leaked
    // into the scan they would win the nearest-return comparison in their bin.
    const int bucket = static_cast<int>(rng.NextU64() % 4);
    if (bucket == 0) {
      b.AddPolar(range, bearing, z, PointLabel::kNonGround);
    } else if (bucket == 1) {
      b.AddPolar(range * 0.3, bearing, z, PointLabel::kGround);
    } else if (bucket == 2) {
      b.AddPolar(range * 0.25, bearing, z, PointLabel::kSelf);
    } else {
      b.AddPolar(range * 0.2, bearing, z, PointLabel::kInvalid);
    }
  }
  return {"mixed_labels", "all four labels; non-projected labels placed nearer on purpose",
          b.Finish(8.0, 8)};
}

// No non-ground points at all. The projector must produce an all-NaN scan that still
// validates, rather than an empty or degenerate one.
inline Scene AllGround() {
  SceneBuilder b;
  Rng rng(0x1234567800000001ull);
  for (int i = 0; i < 200; ++i) {
    b.AddPolar(rng.NextRange(0.2, 9.0), rng.NextRange(-kPi, kPi), rng.NextRange(-0.9, -0.75),
               PointLabel::kGround);
  }
  return {"all_ground", "no non-ground points; the empty-scan case", b.Finish(9.0, 9)};
}

// Uniform random fill of the whole measurement volume, including points outside every gate.
inline Scene DenseRandom() {
  SceneBuilder b;
  Rng rng(0xDEADBEEFCAFEF00Dull);
  for (int i = 0; i < 4000; ++i) {
    // Sampled in Cartesian, not polar, so the bin occupancy is not uniform by construction
    // and the points-per-bin histogram has something to say.
    const double x = rng.NextRange(-12.0, 12.0);
    const double y = rng.NextRange(-12.0, 12.0);
    const double z = rng.NextRange(-1.2, 0.8);
    b.Add(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z),
          PointLabel::kNonGround);
  }
  return {"dense_random", "4000 uniform Cartesian points spanning every gate",
          b.Finish(10.0, 10)};
}

// The scene P8 actually needs: an arena of upright cylinders and four walls, sampled the way
// a lidar would see them, with a ground sheet labelled as ground. Obstacle radii and the
// arena size follow dpcbf_config.yaml's own ranges so the fixture is recognisably this
// repository's problem rather than a generic point cloud.
inline Scene ArenaCylinders() {
  SceneBuilder b;
  Rng rng(0x0B57AC1E00000001ull);

  struct Cylinder { double cx, cy, radius; };
  const Cylinder cylinders[] = {
      {2.5, 0.0, 0.25}, {-1.8, 2.2, 0.30}, {0.5, -3.1, 0.20}, {4.0, 3.5, 0.28},
  };

  // Cylinder surfaces: only the half facing the origin is visible, which is what makes the
  // short-arc bias P8 has to measure show up in this fixture at all. The surface angle is
  // swept over the half-circle centred on the direction back towards the sensor.
  for (const Cylinder& c : cylinders) {
    const double centre_bearing = std::atan2(c.cy, c.cx);
    for (int i = 0; i < 90; ++i) {
      const double s = -0.5 * kPi + kPi * (static_cast<double>(i) / 89.0);
      const double surface_bearing = centre_bearing + kPi + s;
      const double sx = c.cx + c.radius * std::cos(surface_bearing);
      const double sy = c.cy + c.radius * std::sin(surface_bearing);
      // Keep only the surface points whose outward normal faces the sensor.
      if (sx * (sx - c.cx) + sy * (sy - c.cy) > 0.0) continue;
      for (int layer = 0; layer < 6; ++layer) {
        const double z = -0.65 + 0.16 * layer + rng.NextRange(-0.01, 0.01);
        b.Add(static_cast<float>(sx + rng.NextRange(-0.004, 0.004)),
              static_cast<float>(sy + rng.NextRange(-0.004, 0.004)),
              static_cast<float>(z), PointLabel::kNonGround);
      }
    }
  }

  // Four walls at +/- 6 m.
  for (int i = 0; i < 300; ++i) {
    const double t = rng.NextRange(-6.0, 6.0);
    const double z = rng.NextRange(-0.68, 0.18);
    b.Add(6.0f, static_cast<float>(t), static_cast<float>(z), PointLabel::kNonGround);
    b.Add(-6.0f, static_cast<float>(t), static_cast<float>(z), PointLabel::kNonGround);
    b.Add(static_cast<float>(t), 6.0f, static_cast<float>(z), PointLabel::kNonGround);
    b.Add(static_cast<float>(t), -6.0f, static_cast<float>(z), PointLabel::kNonGround);
  }

  // A ground sheet, labelled ground. Below the band as well as ground-labelled, so it is
  // excluded twice over - the deliberate double gate of risk R6.
  for (int i = 0; i < 800; ++i) {
    b.AddPolar(rng.NextRange(0.3, 9.0), rng.NextRange(-kPi, kPi), rng.NextRange(-1.30, -0.95),
               PointLabel::kGround);
  }

  // A handful of self-returns close in, labelled self.
  for (int i = 0; i < 40; ++i) {
    b.AddPolar(rng.NextRange(0.12, 0.35), rng.NextRange(-kPi, kPi), rng.NextRange(-0.5, 0.1),
               PointLabel::kSelf);
  }

  return {"arena_cylinders", "four cylinders, four walls, ground sheet and self returns",
          b.Finish(11.0, 11)};
}

inline std::vector<Scene> AllScenes() {
  return {AxisCross(),   BinCentres(),  AngleSeam(),   RangeEdges(),
          HeightBand(),  NearestReturn(), NonFinite(), MixedLabels(),
          AllGround(),   DenseRandom(), ArenaCylinders()};
}

// ---------------------------------------------------------------------------------------
// Input digest. FNV-1a over the RAW BITS of every point's coordinates and label, in order.
//
// This is what makes the committed golden fixture honest. Without it, a change to this
// header that altered the scenes would be "fixed" by regenerating the goldens, and the
// regression test would keep passing while testing something else. With it, the manifest
// records what the scan was computed FROM, and a scene drift is a manifest mismatch that
// has to be acknowledged deliberately.
// ---------------------------------------------------------------------------------------
inline std::uint64_t SceneInputDigest(const LabeledPointCloud& cloud) {
  std::uint64_t hash = 1469598103934665603ull;  // FNV-1a 64 offset basis.
  const auto mix = [&hash](const void* data, std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < size; ++i) {
      hash ^= bytes[i];
      hash *= 1099511628211ull;  // FNV-1a 64 prime.
    }
  };
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const float xyz[3] = {cloud.points[i].position.x(), cloud.points[i].position.y(),
                          cloud.points[i].position.z()};
    mix(xyz, sizeof(xyz));
    const auto label = static_cast<std::uint8_t>(cloud.labels[i]);
    mix(&label, sizeof(label));
  }
  return hash;
}

// The same digest over a produced scan, so the manifest can pin the OUTPUT bit-exactly too -
// including the NaNs, which no decimal comparison can pin.
inline std::uint64_t ScanOutputDigest(const std::vector<float>& ranges) {
  std::uint64_t hash = 1469598103934665603ull;
  for (const float value : ranges) {
    unsigned char bytes[sizeof(float)];
    std::memcpy(bytes, &value, sizeof(float));
    for (unsigned char byte : bytes) {
      hash ^= byte;
      hash *= 1099511628211ull;
    }
  }
  return hash;
}

// The non-ground points of a scene, in order - the exact input the reference implementation
// is given, so the comparison is about the binning math and nothing else.
inline std::vector<Eigen::Vector3f> NonGroundPoints(const LabeledPointCloud& cloud) {
  std::vector<Eigen::Vector3f> points;
  points.reserve(cloud.points.size());
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    if (cloud.labels[i] == PointLabel::kNonGround) points.push_back(cloud.points[i].position);
  }
  return points;
}

}  // namespace projection_scenes

#endif  // PERCEPTION_TESTS_FIXTURES_PROJECTION_SCENES_H_
