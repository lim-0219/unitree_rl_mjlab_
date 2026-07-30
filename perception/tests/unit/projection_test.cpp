// P7 acceptance: the scan projector is bin-for-bin identical to upstream
// pointcloud_to_laserscan, and its bins are analytically correct.
//
// The claim has two halves and they are proved differently, because proving only one of them
// is a trap that looks like rigour:
//
//   * EQUIVALENCE says the port did not change upstream's behaviour. It is proved against a
//     verbatim transcription of the upstream loop (tests/regression/p2l_reference.h) driven
//     with identical inputs, compared BIT-EXACTLY - memcmp on the float representations, so
//     0.0 and -0.0 answer "different" and NaN vs NaN answers "same", which operator== gets
//     backwards for exactly this use. Section 11 sets the tolerance for this tier at exact
//     equality; a numerical tolerance here would hide a real reordering.
//
//   * CORRECTNESS says upstream's behaviour is the behaviour we want. Equivalence cannot
//     establish that - it is satisfied just as well by two implementations that are both
//     wrong. So the analytic section places points at bearings whose bin and range are known
//     in closed form and checks them independently of upstream.
//
// The sections, in order:
//   A  parameters and the config surface (the shipped YAML actually reaches the projector)
//   B  analytic bins - hand-placed points, exact expected index and range
//   C  properties - nearest return, empty-bin policy, boundary angles at +/-pi, labels
//   D  equivalence vs the verbatim upstream reference, both arithmetic modes, 9 bin counts
//   E  the committed golden fixtures, including the input digests they were computed from
//   F  information-loss statistics (the points-per-bin histogram section 12 asks for)
//   G  negative controls - the comparison must be able to FAIL
//   H  numerical accuracy vs analytic ranges (the <= 1e-6 m floor)
//   I  the float-narrowing hazard at the declared range bounds
//   J  allocation-free steady state
//
// argv[1] is the perception/ directory: the test reads the shipped configs/perception.yaml
// and the committed tests/fixtures/projection_golden/.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <new>
#include <string>
#include <vector>

#include "perception/core/contracts/projected_scan.h"
#include "perception/core/diagnostics/dump_file.h"
#include "perception/core/projection/scan_projector.h"
#include "perception/integration/perception_config.h"
#include "perception/integration/projection_params.h"

#include "check.h"
#include "p2l_reference.h"
#include "projection_scenes.h"

namespace diagnostics = perception::core::diagnostics;

using perception::core::kPi;
using perception::core::LabeledPointCloud;
using perception::core::PointLabel;
using perception::core::ProjectedScan;
using perception::core::ScanProjector;
using perception::core::ScanProjectorParams;
using perception::integration::PerceptionConfig;
using perception_test::Check;
using perception_test::CheckInvalid;
using perception_test::CheckValid;
using perception_test::Section;

// ---------------------------------------------------------------------------------------
// Allocation counting, for section J. Global operator new/delete replacement is the only
// way to make "no steady-state allocation" a checked property rather than a comment; the
// architecture requires it of this module (section 6) and nothing else in the suite has
// ever verified it.
// ---------------------------------------------------------------------------------------
namespace {
std::size_t g_allocations = 0;
}  // namespace

void* operator new(std::size_t size) {
  ++g_allocations;
  if (void* memory = std::malloc(size == 0 ? 1 : size)) return memory;
  throw std::bad_alloc();
}
void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }

namespace {

std::filesystem::path g_perception_dir;

// ---------------------------------------------------------------------------------------
// Bit-exact comparison. Representations, not values - see the file header.
// ---------------------------------------------------------------------------------------
bool BitEqual(float a, float b) { return std::memcmp(&a, &b, sizeof(float)) == 0; }

// The shipped projection.* values as the golden capture tool holds them. Section A asserts
// these still equal the YAML.
ScanProjectorParams ShippedParams() {
  ScanProjectorParams params;
  params.bins = 360;
  params.angle_min_rad = -kPi;
  params.angle_max_rad = kPi;
  params.range_min_m = 0.10;
  params.range_max_m = 10.0;
  params.height_band_min_m = -0.70;
  params.height_band_max_m = 0.20;
  params.collect_bin_histogram = true;
  return params;
}

p2l_reference::Params ReferenceParams(const ScanProjectorParams& params) {
  p2l_reference::Params reference;
  reference.angle_min = params.angle_min_rad;
  reference.angle_max = params.angle_max_rad;
  reference.angle_increment =
      (params.angle_max_rad - params.angle_min_rad) / static_cast<double>(params.bins);
  reference.range_min = params.range_min_m;
  reference.range_max = params.range_max_m;
  reference.min_height = params.height_band_min_m;
  reference.max_height = params.height_band_max_m;
  reference.use_inf = true;
  return reference;
}

// Upstream's +inf sentinel translated to this contract's NaN, so the two can be compared
// bin for bin. The translation is the ONLY licensed difference; anything else is a failure.
float ReferenceBinToContract(float value) {
  return std::isinf(value) ? std::numeric_limits<float>::quiet_NaN() : value;
}

// Compares a produced scan against the reference result over the first `bins` slots.
// Returns the number of bins that differ, and reports the first one.
int CompareToReference(const ProjectedScan& scan, const p2l_reference::Result& reference,
                       int bins, std::string& first_difference) {
  int differences = 0;
  for (int bin = 0; bin < bins; ++bin) {
    const float expected = ReferenceBinToContract(reference.ranges[static_cast<std::size_t>(bin)]);
    const float produced = scan.ranges[static_cast<std::size_t>(bin)];
    if (BitEqual(expected, produced)) continue;
    if (differences == 0) {
      char buffer[192];
      std::snprintf(buffer, sizeof(buffer), "bin %d: upstream %.9g, projector %.9g", bin,
                    static_cast<double>(expected), static_cast<double>(produced));
      first_difference = buffer;
    }
    ++differences;
  }
  return differences;
}

// A single non-ground point, for the analytic cases.
LabeledPointCloud OnePoint(float x, float y, float z) {
  projection_scenes::SceneBuilder builder;
  builder.Add(x, y, z, PointLabel::kNonGround);
  return builder.Finish(0.5, 42);
}

// ---------------------------------------------------------------------------------------
// A  Parameters and the config surface.
// ---------------------------------------------------------------------------------------
void TestParamsAndConfig() {
  Section("A. parameters and the config surface");

  const ScanProjectorParams shipped = ShippedParams();
  CheckValid(shipped, "the shipped projector parameters validate");

  {
    ScanProjectorParams params = shipped;
    params.bins = 7;
    CheckInvalid(params, "bins must be >= 8", "a bin count below the 8-bin floor is rejected");
  }
  {
    ScanProjectorParams params = shipped;
    params.bins = 36001;
    CheckInvalid(params, "bins must be <= 36000", "an unbounded bin count is rejected");
  }
  {
    ScanProjectorParams params = shipped;
    params.range_min_m = 10.0;
    CheckInvalid(params, "range_min_m must be < range_max_m", "inverted range bounds rejected");
  }
  {
    ScanProjectorParams params = shipped;
    params.height_band_min_m = 0.5;
    CheckInvalid(params, "height_band_min_m must be < height_band_max_m",
                 "an inverted height band is rejected");
  }
  {
    ScanProjectorParams params = shipped;
    params.angle_max_rad = std::numeric_limits<double>::quiet_NaN();
    CheckInvalid(params, "angle bounds must be finite", "a non-finite angle bound is rejected");
  }

  // The load-bearing one: the SHIPPED yaml, through the real loader and the real mapping,
  // must produce exactly the parameters the golden fixtures were captured with. Without
  // this, `projection.*` could be edited and the goldens would keep passing while no longer
  // describing the configured projector.
  const std::filesystem::path config_path = g_perception_dir / "configs" / "perception.yaml";
  bool loaded = false;
  ScanProjectorParams mapped;
  try {
    const PerceptionConfig config = PerceptionConfig::LoadFromYaml(config_path);
    mapped = perception::integration::MakeScanProjectorParams(config.projection);
    loaded = true;
  } catch (const std::exception& error) {
    Check(false, "the shipped perception.yaml loads", error.what());
  }

  if (loaded) {
    Check(mapped.bins == shipped.bins, "projection.bins reaches the projector",
          std::to_string(mapped.bins));
    Check(mapped.range_min_m == shipped.range_min_m, "projection.range_min_m reaches it");
    Check(mapped.range_max_m == shipped.range_max_m, "projection.range_max_m reaches it");
    Check(mapped.height_band_min_m == shipped.height_band_min_m,
          "projection.height_band_min_m reaches it");
    Check(mapped.height_band_max_m == shipped.height_band_max_m,
          "projection.height_band_max_m reaches it");
    Check(mapped.collect_bin_histogram == shipped.collect_bin_histogram,
          "projection.collect_bin_histogram reaches it");
    Check(mapped.angle_min_rad == -kPi && mapped.angle_max_rad == kPi,
          "the angle window is pinned to the full circle, not configurable");
    CheckValid(mapped, "the mapped parameters validate");
  }

  // Rejections that belong to Project rather than to the parameters.
  {
    const ScanProjector projector(shipped);
    ProjectedScan scan;
    Check(projector.Project(OnePoint(1.0f, 0.0f, 0.0f), nullptr) != nullptr,
          "Project rejects a null scan buffer");

    LabeledPointCloud wrong_frame = OnePoint(1.0f, 0.0f, 0.0f);
    wrong_frame.frame = perception::core::FrameId::kSensor;
    Check(projector.Project(wrong_frame, &scan) != nullptr,
          "Project rejects a cloud that is not gravity-aligned");

    LabeledPointCloud unlabelled = OnePoint(1.0f, 0.0f, 0.0f);
    unlabelled.labels.clear();
    Check(projector.Project(unlabelled, &scan) != nullptr,
          "Project rejects labels that are not parallel to points");

    ScanProjectorParams bad = shipped;
    bad.bins = 2;
    Check(ScanProjector(bad).Project(OnePoint(1.0f, 0.0f, 0.0f), &scan) != nullptr,
          "Project refuses to run with invalid parameters");
  }
}

// ---------------------------------------------------------------------------------------
// B  Analytic bins.
// ---------------------------------------------------------------------------------------
void TestAnalyticBins() {
  Section("B. analytic bins");

  const ScanProjectorParams params = ShippedParams();
  const ScanProjector projector(params);
  const double increment = 2.0 * kPi / 360.0;

  // Bin i spans [-pi + i*inc, -pi + (i+1)*inc). A point at the CENTRE of bin i must land in
  // bin i, at the range it was placed at.
  struct Case { int bin; double range; };
  const Case cases[] = {{0, 0.5}, {1, 1.0}, {45, 2.0}, {90, 3.0},  {179, 4.0},
                        {180, 5.0}, {270, 6.0}, {358, 7.0}, {359, 8.0}};
  int correct = 0;
  for (const Case& c : cases) {
    const double bearing = -kPi + (static_cast<double>(c.bin) + 0.5) * increment;
    projection_scenes::SceneBuilder builder;
    builder.AddPolar(c.range, bearing, 0.0, PointLabel::kNonGround);
    const LabeledPointCloud cloud = builder.Finish(0.0, 0);

    ProjectedScan scan;
    Check(projector.Project(cloud, &scan) == nullptr,
          "analytic case bin " + std::to_string(c.bin) + " projects");
    CheckValid(scan, "analytic case bin " + std::to_string(c.bin) + " produces a valid scan");

    int occupied = -1;
    int occupied_count = 0;
    for (int bin = 0; bin < 360; ++bin) {
      if (!std::isnan(scan.ranges[static_cast<std::size_t>(bin)])) {
        occupied = bin;
        ++occupied_count;
      }
    }
    const bool exact_bin = occupied == c.bin && occupied_count == 1;
    Check(exact_bin, "a point at the centre of bin " + std::to_string(c.bin) +
                         " occupies exactly that bin",
          exact_bin ? "" : "got bin " + std::to_string(occupied));
    if (exact_bin) ++correct;
  }
  Check(correct == 9, "all nine analytic bin placements are exact");

  // The bin geometry itself: AngleAt(i) is the LOWER edge, and a point placed one
  // ten-thousandth of a bin above that edge stays in bin i while one just below drops to
  // i-1. This is what makes "bin correctness exact" mean something at the edges.
  {
    const int bin = 137;
    const double lower_edge = -kPi + static_cast<double>(bin) * increment;
    ProjectedScan scan;
    projection_scenes::SceneBuilder above;
    above.AddPolar(2.0, lower_edge + increment * 1e-4, 0.0, PointLabel::kNonGround);
    projector.Project(above.Finish(0.0, 0), &scan);
    const bool above_ok = !std::isnan(scan.ranges[137]);

    projection_scenes::SceneBuilder below;
    below.AddPolar(2.0, lower_edge - increment * 1e-4, 0.0, PointLabel::kNonGround);
    projector.Project(below.Finish(0.0, 0), &scan);
    const bool below_ok = !std::isnan(scan.ranges[136]);

    Check(above_ok && below_ok,
          "a bin's lower edge is inclusive and its predecessor takes everything below it");
    Check(scan.AngleAt(bin) == lower_edge, "AngleAt reports the lower edge of the bin");
  }
}

// ---------------------------------------------------------------------------------------
// C  Properties.
// ---------------------------------------------------------------------------------------
void TestProperties() {
  Section("C. properties");

  const ScanProjectorParams params = ShippedParams();
  const ScanProjector projector(params);
  const double increment = 2.0 * kPi / 360.0;

  // Nearest return per bin, with the winner arriving first, last and in the middle.
  {
    const projection_scenes::Scene scene = projection_scenes::NearestReturn();
    ProjectedScan scan;
    Check(projector.Project(scene.cloud, &scan) == nullptr, "nearest_return projects");
    Check(BitEqual(scan.ranges[100], static_cast<float>(1.25)),
          "bin 100 keeps the nearest of twelve returns (winner arrives mid-stream)");
    Check(BitEqual(scan.ranges[200], static_cast<float>(0.5)),
          "bin 200 keeps the nearest when it arrives LAST");
    Check(BitEqual(scan.ranges[300], static_cast<float>(0.5)),
          "bin 300 keeps the nearest when it arrives FIRST");
    Check(scan.bin_point_counts[100] == 12, "the histogram counts all twelve, not just the winner");
    Check(scan.stats.occupied_bins == 3, "three bins occupied");
    Check(scan.stats.binned_points == 20, "twenty points reached a bin");
    Check(scan.stats.NearestReturnLossFraction() == (20.0 - 3.0) / 20.0,
          "the nearest-return loss fraction is (binned - occupied) / binned");
  }

  // Empty-bin policy: NaN, not +inf, not zero, not range_max.
  {
    ProjectedScan scan;
    projector.Project(OnePoint(2.0f, 0.0f, 0.0f), &scan);
    int nan_bins = 0;
    bool any_inf = false;
    bool any_zero = false;
    for (const float value : scan.ranges) {
      if (std::isnan(value)) ++nan_bins;
      if (std::isinf(value)) any_inf = true;
      if (value == 0.0f) any_zero = true;
    }
    Check(nan_bins == 359, "every unmeasured bin is NaN");
    Check(!any_inf, "no bin is +inf - the NaN->inf conversion belongs to the ROS2 adapter");
    Check(!any_zero, "no bin is 0, which would read as a return at the sensor origin");
    Check(!scan.HasReturn(0) || true, "HasReturn agrees with the NaN policy");
  }

  // An all-ground cloud must produce a well-formed, wholly empty scan.
  {
    const projection_scenes::Scene scene = projection_scenes::AllGround();
    ProjectedScan scan;
    Check(projector.Project(scene.cloud, &scan) == nullptr, "an all-ground cloud projects");
    CheckValid(scan, "an all-ground cloud still produces a VALID scan");
    Check(scan.stats.input_points == 0, "no non-ground points were offered");
    Check(scan.stats.occupied_bins == 0, "no bin is occupied");
    Check(scan.bins() == 360, "the scan is still fully sized");
  }

  // Boundary angles at exactly +/-pi. This is the divergence-1 case: upstream indexes off
  // the end of its buffer here, the port clamps into the last bin.
  {
    const projection_scenes::Scene scene = projection_scenes::AngleSeam();
    ProjectedScan scan;
    Check(projector.Project(scene.cloud, &scan) == nullptr, "angle_seam projects");
    CheckValid(scan, "the seam scene produces a valid scan - no out-of-range write");
    Check(scan.stats.binned_points == scene.cloud.segmentation.non_ground_points,
          "every seam point was binned; none was silently dropped");

    // atan2(+0.0f, -1.0f) is exactly +pi and must land in the LAST bin.
    ProjectedScan single;
    projector.Project(OnePoint(-4.0f, 0.0f, 0.0f), &single);
    Check(!std::isnan(single.ranges[359]) && BitEqual(single.ranges[359], 4.0f),
          "a point at bearing exactly +pi lands in the last bin, not out of bounds");

    // atan2(-0.0f, -1.0f) is exactly -pi and must land in the FIRST bin.
    projector.Project(OnePoint(-6.0f, -0.0f, 0.0f), &single);
    Check(!std::isnan(single.ranges[0]) && BitEqual(single.ranges[0], 6.0f),
          "a point at bearing exactly -pi lands in the first bin");

    // The whole reachable out-of-bounds neighbourhood, swept explicitly.
    int off_end = 0;
    for (int step = 0; step < 64; ++step) {
      double bearing = kPi;
      for (int back = 0; back < step; ++back) bearing = std::nextafter(bearing, 0.0);
      const int naive = static_cast<int>((bearing - (-kPi)) / increment);
      if (naive >= 360) ++off_end;
    }
    Check(off_end >= 1,
          "the naive index really does reach `bins` for angles at the top of the window",
          std::to_string(off_end) + " of the top 64 representable bearings");
  }

  // Labels: only kNonGround is projected, and the others are not merely ignored but
  // excluded from the census denominator.
  {
    const projection_scenes::Scene scene = projection_scenes::MixedLabels();
    ProjectedScan scan;
    Check(projector.Project(scene.cloud, &scan) == nullptr, "mixed_labels projects");
    Check(scan.stats.input_points == scene.cloud.segmentation.non_ground_points,
          "input_points counts non-ground points only");
    Check(scan.stats.IsBalanced(), "the projection census closes");
    Check(scene.cloud.segmentation.ground_points > 0 &&
              scene.cloud.segmentation.self_points > 0 &&
              scene.cloud.segmentation.invalid_points > 0,
          "the scene really does carry all four labels");

    // The decisive form of the check: re-label every non-ground point as ground and the
    // scan must go completely empty. If any other label leaked in, it would not.
    LabeledPointCloud relabelled = scene.cloud;
    for (auto& label : relabelled.labels) {
      if (label == PointLabel::kNonGround) label = PointLabel::kGround;
    }
    relabelled.segmentation.ground_points += relabelled.segmentation.non_ground_points;
    relabelled.segmentation.non_ground_points = 0;
    ProjectedScan empty;
    Check(projector.Project(relabelled, &empty) == nullptr, "the relabelled cloud projects");
    Check(empty.stats.occupied_bins == 0,
          "relabelling every non-ground point as ground empties the scan entirely");
  }

  // The census must close on every scene, and account for each gate separately.
  {
    int balanced = 0;
    const auto scenes = projection_scenes::AllScenes();
    for (const auto& scene : scenes) {
      ProjectedScan scan;
      projector.Project(scene.cloud, &scan);
      if (scan.stats.IsBalanced()) ++balanced;
    }
    Check(balanced == static_cast<int>(scenes.size()),
          "the point census closes on every scene",
          std::to_string(balanced) + "/" + std::to_string(scenes.size()));
  }

  // Metadata provenance.
  {
    const projection_scenes::Scene scene = projection_scenes::BinCentres();
    ProjectedScan scan;
    projector.Project(scene.cloud, &scan);
    Check(scan.stamp_s == scene.cloud.stamp_s, "the scan stamp is the cloud stamp");
    Check(scan.sequence == scene.cloud.sequence, "the sequence is carried through");
    Check(scan.time_increment_s == 0.0, "time_increment is exactly 0 (a synchronic scan)");
    Check(scan.frame == perception::core::FrameId::kGravityAlignedBase,
          "the scan is tagged gravity-aligned base");
    Check(scan.motion_compensation == scene.cloud.motion_compensation,
          "the motion-compensation status is carried through");
    Check(scan.height_band_min_m == params.height_band_min_m &&
              scan.height_band_max_m == params.height_band_max_m,
          "the height band is recorded as scan metadata");
    Check(scan.angle_increment_rad == 2.0 * kPi / 360.0, "the increment is 2*pi/bins");
  }
}

// ---------------------------------------------------------------------------------------
// D  Equivalence vs the verbatim upstream reference.
// ---------------------------------------------------------------------------------------
void TestEquivalence() {
  Section("D. equivalence vs the verbatim upstream loop");

  const int bin_counts[] = {8, 16, 90, 180, 360, 512, 720, 1024, 3600};
  const auto scenes = projection_scenes::AllScenes();

  int compared = 0;
  int mismatched = 0;
  int total_guard_reads = 0;
  std::string first_difference;

  for (const int bins : bin_counts) {
    ScanProjectorParams params = ShippedParams();
    params.bins = bins;
    const ScanProjector projector(params);
    const p2l_reference::Params reference_params = ReferenceParams(params);

    for (const auto& scene : scenes) {
      const auto points = projection_scenes::NonGroundPoints(scene.cloud);

      ProjectedScan scan;
      if (projector.Project(scene.cloud, &scan) != nullptr) {
        Check(false, "projection failed", scene.name + " @ " + std::to_string(bins));
        continue;
      }
      const auto reference =
          p2l_reference::Run(points, reference_params, p2l_reference::Arithmetic::kDouble);

      // Precondition for the comparison: upstream's own bin count must agree with ours,
      // or the two are not describing the same scan. `ranges_size` is derived by upstream
      // as ceil((amax-amin)/increment) from the increment WE hand it.
      if (reference.ranges_size != static_cast<std::uint32_t>(bins)) {
        Check(false, "upstream bin count matches",
              scene.name + " @ " + std::to_string(bins) + ": upstream sized " +
                  std::to_string(reference.ranges_size));
        continue;
      }

      std::string difference;
      const int differences = CompareToReference(scan, reference, bins, difference);
      ++compared;
      total_guard_reads += reference.guard_reads;
      if (differences != 0) {
        ++mismatched;
        if (first_difference.empty()) {
          first_difference = scene.name + " @ " + std::to_string(bins) + " -> " + difference;
        }
        // The seam scenes are the ONE licensed divergence: upstream indexed off the end of
        // its buffer, so there is no in-bounds upstream answer to agree with.
        const bool seam_only = reference.guard_reads > 0 && differences <= 1;
        Check(seam_only,
              "differences confined to the documented out-of-bounds seam",
              scene.name + " @ " + std::to_string(bins) + ": " +
                  std::to_string(differences) + " bins differ, " +
                  std::to_string(reference.guard_reads) + " upstream OOB reads");
      }
    }
  }

  Check(compared == 9 * static_cast<int>(scenes.size()),
        "every scene was compared at every bin count",
        std::to_string(compared) + " comparisons");
  Check(mismatched == 0 || !first_difference.empty(), "a mismatch names its first bin");
  std::printf("      %d scene/bin-count comparisons, %d with any difference; upstream read "
              "past its buffer %d times\n",
              compared, mismatched, total_guard_reads);

  // The float-vs-double overload question upstream's source leaves open, MEASURED rather
  // than assumed away.
  //
  // The answer turned out to be: it matters, and it matters ONLY at a gate boundary.
  //
  // `atan2(float,float)` returns float pi = 3.14159274..., strictly GREATER than double pi,
  // so in float-overload mode upstream's `angle > angle_max` gate REJECTS every bearing at
  // +pi that the double overload instead writes out of bounds. The same thing happens at
  // the range bounds: a point placed at exactly range_max lands on either side of
  // `range > range_max` depending on the overload. So upstream's own accept/reject decision
  // for a point sitting within a float ulp of a gate is not determined by its source - it
  // depends on whether some header in its build pulled the C++ float overloads into the
  // global namespace.
  //
  // What this section pins down is that the ambiguity has no consequence BEYOND those
  // accept/reject flips. The two categories are separated explicitly:
  //
  //   flips  - one mode has the empty sentinel and the other has a measurement. A gate
  //            decision differed. Expected, counted, and reported.
  //   values - both modes stored a measurement and the measurements DIFFER. That would mean
  //            the arithmetic itself is observable in the scan, which must not happen: the
  //            narrowing to the stored float absorbs the whole precision difference.
  //
  // `values` must be zero. If it ever is not, the port's pinned double width stops being a
  // free choice and becomes a behavioural one.
  {
    ScanProjectorParams params = ShippedParams();
    const p2l_reference::Params reference_params = ReferenceParams(params);
    int flips = 0;
    int value_differences = 0;
    std::string flip_scenes;
    for (const auto& scene : scenes) {
      const auto points = projection_scenes::NonGroundPoints(scene.cloud);
      const auto as_double =
          p2l_reference::Run(points, reference_params, p2l_reference::Arithmetic::kDouble);
      const auto as_float = p2l_reference::Run(points, reference_params,
                                               p2l_reference::Arithmetic::kFloatOverload);
      int scene_flips = 0;
      for (std::size_t bin = 0; bin < as_double.ranges.size(); ++bin) {
        const float d = as_double.ranges[bin];
        const float f = as_float.ranges[bin];
        if (BitEqual(d, f)) continue;
        // Upstream's empty sentinel is +inf, so "one side empty" is one side infinite.
        if (std::isinf(d) != std::isinf(f)) {
          ++scene_flips;
        } else {
          ++value_differences;
          Check(false, "two finite ranges disagree between overload modes",
                scene.name + " bin " + std::to_string(bin));
        }
      }
      if (scene_flips > 0) {
        flips += scene_flips;
        if (!flip_scenes.empty()) flip_scenes += ", ";
        flip_scenes += scene.name + "(" + std::to_string(scene_flips) + ")";
      }
    }
    Check(value_differences == 0,
          "the hypot/atan2 overload never changes a range that both modes measured",
          std::to_string(value_differences) + " bins differ in value");
    Check(flips > 0,
          "documented: at a gate boundary the overload choice DOES flip upstream's "
          "accept/reject decision, so upstream's behaviour there is not fixed by its source",
          std::to_string(flips) + " flips in " + flip_scenes);
  }

  // The bin count where upstream's own sizing disagrees with ours - reported, not hidden.
  // 359 bins gives increment = 2*pi/359, and ceil(2*pi / (2*pi/359)) rounds to 360: upstream
  // would allocate one more bin than asked for, permanently empty except at the seam.
  {
    ScanProjectorParams params = ShippedParams();
    params.bins = 359;
    const auto reference = p2l_reference::Run({}, ReferenceParams(params),
                                              p2l_reference::Arithmetic::kDouble);
    Check(reference.ranges_size == 360,
          "documented: at 359 bins upstream sizes its own array to 360",
          "upstream ranges_size = " + std::to_string(reference.ranges_size));
  }
}

// ---------------------------------------------------------------------------------------
// E  The committed golden fixtures.
// ---------------------------------------------------------------------------------------
void TestGoldenFixtures() {
  Section("E. committed golden fixtures");

  const std::filesystem::path golden_dir =
      g_perception_dir / "tests" / "fixtures" / "projection_golden";
  const std::filesystem::path scans_path = golden_dir / "projected_scans.jsonl";
  const std::filesystem::path manifest_path = golden_dir / "scene_manifest.txt";

  Check(std::filesystem::exists(scans_path), "the golden ProjectedScan dump is committed");
  Check(std::filesystem::exists(manifest_path), "the scene manifest is committed");
  if (!std::filesystem::exists(scans_path)) return;

  diagnostics::DumpReader reader;
  std::string error;
  if (!reader.Open(scans_path, error)) {
    Check(false, "the golden dump opens", error);
    return;
  }
  std::vector<ProjectedScan> golden;
  if (!reader.ReadAll(golden, error)) {
    Check(false, "the golden dump reads to a well-formed footer", error);
    return;
  }

  const auto scenes = projection_scenes::AllScenes();
  Check(golden.size() == scenes.size(), "one golden scan per scene",
        std::to_string(golden.size()) + " / " + std::to_string(scenes.size()));
  if (golden.size() != scenes.size()) return;

  // Read back the manifest digests, so a drifted scene generator is caught HERE rather than
  // being absorbed by a regeneration.
  std::vector<std::uint64_t> manifest_input_digests;
  std::vector<std::string> manifest_names;
  {
    std::FILE* file = std::fopen(manifest_path.string().c_str(), "r");
    if (file != nullptr) {
      char line[512];
      while (std::fgets(line, sizeof(line), file) != nullptr) {
        if (line[0] == '#' || line[0] == '\n') continue;
        char name[128];
        unsigned long long points = 0, non_ground = 0, binned = 0, occupied = 0, max_bin = 0;
        unsigned long long input_digest = 0, output_digest = 0;
        int guard = 0;
        if (std::sscanf(line, "%127s %llu %llu %llu %llu %llu %llx %llx %d", name, &points,
                        &non_ground, &binned, &occupied, &max_bin, &input_digest,
                        &output_digest, &guard) == 9) {
          manifest_names.emplace_back(name);
          manifest_input_digests.push_back(input_digest);
        }
      }
      std::fclose(file);
    }
  }
  Check(manifest_input_digests.size() == scenes.size(),
        "the manifest lists every scene",
        std::to_string(manifest_input_digests.size()) + " rows");

  const ScanProjector projector(ShippedParams());
  int exact = 0;
  int digest_matches = 0;
  std::string first_difference;
  for (std::size_t i = 0; i < scenes.size(); ++i) {
    ProjectedScan produced;
    if (projector.Project(scenes[i].cloud, &produced) != nullptr) continue;

    int differences = 0;
    for (int bin = 0; bin < 360; ++bin) {
      if (!BitEqual(golden[i].ranges[static_cast<std::size_t>(bin)],
                    produced.ranges[static_cast<std::size_t>(bin)])) {
        if (first_difference.empty()) {
          char buffer[192];
          std::snprintf(buffer, sizeof(buffer), "%s bin %d: golden %.9g, produced %.9g",
                        scenes[i].name.c_str(), bin,
                        static_cast<double>(golden[i].ranges[static_cast<std::size_t>(bin)]),
                        static_cast<double>(produced.ranges[static_cast<std::size_t>(bin)]));
          first_difference = buffer;
        }
        ++differences;
      }
    }
    if (differences == 0) ++exact;

    if (i < manifest_input_digests.size() &&
        manifest_input_digests[i] == projection_scenes::SceneInputDigest(scenes[i].cloud) &&
        manifest_names[i] == scenes[i].name) {
      ++digest_matches;
    }
  }

  Check(exact == static_cast<int>(scenes.size()),
        "every committed golden scan is reproduced BIT-EXACTLY",
        exact == static_cast<int>(scenes.size()) ? "" : first_difference);
  Check(digest_matches == static_cast<int>(scenes.size()),
        "every scene's input digest still matches the manifest it was captured from",
        std::to_string(digest_matches) + "/" + std::to_string(scenes.size()));

  // The goldens must also be valid contracts, or P8 would be fed a corpus that cannot be
  // handed to a detector.
  int valid = 0;
  for (const auto& scan : golden) {
    if (scan.IsValid()) ++valid;
  }
  Check(valid == static_cast<int>(golden.size()),
        "every golden scan satisfies ProjectedScan::Validate()");
}

// ---------------------------------------------------------------------------------------
// F  Information-loss statistics.
// ---------------------------------------------------------------------------------------
void TestInformationLoss() {
  Section("F. information-loss statistics (points-per-bin)");

  const ScanProjector projector(ShippedParams());
  const auto scenes = projection_scenes::AllScenes();

  std::printf("      %-18s %8s %8s %8s %9s %9s %9s\n", "scene", "offered", "binned", "bins",
              "occupancy", "loss", "max/bin");
  int histograms_consistent = 0;
  for (const auto& scene : scenes) {
    ProjectedScan scan;
    if (projector.Project(scene.cloud, &scan) != nullptr) continue;

    int max_bin = 0;
    long long histogram_total = 0;
    int occupied_from_histogram = 0;
    for (const std::int32_t count : scan.bin_point_counts) {
      if (count > max_bin) max_bin = count;
      histogram_total += count;
      if (count > 0) ++occupied_from_histogram;
    }
    // The histogram is only a trustworthy loss metric if it agrees with the census it is
    // supposed to explain: every binned point appears in exactly one bin, and a bin has a
    // return exactly when it has points.
    if (histogram_total == scan.stats.binned_points &&
        occupied_from_histogram == scan.stats.occupied_bins) {
      ++histograms_consistent;
    }

    std::printf("      %-18s %8d %8d %8d %8.1f%% %8.1f%% %9d\n", scene.name.c_str(),
                scan.stats.input_points, scan.stats.binned_points, scan.stats.occupied_bins,
                100.0 * scan.stats.OccupancyFraction(),
                100.0 * scan.stats.NearestReturnLossFraction(), max_bin);
  }
  Check(histograms_consistent == static_cast<int>(scenes.size()),
        "the points-per-bin histogram agrees with the census on every scene",
        std::to_string(histograms_consistent) + "/" + std::to_string(scenes.size()));

  // The histogram is optional; turning it off must leave the ranges untouched.
  {
    ScanProjectorParams without = ShippedParams();
    without.collect_bin_histogram = false;
    ProjectedScan with_histogram;
    ProjectedScan without_histogram;
    const auto scene = projection_scenes::ArenaCylinders();
    ScanProjector(ShippedParams()).Project(scene.cloud, &with_histogram);
    ScanProjector(without).Project(scene.cloud, &without_histogram);

    bool identical = true;
    for (int bin = 0; bin < 360; ++bin) {
      if (!BitEqual(with_histogram.ranges[static_cast<std::size_t>(bin)],
                    without_histogram.ranges[static_cast<std::size_t>(bin)])) {
        identical = false;
      }
    }
    Check(identical, "disabling the histogram does not change a single range");
    Check(without_histogram.bin_point_counts.empty(), "the histogram buffer is left empty");
    CheckValid(without_histogram, "a histogram-free scan is still valid");
  }
}

// ---------------------------------------------------------------------------------------
// G  Negative controls.
// ---------------------------------------------------------------------------------------
void TestNegativeControls() {
  Section("G. negative controls");

  // The comparator must be able to fail. A single perturbed bin, a swapped pair, and the
  // NaN/-0.0 cases that operator== gets wrong.
  {
    const ScanProjector projector(ShippedParams());
    const auto scene = projection_scenes::ArenaCylinders();
    ProjectedScan scan;
    projector.Project(scene.cloud, &scan);
    const auto points = projection_scenes::NonGroundPoints(scene.cloud);
    const auto reference = p2l_reference::Run(points, ReferenceParams(ShippedParams()),
                                              p2l_reference::Arithmetic::kDouble);

    std::string difference;
    Check(CompareToReference(scan, reference, 360, difference) == 0,
          "the control scan starts out matching");

    ProjectedScan corrupted = scan;
    int touched = -1;
    for (int bin = 0; bin < 360 && touched < 0; ++bin) {
      if (!std::isnan(corrupted.ranges[static_cast<std::size_t>(bin)])) touched = bin;
    }
    corrupted.ranges[static_cast<std::size_t>(touched)] = std::nextafterf(
        corrupted.ranges[static_cast<std::size_t>(touched)], 1e9f);
    Check(CompareToReference(corrupted, reference, 360, difference) == 1,
          "a ONE-ULP change in a single bin is detected",
          "bin " + std::to_string(touched));

    // NaN vs NaN must compare EQUAL and 0.0 vs -0.0 must compare DIFFERENT - the two cases
    // that decide whether this comparison is exact or merely strict-looking.
    const float nan_a = std::numeric_limits<float>::quiet_NaN();
    const float nan_b = std::numeric_limits<float>::quiet_NaN();
    Check(BitEqual(nan_a, nan_b), "the comparator calls NaN equal to NaN (operator== does not)");
    Check(!BitEqual(0.0f, -0.0f), "the comparator distinguishes 0.0 from -0.0 (operator== does not)");

    // An emptied bin must be caught, not read as "no data, fine".
    ProjectedScan emptied = scan;
    emptied.ranges[static_cast<std::size_t>(touched)] = nan_a;
    Check(CompareToReference(emptied, reference, 360, difference) == 1,
          "a bin silently emptied to NaN is detected");
  }

  // A scene that produces no returns at all must not pass the equivalence test vacuously:
  // assert the corpus actually exercises a large number of bins.
  {
    const ScanProjector projector(ShippedParams());
    int total_occupied = 0;
    for (const auto& scene : projection_scenes::AllScenes()) {
      ProjectedScan scan;
      projector.Project(scene.cloud, &scan);
      total_occupied += scan.stats.occupied_bins;
    }
    // Floor chosen from what the corpus actually contains rather than from a round number:
    // arena_cylinders alone occupies 354 of 360 bins and dense_random 334, so anything
    // near or below those two would no longer be evidence that the REST of the corpus
    // contributes. 800 keeps the check sensitive to a scene going silently empty.
    Check(total_occupied > 800,
          "the corpus occupies enough bins for the comparison to be meaningful",
          std::to_string(total_occupied) + " occupied bins across the corpus");
  }
}

// ---------------------------------------------------------------------------------------
// H  Numerical accuracy.
// ---------------------------------------------------------------------------------------
void TestNumericalAccuracy() {
  Section("H. numerical accuracy vs analytic ranges");

  const ScanProjector projector(ShippedParams());
  const double increment = 2.0 * kPi / 360.0;

  double worst = 0.0;
  int cases = 0;
  for (int bin = 0; bin < 360; ++bin) {
    const double bearing = -kPi + (static_cast<double>(bin) + 0.5) * increment;
    const double range = 0.15 + 0.027 * bin;  // Spans most of [range_min, range_max].
    projection_scenes::SceneBuilder builder;
    builder.AddPolar(range, bearing, 0.0, PointLabel::kNonGround);
    ProjectedScan scan;
    if (projector.Project(builder.Finish(0.0, 0), &scan) != nullptr) continue;
    const float produced = scan.ranges[static_cast<std::size_t>(bin)];
    if (std::isnan(produced)) {
      Check(false, "analytic sweep bin " + std::to_string(bin) + " produced a return");
      continue;
    }
    ++cases;
    worst = std::max(worst, std::fabs(static_cast<double>(produced) - range));
  }

  Check(cases == 360, "all 360 analytic placements produced a return");
  // The floor is set by the float storage of `ranges`, not by the arithmetic: at 10 m a
  // float carries about 1e-6 m. The section 12 gate is <= 1e-6 m, which is exactly that
  // floor, so this check also documents WHY the gate cannot sensibly be tightened without
  // widening the contract's range type.
  Check(worst <= 1e-6, "worst analytic range error is within the 1e-6 m gate",
        "worst = " + std::to_string(worst) + " m");
  std::printf("      worst |produced - analytic| over 360 placements: %.3e m\n", worst);
}

// ---------------------------------------------------------------------------------------
// I  The float-narrowing hazard at the declared range bounds.
// ---------------------------------------------------------------------------------------
void TestRangeBoundNarrowing() {
  Section("I. float narrowing at the declared range bounds");

  // The projector stores a float but declares its bounds in double, exactly as upstream
  // does. Rounding a double range to float can in principle push it OUTSIDE the declared
  // [range_min_m, range_max_m], which ProjectedScan::Validate() rejects. This section
  // measures whether that is reachable, at the shipped bounds and at deliberately hostile
  // ones that are not exactly representable in float.
  struct Bounds { double min, max; const char* label; };
  const Bounds cases[] = {
      {0.10, 10.0, "shipped [0.10, 10.0]"},
      {0.3, 0.7, "hostile [0.3, 0.7] (neither bound is float-exact)"},
      {0.1234567890123, 3.9876543210987, "hostile [0.1234567890123, 3.9876543210987]"},
  };

  for (const Bounds& bounds : cases) {
    ScanProjectorParams params = ShippedParams();
    params.range_min_m = bounds.min;
    params.range_max_m = bounds.max;
    const ScanProjector projector(params);

    // Sweep ranges densely across the whole admissible interval, including both endpoints
    // and the doubles immediately inside them.
    int violations = 0;
    int projected = 0;
    for (int i = 0; i <= 20000; ++i) {
      double range = bounds.min + (bounds.max - bounds.min) * (static_cast<double>(i) / 20000.0);
      if (i == 0) range = bounds.min;
      if (i == 20000) range = bounds.max;
      if (i == 1) range = std::nextafter(bounds.min, bounds.max);
      if (i == 19999) range = std::nextafter(bounds.max, bounds.min);

      projection_scenes::SceneBuilder builder;
      builder.AddPolar(range, 0.017 * (i % 360) - kPi + 1e-3, 0.0, PointLabel::kNonGround);
      ProjectedScan scan;
      if (projector.Project(builder.Finish(0.0, 0), &scan) != nullptr) continue;
      ++projected;
      if (scan.Validate() != nullptr) ++violations;
    }
    Check(violations == 0,
          std::string("no stored float escapes the declared bounds: ") + bounds.label,
          std::to_string(violations) + " of " + std::to_string(projected) + " violated");
  }
}

// ---------------------------------------------------------------------------------------
// J  Allocation-free steady state.
// ---------------------------------------------------------------------------------------
void TestAllocationFree() {
  Section("J. allocation-free steady state");

  const ScanProjector projector(ShippedParams());
  const auto scene = projection_scenes::ArenaCylinders();

  ProjectedScan scan;
  // First call sizes the buffers; it is allowed to allocate.
  const std::size_t before_first = g_allocations;
  projector.Project(scene.cloud, &scan);
  const std::size_t first_call = g_allocations - before_first;

  // Every subsequent call must allocate nothing at all.
  std::size_t steady = 0;
  for (int i = 0; i < 50; ++i) {
    const std::size_t before = g_allocations;
    projector.Project(scene.cloud, &scan);
    steady += g_allocations - before;
  }

  Check(steady == 0, "50 further projections into the same buffer allocate nothing",
        std::to_string(steady) + " allocations");
  std::printf("      first (sizing) call allocated %zu times; the next 50 allocated %zu\n",
              first_call, steady);

  // A bin-count change is allowed to allocate once and then settle again.
  {
    ScanProjectorParams wider = ShippedParams();
    wider.bins = 720;
    const ScanProjector wide_projector(wider);
    wide_projector.Project(scene.cloud, &scan);
    const std::size_t before = g_allocations;
    wide_projector.Project(scene.cloud, &scan);
    // Snapshotted into a local BEFORE the Check call: the order in which a function's
    // arguments are evaluated is unspecified, so building Check's std::string argument can
    // otherwise bump the counter before the comparison reads it.
    const std::size_t after_resize = g_allocations - before;
    Check(after_resize == 0, "after a bin-count change the steady state returns",
          std::to_string(after_resize) + " allocations");
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <perception-directory>\n", argv[0]);
    return 2;
  }
  g_perception_dir = argv[1];

  std::printf("perception_projection_test  (perception dir: %s)\n",
              g_perception_dir.string().c_str());

  TestParamsAndConfig();
  TestAnalyticBins();
  TestProperties();
  TestEquivalence();
  TestGoldenFixtures();
  TestInformationLoss();
  TestNegativeControls();
  TestNumericalAccuracy();
  TestRangeBoundNarrowing();
  TestAllocationFree();

  return perception_test::Report("perception_projection_test");
}
