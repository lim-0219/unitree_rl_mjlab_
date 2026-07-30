// projection_golden_capture - produces this phase's committed ProjectedScan fixtures.
//
// NOT A TEST. This is the tool that writes tests/fixtures/projection_golden/, which
// perception_projection_test then checks against, and which P8's detector port consumes as
// its input corpus (architecture doc section 10, P7 "Artifacts").
//
// It is the same device as P3's oracle_golden_capture, with one difference that is worth
// stating plainly rather than glossing: P3's capture ran against code that was about to be
// deleted, so the fixture recorded a behaviour that could not be re-derived afterwards. Here
// there is no pre-change code - the projector is new - so the golden's authority does NOT
// come from being captured first. It comes from being captured through the VERBATIM UPSTREAM
// REFERENCE (tests/regression/p2l_reference.h), not through the new projector. The committed
// ranges are upstream's numbers. The test then requires the new projector to reproduce them.
//
// That ordering is the whole point. A golden captured from the implementation under test
// proves only that the implementation is stable, which is the weaker claim and the easy one
// to accidentally make.
//
// The manifest additionally pins the INPUT each scan was computed from, by digest. A change
// to the scene generator that silently altered the corpus would otherwise be absorbed by
// regenerating the goldens, leaving a green test that no longer tests the same thing.

#include <cinttypes>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "perception/core/contracts/projected_scan.h"
#include "perception/core/diagnostics/dump_file.h"
#include "perception/core/projection/scan_projector.h"

#include "p2l_reference.h"
#include "projection_scenes.h"

namespace diagnostics = perception::core::diagnostics;

using perception::core::ProjectedScan;
using perception::core::ScanProjector;
using perception::core::ScanProjectorParams;

namespace {

// The shipped projection.* values, restated here as literals on purpose. The capture tool
// must not read configs/perception.yaml: a fixture whose meaning depends on a file that can
// change is not a fixture. The projection test asserts these literals still match the
// shipped config, so a deliberate config change fails a test instead of silently
// re-interpreting the committed goldens.
ScanProjectorParams ShippedParams() {
  ScanProjectorParams params;
  params.bins = 360;
  params.angle_min_rad = -perception::core::kPi;
  params.angle_max_rad = perception::core::kPi;
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

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <output-directory>\n", argv[0]);
    return 2;
  }
  const std::filesystem::path output_directory = argv[1];
  std::filesystem::create_directories(output_directory);

  const ScanProjectorParams params = ShippedParams();
  const p2l_reference::Params reference_params = ReferenceParams(params);
  const ScanProjector projector(params);

  diagnostics::RunProvenance provenance;
  provenance.producer = "projection_golden_capture";
  provenance.config_source = "perception/configs/perception.yaml (projection.*, transcribed)";

  diagnostics::DumpWriter writer;
  std::string error;
  const std::filesystem::path scans_path = output_directory / "projected_scans.jsonl";
  if (!writer.Open(scans_path, diagnostics::DumpFormat::kJsonl,
                   diagnostics::RecordType::kProjectedScan, provenance,
                   /*retained_mask=*/0, error)) {
    std::fprintf(stderr, "DumpWriter::Open failed: %s\n", error.c_str());
    return 1;
  }

  const std::filesystem::path manifest_path = output_directory / "scene_manifest.txt";
  std::FILE* manifest = std::fopen(manifest_path.string().c_str(), "w");
  if (manifest == nullptr) {
    std::fprintf(stderr, "could not open %s\n", manifest_path.string().c_str());
    return 1;
  }

  std::fprintf(manifest,
               "# P7 projection golden fixtures.\n"
               "#\n"
               "# Produced by perception_projection_golden_capture. The `ranges` committed in\n"
               "# projected_scans.jsonl are the output of the VERBATIM upstream reference\n"
               "# (tests/regression/p2l_reference.h), NOT of ScanProjector - see that file's\n"
               "# header for why the direction matters.\n"
               "#\n"
               "# input_digest  FNV-1a 64 over the raw bits of every point's xyz and label,\n"
               "#               in order, over the WHOLE scene including non-projected labels.\n"
               "# output_digest FNV-1a 64 over the raw bits of the %d committed range floats,\n"
               "#               which pins the NaNs that no decimal comparison can pin.\n"
               "#\n"
               "# bins=%d angle=[%.17g, %.17g] range=[%.17g, %.17g] band=[%.17g, %.17g]\n"
               "#\n"
               "# scene  points  non_ground  binned  occupied_bins  max_bin  input_digest  "
               "output_digest  guard_reads\n",
               params.bins, params.bins, params.angle_min_rad, params.angle_max_rad,
               params.range_min_m, params.range_max_m, params.height_band_min_m,
               params.height_band_max_m);

  int total_guard_reads = 0;
  const std::vector<projection_scenes::Scene> scenes = projection_scenes::AllScenes();
  for (const projection_scenes::Scene& scene : scenes) {
    if (const char* reason = scene.cloud.Validate()) {
      std::fprintf(stderr, "scene '%s' is not a valid LabeledPointCloud: %s\n",
                   scene.name.c_str(), reason);
      return 1;
    }

    const std::vector<Eigen::Vector3f> points =
        projection_scenes::NonGroundPoints(scene.cloud);
    const p2l_reference::Result reference =
        p2l_reference::Run(points, reference_params, p2l_reference::Arithmetic::kDouble);
    total_guard_reads += reference.guard_reads;

    // The committed record is built from the REFERENCE ranges. Everything else - the
    // metadata and the census - comes from the projector, because upstream has no
    // equivalent and inventing it by hand would be a second implementation to get wrong.
    ProjectedScan scan;
    if (const char* reason = projector.Project(scene.cloud, &scan)) {
      std::fprintf(stderr, "projection failed on scene '%s': %s\n", scene.name.c_str(),
                   reason);
      return 1;
    }

    // Overwrite the ranges with upstream's, bin for bin, applying the TWO documented
    // translations and nothing else.
    for (int bin = 0; bin < params.bins; ++bin) {
      const float value = reference.ranges[static_cast<std::size_t>(bin)];
      // (1) Upstream's never-measured sentinel is +inf; this contract's is NaN. The
      // conversion is done HERE, at the format boundary, and nowhere else.
      scan.ranges[static_cast<std::size_t>(bin)] =
          std::isinf(value) ? std::numeric_limits<float>::quiet_NaN() : value;
    }

    // (2) Fold the guard slot into the last bin, by upstream's OWN nearest-return rule.
    //
    // The guard slot holds what upstream wrote one element past the end of its array for
    // bearings within ~1e-14 rad of +pi (see p2l_reference.h). Committing the corpus with
    // that return simply missing would bake an upstream buffer overrun into the fixture P8
    // is going to be handed, and would make the golden an invalid sample of the scan the
    // detector is promised. Clamping it into the last bin is exactly divergence 1 in
    // scan_projector.h - the port's documented repair - applied here so the committed
    // ranges are still upstream's NUMBERS while being a well-formed scan.
    {
      const float guard = reference.ranges[static_cast<std::size_t>(reference.ranges_size)];
      if (!std::isinf(guard)) {
        float& last = scan.ranges[static_cast<std::size_t>(params.bins - 1)];
        if (std::isnan(last) || static_cast<double>(guard) < static_cast<double>(last)) {
          last = guard;
        }
      }
    }

    if (const char* reason = scan.Validate()) {
      std::fprintf(stderr, "golden scan for '%s' is not a valid ProjectedScan: %s\n",
                   scene.name.c_str(), reason);
      return 1;
    }

    if (!writer.Write(scan, scan.sequence, scan.stamp_s, error)) {
      std::fprintf(stderr, "DumpWriter::Write failed: %s\n", error.c_str());
      return 1;
    }

    int max_bin_points = 0;
    for (const std::int32_t count : scan.bin_point_counts) {
      if (count > max_bin_points) max_bin_points = count;
    }

    std::fprintf(manifest, "%-18s %6zu %6d %6d %6d %6d %016" PRIx64 " %016" PRIx64 " %3d\n",
                 scene.name.c_str(), scene.cloud.points.size(),
                 scene.cloud.segmentation.non_ground_points, scan.stats.binned_points,
                 scan.stats.occupied_bins, max_bin_points,
                 projection_scenes::SceneInputDigest(scene.cloud),
                 projection_scenes::ScanOutputDigest(scan.ranges), reference.guard_reads);

    std::printf("  %-18s %5zu points, %4d non-ground, %4d binned, %4d bins occupied, "
                "%d upstream out-of-bounds reads  (%s)\n",
                scene.name.c_str(), scene.cloud.points.size(),
                scene.cloud.segmentation.non_ground_points, scan.stats.binned_points,
                scan.stats.occupied_bins, reference.guard_reads, scene.purpose.c_str());
  }

  std::fclose(manifest);
  if (!writer.Close(error)) {
    std::fprintf(stderr, "DumpWriter::Close failed: %s\n", error.c_str());
    return 1;
  }

  std::printf("\nwrote %llu golden scans to %s\n",
              static_cast<unsigned long long>(writer.record_count()),
              scans_path.string().c_str());
  std::printf("wrote %s\n", manifest_path.string().c_str());
  std::printf("upstream reference read past the end of its buffer %d times across the corpus\n",
              total_guard_reads);
  return 0;
}
