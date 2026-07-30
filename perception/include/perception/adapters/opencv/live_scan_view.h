// LiveScanView - the interactive window for watching the Mid-360 scan as the sim runs.
//
// WHY THIS AND NOT THE MUJOCO VIEWER'S 3D SCENE. The vendored `simulate` app does expose a
// user-owned mjvScene (`Simulate::user_scn`), and adapters/mujoco/live_scan_overlay.h fills
// it. But in MuJoCo 3.3.6 the app only APPENDS those geoms into the render scene in PASSIVE
// mode (`simulate.cc`, the `else if (m_passive_ && d_passive_)` branch). This application
// constructs Simulate with `is_passive_ = false`, so `Sync()` collects the user geoms into a
// scratch vector and then never uses them. The 3D overlay would therefore be a silent
// no-op here, which is why it is not the default. See perception/docs/architecture.md.
//
// THREADING. Exactly the pattern DpcbfVisualizer already uses in this repository: a
// dedicated render thread that owns the OpenCV window, and `Publish()` handing it COPIES
// under a mutex. This module never touches mjModel/mjData, never holds sim.mtx, and never
// mutates perception state - it is a read-only sink.
#ifndef PERCEPTION_ADAPTERS_OPENCV_LIVE_SCAN_VIEW_H_
#define PERCEPTION_ADAPTERS_OPENCV_LIVE_SCAN_VIEW_H_

#include <memory>
#include <string>

#include "perception/core/contracts/frame_transform_snapshot.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/integration/perception_config.h"

namespace perception::adapters::opencv {

class LiveScanView {
 public:
  LiveScanView();
  ~LiveScanView();

  LiveScanView(const LiveScanView&) = delete;
  LiveScanView& operator=(const LiveScanView&) = delete;

  // `azimuth_rays` / `elevation_rays` describe the ray table so the range-image panel can
  // map a ray index back to its grid cell.
  void Configure(const integration::LidarVisualizationConfig& config, int azimuth_rays,
                 int elevation_rays, double max_range_m);

  void Start();
  void Stop();
  bool running() const;

  // Copies the frame into the back buffer. Cheap and non-blocking beyond a short mutex.
  void Publish(const core::TimedPointCloud& cloud, const core::FrameTransformSnapshot& snapshot,
               const core::ScanStats& stats);

  // Writes the next rendered canvas to `path`. A debugging aid for confirming what the
  // window is showing (e.g. over ssh, or in CI); the window itself is the deliverable.
  void RequestSnapshot(const std::string& path);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception::adapters::opencv

#endif  // PERCEPTION_ADAPTERS_OPENCV_LIVE_SCAN_VIEW_H_
