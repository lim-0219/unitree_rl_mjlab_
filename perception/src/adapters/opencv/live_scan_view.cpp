#include "perception/adapters/opencv/live_scan_view.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace perception::adapters::opencv {
namespace {

constexpr int kHeaderHeight = 170;
constexpr int kTopDownSize = 660;
constexpr int kRightPanelWidth = 720;
constexpr int kMargin = 12;

// The documented sensor height at the G1's neutral standing pose
// (g1_mid360_extrinsic.md section 5.5). Used only as the reference the posture readout is
// compared against, never as a substitute for the measured value.
constexpr double kNominalStandingHeightM = 1.2654;

// ABSOLUTE world-z range for the height colour map. This replaces an earlier
// sensor-RELATIVE scale (sensor_z - 1.5 .. sensor_z + 0.5), which was actively misleading:
// the floor at z = 0 rendered a different colour depending on how high the sensor was, so
// identical world geometry changed appearance when the robot moved, and different
// geometries could render identically. A fixed world range makes the floor always the same
// colour and any height change immediately legible.
constexpr double kWorldZLo = -0.25;
constexpr double kWorldZHi = 2.00;

const cv::Scalar kBackground(24, 20, 18);
const cv::Scalar kGrid(64, 58, 52);
const cv::Scalar kGridStrong(96, 88, 80);
const cv::Scalar kText(232, 232, 232);
const cv::Scalar kTextDim(150, 150, 150);
const cv::Scalar kAccent(0, 150, 255);
const cv::Scalar kOkGreen(90, 210, 110);
const cv::Scalar kWarnAmber(40, 190, 240);
const cv::Scalar kBadRed(60, 60, 245);

enum class Posture { kStanding, kTilted, kFallen };

struct Frame {
  std::vector<Eigen::Vector3f> points;
  std::vector<double> ranges;
  std::vector<int> ray_indices;
  Eigen::Vector3d sensor_origin{0.0, 0.0, 0.0};
  Eigen::Vector3d sensor_forward{1.0, 0.0, 0.0};
  Eigen::Vector3d sensor_up{0.0, 0.0, 1.0};
  double sensor_roll_deg = 0.0;
  double sensor_pitch_deg = 0.0;
  double sensor_yaw_deg = 0.0;
  double torso_z = 0.0;
  double torso_tilt_deg = 0.0;
  double lowest_return_z = 0.0;
  core::ScanStats stats;
  double stamp_s = 0.0;
  bool valid = false;
};

Posture ClassifyPosture(const Frame& frame) {
  if (frame.sensor_origin.z() < 0.60 || frame.torso_tilt_deg > 60.0) {
    return Posture::kFallen;
  }
  if (frame.sensor_origin.z() < 1.05 || frame.torso_tilt_deg > 20.0) {
    return Posture::kTilted;
  }
  return Posture::kStanding;
}

void PutText(cv::Mat& canvas, const std::string& text, cv::Point origin, double scale,
             const cv::Scalar& colour, int thickness = 1, int font = cv::FONT_HERSHEY_SIMPLEX) {
  cv::putText(canvas, text, origin, font, scale, colour, thickness, cv::LINE_AA);
}

// Precomputed turbo ramp; building a 1x1 Mat per point would dominate the draw cost.
const std::vector<cv::Vec3b>& TurboLut() {
  static const std::vector<cv::Vec3b> lut = [] {
    cv::Mat ramp(1, 256, CV_8UC1);
    for (int i = 0; i < 256; ++i) {
      ramp.at<unsigned char>(0, i) = static_cast<unsigned char>(i);
    }
    cv::Mat coloured;
    cv::applyColorMap(ramp, coloured, cv::COLORMAP_TURBO);
    std::vector<cv::Vec3b> table(256);
    for (int i = 0; i < 256; ++i) {
      table[i] = coloured.at<cv::Vec3b>(0, i);
    }
    return table;
  }();
  return lut;
}

cv::Vec3b TurboAt(double t) {
  return TurboLut()[static_cast<std::size_t>(std::clamp(t, 0.0, 1.0) * 255.0)];
}

cv::Scalar WorldHeightColour(double world_z) {
  const double t = (world_z - kWorldZLo) / (kWorldZHi - kWorldZLo);
  const cv::Vec3b bgr = TurboAt(t);
  return cv::Scalar(bgr[0], bgr[1], bgr[2]);
}

}  // namespace

struct LiveScanView::Impl {
  integration::LidarVisualizationConfig config;
  int azimuth_rays = 360;
  int elevation_rays = 32;
  double max_range_m = 40.0;
  double view_range_m = 8.0;

  std::mutex mutex;
  Frame pending;
  bool has_pending = false;
  std::string snapshot_path;

  std::atomic<bool> running{false};
  std::thread render_thread;

  void RenderLoop();
  void Draw(cv::Mat& canvas, const Frame& frame) const;
  void DrawHeader(cv::Mat& canvas, const Frame& frame) const;
  void DrawTopDown(cv::Mat& canvas, const cv::Rect& panel, const Frame& frame) const;
  void DrawRangeImage(cv::Mat& canvas, const cv::Rect& panel, const Frame& frame) const;
  void DrawStatePanel(cv::Mat& canvas, const cv::Rect& panel, const Frame& frame) const;
};

LiveScanView::LiveScanView() : impl_(std::make_unique<Impl>()) {}

LiveScanView::~LiveScanView() { Stop(); }

void LiveScanView::Configure(const integration::LidarVisualizationConfig& config,
                             int azimuth_rays, int elevation_rays, double max_range_m) {
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->config = config;
  impl_->azimuth_rays = std::max(1, azimuth_rays);
  impl_->elevation_rays = std::max(1, elevation_rays);
  impl_->max_range_m = max_range_m;
}

void LiveScanView::Start() {
  if (impl_->running.exchange(true)) {
    return;
  }
  impl_->render_thread = std::thread([this] { impl_->RenderLoop(); });
}

void LiveScanView::Stop() {
  if (!impl_->running.exchange(false)) {
    return;
  }
  if (impl_->render_thread.joinable()) {
    impl_->render_thread.join();
  }
}

bool LiveScanView::running() const { return impl_->running.load(); }

void LiveScanView::RequestSnapshot(const std::string& path) {
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->snapshot_path = path;
}

void LiveScanView::Publish(const core::TimedPointCloud& cloud,
                           const core::FrameTransformSnapshot& snapshot,
                           const core::ScanStats& stats) {
  if (!impl_->running.load()) {
    return;
  }

  Frame frame;
  frame.points.reserve(cloud.points.size());
  frame.ranges.reserve(cloud.points.size());
  frame.ray_indices.reserve(cloud.points.size());
  double lowest = std::numeric_limits<double>::max();
  for (const auto& point : cloud.points) {
    frame.points.push_back(point.position);
    frame.ranges.push_back(point.range_m);
    frame.ray_indices.push_back(point.ray_index);
    lowest = std::min(lowest, static_cast<double>(point.position.z()));
  }
  frame.lowest_return_z = cloud.points.empty() ? 0.0 : lowest;

  const Eigen::Matrix3d sensor_rotation = snapshot.world_from_sensor.linear();
  frame.sensor_origin = snapshot.world_from_sensor.translation();
  frame.sensor_forward = sensor_rotation.col(0);
  frame.sensor_up = sensor_rotation.col(2);

  // Intrinsic Z-Y-X Euler angles, purely for the readout.
  const Eigen::Vector3d euler = sensor_rotation.eulerAngles(2, 1, 0);
  frame.sensor_yaw_deg = euler[0] * 180.0 / M_PI;
  frame.sensor_pitch_deg = euler[1] * 180.0 / M_PI;
  frame.sensor_roll_deg = euler[2] * 180.0 / M_PI;

  // Torso tilt: angle between the torso's own +Z and world +Z. This is the posture cue
  // that survives the sensor being mounted upside down.
  const Eigen::Matrix3d torso_rotation = snapshot.world_from_base.linear();
  frame.torso_z = snapshot.world_from_base.translation().z();
  frame.torso_tilt_deg = std::acos(std::clamp(torso_rotation(2, 2), -1.0, 1.0)) * 180.0 / M_PI;

  frame.stats = stats;
  frame.stamp_s = cloud.stamp_s;
  frame.valid = snapshot.valid;

  const std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->pending = std::move(frame);
  impl_->has_pending = true;
}

void LiveScanView::Impl::RenderLoop() {
  const std::string window = "Mid-360 live scan (perception)";
  cv::namedWindow(window, cv::WINDOW_NORMAL);
  const int width = kTopDownSize + kRightPanelWidth + 3 * kMargin;
  const int height = kHeaderHeight + kTopDownSize + 2 * kMargin;
  cv::resizeWindow(window, width, height);

  cv::Mat canvas(height, width, CV_8UC3, kBackground);
  Frame frame;

  while (running.load()) {
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (has_pending) {
        frame = pending;
        has_pending = false;
      }
    }

    canvas.setTo(kBackground);
    Draw(canvas, frame);
    cv::imshow(window, canvas);

    std::string save_to;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (!snapshot_path.empty() && frame.valid) {
        save_to.swap(snapshot_path);
      }
    }
    if (!save_to.empty()) {
      if (cv::imwrite(save_to, canvas)) {
        std::printf("[perception] live view snapshot written to %s\n", save_to.c_str());
      } else {
        std::printf("[perception] live view snapshot FAILED to write %s\n", save_to.c_str());
      }
      std::fflush(stdout);
    }

    const int delay_ms = std::max(1, static_cast<int>(1000.0 / std::max(1.0, config.refresh_hz)));
    cv::waitKey(delay_ms);
  }

  cv::destroyWindow(window);
  cv::waitKey(1);
}

void LiveScanView::Impl::Draw(cv::Mat& canvas, const Frame& frame) const {
  DrawHeader(canvas, frame);

  const cv::Rect top_down(kMargin, kHeaderHeight, kTopDownSize, kTopDownSize);
  const cv::Rect right(kTopDownSize + 2 * kMargin, kHeaderHeight, kRightPanelWidth, 330);
  const cv::Rect state(kTopDownSize + 2 * kMargin, kHeaderHeight + 342, kRightPanelWidth,
                       kTopDownSize - 342);
  DrawTopDown(canvas, top_down, frame);
  DrawRangeImage(canvas, right, frame);
  DrawStatePanel(canvas, state, frame);
}

// The always-visible world-frame readout. Everything a viewer needs to tell "the robot
// fell" from "the robot is standing" is in here, at a size that cannot be missed.
void LiveScanView::Impl::DrawHeader(cv::Mat& canvas, const Frame& frame) const {
  char line[320];
  PutText(canvas, "Livox Mid-360 on Unitree G1   torso_link @ [0.0002835 0.00003 0.428434]"
                  "   rpy [pi, 0.0511207, 0]  (upside-down mount)",
          {kMargin, 20}, 0.44, kTextDim);

  if (!frame.valid) {
    PutText(canvas, "WAITING FOR FIRST SCAN", {kMargin, 76}, 1.0, kTextDim, 2,
            cv::FONT_HERSHEY_DUPLEX);
    return;
  }

  // ---- posture banner -------------------------------------------------------------
  const Posture posture = ClassifyPosture(frame);
  const char* label = posture == Posture::kStanding ? "STANDING"
                      : posture == Posture::kTilted ? "TILTED"
                                                    : "FALLEN";
  const cv::Scalar colour = posture == Posture::kStanding ? kOkGreen
                            : posture == Posture::kTilted ? kWarnAmber
                                                          : kBadRed;
  const cv::Rect banner(kMargin, 32, 250, 60);
  cv::rectangle(canvas, banner, colour, cv::FILLED);
  const cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_DUPLEX, 1.15, 2, nullptr);
  PutText(canvas, label,
          {banner.x + (banner.width - text_size.width) / 2,
           banner.y + (banner.height + text_size.height) / 2 - 2},
          1.15, cv::Scalar(20, 18, 16), 2, cv::FONT_HERSHEY_DUPLEX);

  // ---- big world-frame numbers ----------------------------------------------------
  int x = banner.x + banner.width + 22;

  std::snprintf(line, sizeof(line), "SENSOR WORLD Z  %.3f m", frame.sensor_origin.z());
  PutText(canvas, line, {x, 58}, 0.78, colour, 2, cv::FONT_HERSHEY_DUPLEX);
  std::snprintf(line, sizeof(line), "(nominal standing %.3f m)", kNominalStandingHeightM);
  PutText(canvas, line, {x, 82}, 0.44, kTextDim);

  x += 380;
  std::snprintf(line, sizeof(line), "TORSO TILT  %.1f deg", frame.torso_tilt_deg);
  PutText(canvas, line, {x, 58}, 0.78, colour, 2, cv::FONT_HERSHEY_DUPLEX);
  std::snprintf(line, sizeof(line), "torso world z %.3f m", frame.torso_z);
  PutText(canvas, line, {x, 82}, 0.44, kTextDim);

  x += 330;
  std::snprintf(line, sizeof(line), "ACCEPTED  %d / %d", frame.stats.accepted,
                frame.stats.rays_cast);
  PutText(canvas, line, {x, 58}, 0.72, kText, 2, cv::FONT_HERSHEY_DUPLEX);
  std::snprintf(line, sizeof(line), "SELF-HIT  %.2f %%  (%d rays)",
                100.0 * frame.stats.SelfHitFraction(), frame.stats.self_rejected);
  PutText(canvas, line, {x, 84}, 0.60, kText, 1, cv::FONT_HERSHEY_DUPLEX);

  // ---- full world pose line -------------------------------------------------------
  std::snprintf(line, sizeof(line),
                "sensor world XYZ = (%+.3f, %+.3f, %+.3f) m     sensor RPY = "
                "(%+.1f, %+.1f, %+.1f) deg     t = %.3f s",
                frame.sensor_origin.x(), frame.sensor_origin.y(), frame.sensor_origin.z(),
                frame.sensor_roll_deg, frame.sensor_pitch_deg, frame.sensor_yaw_deg,
                frame.stamp_s);
  PutText(canvas, line, {kMargin, 116}, 0.52, kText);

  std::snprintf(line, sizeof(line),
                "no-hit %d   range-rejected %d   scan %.0f us   lowest return z %+.3f m   |   "
                "exact mj_ray + aperture mask   |   %d x %d uniform grid (rosette approximation)",
                frame.stats.no_hit, frame.stats.range_rejected, frame.stats.scan_wall_time_us,
                frame.lowest_return_z, azimuth_rays, elevation_rays);
  PutText(canvas, line, {kMargin, 140}, 0.42, kTextDim);

  cv::line(canvas, {kMargin, 154}, {canvas.cols - kMargin, 154}, kGrid, 1);
}

void LiveScanView::Impl::DrawTopDown(cv::Mat& canvas, const cv::Rect& panel,
                                     const Frame& frame) const {
  cv::rectangle(canvas, panel, kGrid, 1);
  PutText(canvas, "TOP-DOWN (world XY, sensor-centred)", {panel.x + 8, panel.y + 18}, 0.45,
          kText);

  const cv::Point centre(panel.x + panel.width / 2, panel.y + panel.height / 2);
  const double pixels_per_metre = 0.5 * panel.width / view_range_m;

  for (int ring = 1; ring * pixels_per_metre < 0.5 * panel.width; ++ring) {
    cv::circle(canvas, centre, static_cast<int>(ring * pixels_per_metre), kGrid, 1, cv::LINE_AA);
    if (ring % 2 == 0) {
      PutText(canvas, std::to_string(ring) + " m",
              {static_cast<int>(centre.x + ring * pixels_per_metre) - 22, centre.y - 4}, 0.34,
              kTextDim);
    }
  }
  cv::line(canvas, {panel.x, centre.y}, {panel.x + panel.width, centre.y}, kGrid, 1);
  cv::line(canvas, {centre.x, panel.y}, {centre.x, panel.y + panel.height}, kGrid, 1);

  if (!frame.valid) {
    return;
  }

  // Points coloured by ABSOLUTE world height, so the floor is always the same colour.
  for (const auto& point : frame.points) {
    const double dx = point.x() - frame.sensor_origin.x();
    const double dy = point.y() - frame.sensor_origin.y();
    if (std::abs(dx) > view_range_m || std::abs(dy) > view_range_m) {
      continue;
    }
    const cv::Point pixel(static_cast<int>(centre.x + dx * pixels_per_metre),
                          static_cast<int>(centre.y - dy * pixels_per_metre));
    if (!panel.contains(pixel)) {
      continue;
    }
    cv::circle(canvas, pixel, 1, WorldHeightColour(point.z()), cv::FILLED, cv::LINE_AA);
  }

  cv::circle(canvas, centre, 6, kAccent, cv::FILLED, cv::LINE_AA);
  const double forward_x = frame.sensor_forward.x();
  const double forward_y = frame.sensor_forward.y();
  const double forward_norm = std::max(1e-9, std::hypot(forward_x, forward_y));
  const cv::Point tip(static_cast<int>(centre.x + forward_x / forward_norm * 46),
                      static_cast<int>(centre.y - forward_y / forward_norm * 46));
  cv::arrowedLine(canvas, centre, tip, kAccent, 2, cv::LINE_AA, 0, 0.3);
  PutText(canvas, "sensor +X", {tip.x + 6, tip.y}, 0.36, kAccent);

  // Absolute world XY of the sensor, so this panel is not purely sensor-relative.
  char line[128];
  std::snprintf(line, sizeof(line), "panel centre = sensor at world (%+.2f, %+.2f)",
                frame.sensor_origin.x(), frame.sensor_origin.y());
  PutText(canvas, line, {panel.x + 8, panel.y + panel.height - 12}, 0.38, kTextDim);
}

void LiveScanView::Impl::DrawRangeImage(cv::Mat& canvas, const cv::Rect& panel,
                                        const Frame& frame) const {
  cv::rectangle(canvas, panel, kGrid, 1);
  PutText(canvas, "RANGE IMAGE (azimuth x elevation, sensor frame)", {panel.x + 8, panel.y + 18},
          0.45, kText);

  if (!frame.valid) {
    return;
  }

  cv::Mat grid(elevation_rays, azimuth_rays, CV_8UC1, cv::Scalar(0));
  cv::Mat has_return(elevation_rays, azimuth_rays, CV_8UC1, cv::Scalar(0));
  for (std::size_t i = 0; i < frame.ray_indices.size(); ++i) {
    const int index = frame.ray_indices[i];
    if (index < 0) {
      continue;
    }
    const int azimuth = index / elevation_rays;
    const int elevation = index % elevation_rays;
    if (azimuth >= azimuth_rays || elevation >= elevation_rays) {
      continue;
    }
    const double normalized = 1.0 - std::clamp(frame.ranges[i] / 10.0, 0.0, 1.0);
    grid.at<unsigned char>(elevation_rays - 1 - elevation, azimuth) =
        static_cast<unsigned char>(20 + 235 * normalized);
    has_return.at<unsigned char>(elevation_rays - 1 - elevation, azimuth) = 255;
  }

  cv::Mat coloured;
  cv::applyColorMap(grid, coloured, cv::COLORMAP_TURBO);
  coloured.setTo(cv::Scalar(30, 26, 24), has_return == 0);

  const int draw_width = panel.width - 24;
  const int draw_height = 224;
  cv::Mat scaled;
  cv::resize(coloured, scaled, cv::Size(draw_width, draw_height), 0, 0, cv::INTER_NEAREST);
  const cv::Rect target(panel.x + 12, panel.y + 30, draw_width, draw_height);
  scaled.copyTo(canvas(target));
  cv::rectangle(canvas, target, kGrid, 1);

  PutText(canvas, "-180", {target.x - 4, target.y + draw_height + 16}, 0.36, kTextDim);
  PutText(canvas, "0 (fwd)", {target.x + draw_width / 2 - 24, target.y + draw_height + 16}, 0.36,
          kTextDim);
  PutText(canvas, "+180", {target.x + draw_width - 34, target.y + draw_height + 16}, 0.36,
          kTextDim);
  PutText(canvas, "+52", {target.x + 6, target.y + 14}, 0.34, kText);
  PutText(canvas, "-7", {target.x + 6, target.y + draw_height - 6}, 0.34, kText);
  PutText(canvas, "range: RED near / BLUE far (10 m clamp); dark = no return or self-rejected",
          {target.x, target.y + draw_height + 36}, 0.37, kTextDim);
}

// Height gauge + absolute-height colour bar. The gauge is the single most direct
// "did it fall" cue: a marker on an absolute 0..1.6 m scale against the nominal standing
// height, which needs no interpretation of the point pattern at all.
void LiveScanView::Impl::DrawStatePanel(cv::Mat& canvas, const cv::Rect& panel,
                                        const Frame& frame) const {
  cv::rectangle(canvas, panel, kGrid, 1);
  PutText(canvas, "SENSOR HEIGHT ABOVE WORLD Z=0", {panel.x + 8, panel.y + 20}, 0.45, kText);

  const int bar_x = panel.x + 20;
  const int bar_w = panel.width - 150;
  const int bar_y = panel.y + 36;
  const int bar_h = 34;
  constexpr double kGaugeMax = 1.6;

  cv::rectangle(canvas, cv::Rect(bar_x, bar_y, bar_w, bar_h), kGridStrong, 1);

  if (frame.valid) {
    const Posture posture = ClassifyPosture(frame);
    const cv::Scalar colour = posture == Posture::kStanding ? kOkGreen
                              : posture == Posture::kTilted ? kWarnAmber
                                                            : kBadRed;
    const int fill = static_cast<int>(
        std::clamp(frame.sensor_origin.z() / kGaugeMax, 0.0, 1.0) * (bar_w - 2));
    cv::rectangle(canvas, cv::Rect(bar_x + 1, bar_y + 1, std::max(1, fill), bar_h - 2), colour,
                  cv::FILLED);

    char value[64];
    std::snprintf(value, sizeof(value), "%.3f m", frame.sensor_origin.z());
    PutText(canvas, value, {bar_x + bar_w + 14, bar_y + 25}, 0.68, colour, 2,
            cv::FONT_HERSHEY_DUPLEX);
  }

  // Nominal standing marker.
  const int nominal_x =
      bar_x + static_cast<int>(kNominalStandingHeightM / kGaugeMax * (bar_w - 2));
  cv::line(canvas, {nominal_x, bar_y - 6}, {nominal_x, bar_y + bar_h + 6}, kText, 2);
  PutText(canvas, "nominal 1.265", {nominal_x - 46, bar_y + bar_h + 22}, 0.36, kText);

  for (int tick = 0; tick <= 8; ++tick) {
    const double value = tick * 0.2;
    const int tick_x = bar_x + static_cast<int>(value / kGaugeMax * (bar_w - 2));
    cv::line(canvas, {tick_x, bar_y + bar_h}, {tick_x, bar_y + bar_h + 5}, kGridStrong, 1);
    if (tick % 2 == 0) {
      char label[16];
      std::snprintf(label, sizeof(label), "%.1f", value);
      PutText(canvas, label, {tick_x - 8, bar_y + bar_h + 40}, 0.34, kTextDim);
    }
  }

  // Absolute-height colour bar for the top-down panel.
  const int cb_y = bar_y + bar_h + 62;
  PutText(canvas, "TOP-DOWN POINT COLOUR = ABSOLUTE WORLD Z", {panel.x + 8, cb_y - 8}, 0.42,
          kText);
  const int cb_h = 20;
  for (int i = 0; i < bar_w; ++i) {
    const cv::Vec3b bgr = TurboAt(static_cast<double>(i) / bar_w);
    cv::line(canvas, {bar_x + i, cb_y}, {bar_x + i, cb_y + cb_h}, cv::Scalar(bgr[0], bgr[1], bgr[2]),
             1);
  }
  cv::rectangle(canvas, cv::Rect(bar_x, cb_y, bar_w, cb_h), kGrid, 1);
  for (double z = 0.0; z <= kWorldZHi; z += 0.5) {
    const int tick_x = bar_x + static_cast<int>((z - kWorldZLo) / (kWorldZHi - kWorldZLo) * bar_w);
    cv::line(canvas, {tick_x, cb_y + cb_h}, {tick_x, cb_y + cb_h + 5}, kGridStrong, 1);
    char label[16];
    std::snprintf(label, sizeof(label), "%.1f", z);
    PutText(canvas, label, {tick_x - 9, cb_y + cb_h + 20}, 0.34, kTextDim);
  }
  PutText(canvas, "fixed world scale, NOT sensor-relative: the floor keeps one colour however"
                  " the robot moves",
          {bar_x, cb_y + cb_h + 40}, 0.37, kTextDim);

  int y = cb_y + cb_h + 68;
  PutText(canvas, "The sensor frame is UPSIDE DOWN: sensor +52 deg is 54.9 deg BELOW the robot",
          {panel.x + 8, y}, 0.38, kTextDim);
  y += 17;
  PutText(canvas, "horizon, so the top of the range image is the floor close to the robot.",
          {panel.x + 8, y}, 0.38, kTextDim);
}

}  // namespace perception::adapters::opencv
