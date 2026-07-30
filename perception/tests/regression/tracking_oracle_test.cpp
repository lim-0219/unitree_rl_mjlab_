// P9 upstream parity: AxisKalman2 against the real obstacle_detector KalmanFilter (Armadillo) on
// identical (dt, Q, R, measurement) sequences, plus the three upstream behaviours the port had to
// make a decision about.
//
// TEST-ONLY, built only when PERCEPTION_BUILD_UPSTREAM_ORACLE=ON.
//
// THE TOLERANCE. 1e-12 absolute, and it is NOT a modelling tolerance. Both sides execute the same
// five expressions of kalman.h:64-73 in the same order; the only permitted difference is
// floating-point association inside Armadillo's matrix products. There is no pinv-versus-QR
// substitution here as there was in P8, so unlike that phase there was no tolerance POLICY to
// settle - a disagreement above rounding would mean the port is wrong, not that two libraries
// legitimately differ.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "check.h"
#include "perception/core/tracking/axis_kalman.h"
#include "upstream_tracker_reference.h"

using perception::core::AxisKalman2;
using perception_test::Check;
using perception_test::Report;
using perception_test::Section;

namespace {

constexpr double kTolerance = 1e-12;

std::string Number(double value, int precision = 3) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*e", precision, value);
  return buffer;
}

// A deterministic, deliberately awkward measurement sequence: sign changes, a large jump, and a
// long quiet tail, so the comparison exercises both a growing and a collapsing covariance.
std::vector<double> MeasurementSequence(int count) {
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double t = 0.1 * i;
    double value = 0.7 * t + 0.05 * std::sin(3.1 * t) - 0.02 * std::cos(7.7 * t);
    if (i == 17) value += 0.9;   // A jump, to drive a large innovation.
    if (i == 31) value -= 0.45;  // And back.
    values.push_back(value);
  }
  return values;
}

struct Divergence {
  double value = 0.0;
  double rate = 0.0;
  double p00 = 0.0;
  double p01 = 0.0;
  double p10 = 0.0;
  double p11 = 0.0;

  double Worst() const {
    return std::max({value, rate, p00, p01, p10, p11});
  }
  void Accumulate(const AxisKalman2& port, const upstream_tracker_reference::AxisOracle& oracle) {
    value = std::max(value, std::abs(port.value() - oracle.value()));
    rate = std::max(rate, std::abs(port.rate() - oracle.rate()));
    p00 = std::max(p00, std::abs(port.value_variance() - oracle.p00()));
    p01 = std::max(p01, std::abs(port.p01() - oracle.p01()));
    p10 = std::max(p10, std::abs(port.p10() - oracle.p10()));
    p11 = std::max(p11, std::abs(port.rate_variance() - oracle.p11()));
  }
};

}  // namespace

int main() {
  // -------------------------------------------------------------------------------------
  Section("AxisKalman2 vs the real Armadillo KalmanFilter: fixed dt, upstream's own Q");
  // -------------------------------------------------------------------------------------
  // Upstream's numbers exactly: sampling time 1/100 s, Q = diag(0.01, 0.1), R = 1.0. Q is NOT
  // dt-scaled here, because this case is upstream's configuration and the point is to show the
  // arithmetic agrees on it before any reinterpretation is applied.
  {
    const double dt = 1.0 / 100.0;
    const double q_value = 0.01;
    const double q_rate = 0.10;
    const double r = 1.00;

    AxisKalman2 port;
    port.Initialize(0.0, 0.0, 1.0, 1.0);
    upstream_tracker_reference::AxisOracle oracle(0.0, 0.0, 1.0, 1.0);

    Divergence divergence;
    const std::vector<double> measurements = MeasurementSequence(200);
    for (const double measurement : measurements) {
      port.Predict(dt, q_value, q_rate);
      port.Correct(measurement, r);

      oracle.PredictOnly(dt, q_value, q_rate);
      oracle.Correct(measurement, r);

      divergence.Accumulate(port, oracle);
    }
    std::printf("      200 steps, upstream config: worst |delta| = %s"
                " (value %s, rate %s, P %s/%s/%s/%s)\n",
                Number(divergence.Worst()).c_str(), Number(divergence.value).c_str(),
                Number(divergence.rate).c_str(), Number(divergence.p00).c_str(),
                Number(divergence.p01).c_str(), Number(divergence.p10).c_str(),
                Number(divergence.p11).c_str());
    Check(divergence.Worst() < kTolerance,
          "state and full covariance agree with upstream within 1e-12 over 200 steps",
          "worst " + Number(divergence.Worst()));
    Check(std::abs(port.value()) > 1e-9, "the comparison actually moved the filter",
          "value=" + Number(port.value()));
  }

  // -------------------------------------------------------------------------------------
  Section("Same arithmetic under the port's VARIABLE, dt-scaled step");
  // -------------------------------------------------------------------------------------
  // The measurement-driven model's real operating regime: a nominal 10 Hz step with jitter, and Q
  // scaled by dt (change (a)). Both sides are given the same dt and the same scaled Q, so this
  // still isolates the arithmetic - it establishes that the agreement is a property of the
  // implementation and not of upstream's one particular dt.
  {
    const double process_variance = 0.01;
    const double process_rate_variance = 0.10;

    AxisKalman2 port;
    port.Initialize(0.2, 0.0, 0.0009, 0.64);
    upstream_tracker_reference::AxisOracle oracle(0.2, 0.0, 0.0009, 0.64);

    Divergence divergence;
    const std::vector<double> measurements = MeasurementSequence(150);
    for (std::size_t i = 0; i < measurements.size(); ++i) {
      // Jittered step: 0.1 s nominal, +/- 20%, plus one long gap to stand in for a coasting run.
      double dt = 0.1 * (1.0 + 0.2 * std::sin(2.7 * static_cast<double>(i)));
      if (i == 40) dt = 0.5;
      const double r = 0.0009 * (1.0 + 0.5 * std::abs(std::cos(1.3 * static_cast<double>(i))));

      const double q_value = process_variance * dt;
      const double q_rate = process_rate_variance * dt;

      port.Predict(dt, q_value, q_rate);
      port.Correct(measurements[i], r);

      oracle.PredictOnly(dt, q_value, q_rate);
      oracle.Correct(measurements[i], r);

      divergence.Accumulate(port, oracle);
    }
    std::printf("      150 steps, variable dt and R: worst |delta| = %s\n",
                Number(divergence.Worst()).c_str());
    Check(divergence.Worst() < kTolerance,
          "agreement holds under variable dt, variable R and dt-scaled Q",
          "worst " + Number(divergence.Worst()));
  }

  // -------------------------------------------------------------------------------------
  Section("Predict-only runs, i.e. what a coasting track does");
  // -------------------------------------------------------------------------------------
  // A confirmed track that misses several scans is predicted repeatedly with no correction. The
  // oracle has to publish q_pred into q_est by hand between steps, because upstream's
  // predictState() reads q_est and writes q_pred and only correctState() closes the loop - which
  // is itself part of why the timer model does not transfer.
  {
    AxisKalman2 port;
    port.Initialize(1.0, 0.65, 0.0009, 0.02);
    upstream_tracker_reference::AxisOracle oracle(1.0, 0.65, 0.0009, 0.02);

    Divergence divergence;
    for (int step = 0; step < 20; ++step) {
      const double dt = 0.1;
      port.Predict(dt, 0.01 * dt, 0.10 * dt);
      oracle.Predict(dt, 0.01 * dt, 0.10 * dt);
      divergence.Accumulate(port, oracle);
    }
    std::printf("      20 predict-only steps: worst |delta| = %s;"
                " value drifted to %s, variance grew to %s\n",
                Number(divergence.Worst()).c_str(), Number(port.value()).c_str(),
                Number(port.value_variance()).c_str());
    Check(divergence.Worst() < kTolerance, "a pure prediction run agrees within 1e-12",
          "worst " + Number(divergence.Worst()));
    // The port must genuinely coast: 20 steps of 0.1 s at 0.65 m/s is 1.3 m of travel.
    Check(std::abs(port.value() - (1.0 + 0.65 * 2.0)) < 1e-9,
          "the port coasts on its constant-rate model rather than standing still",
          "value=" + Number(port.value()));
    Check(port.value_variance() > 0.0009, "the coasting covariance grows");
  }

  // -------------------------------------------------------------------------------------
  Section("D1: upstream's timer 'coast' re-applies the stale measurement");
  // -------------------------------------------------------------------------------------
  // The behaviour that turned R11 from a determinism argument into a correctness one. A track is
  // corrected once, at rest-position 0.0 with a known velocity, and then the timer runs. If
  // updateState() coasted, the value would advance by dt * rate every tick. It does not: it is
  // pulled back toward the stale measurement and the rate decays.
  {
    upstream_tracker_reference::CircleObstacle seed;
    seed.center_x = 0.0;
    seed.center_y = 0.0;
    seed.velocity_x = 1.0;  // A metre per second, so a real coast would be unmistakable.
    seed.velocity_y = 0.0;
    seed.radius = 0.25;

    upstream_tracker_reference::TrackedObstacle track(seed, 1.0 / 100.0, 200, 0.01, 0.10, 1.00);

    upstream_tracker_reference::CircleObstacle measurement = seed;
    measurement.center_x = 0.0;
    track.correctState(measurement);
    const double rate_after_correct = track.getObstacle().velocity_x;

    for (int tick = 0; tick < 100; ++tick) track.updateState();  // One second of timer ticks.
    const double value_after_ticks = track.getObstacle().center_x;
    const double rate_after_ticks = track.getObstacle().velocity_x;

    std::printf("      after 1.0 s of timer ticks: x = %s m (a true coast would give ~1.0 m),"
                " vx = %s -> %s m/s\n",
                Number(value_after_ticks).c_str(), Number(rate_after_correct).c_str(),
                Number(rate_after_ticks).c_str());
    Check(std::abs(value_after_ticks) < 0.5,
          "D1 confirmed: upstream's timer does NOT coast a full second of motion",
          "x=" + Number(value_after_ticks));
    Check(std::abs(rate_after_ticks) < std::abs(rate_after_correct),
          "D1 confirmed: the velocity estimate decays under the stale-measurement re-application",
          Number(rate_after_correct) + " -> " + Number(rate_after_ticks));
  }

  // -------------------------------------------------------------------------------------
  Section("D2: upstream never initialises P");
  // -------------------------------------------------------------------------------------
  {
    upstream_tracker_reference::CircleObstacle seed;
    seed.center_x = 3.0;
    seed.radius = 0.25;
    upstream_tracker_reference::TrackedObstacle track(seed, 1.0 / 100.0, 200, 0.01, 0.10, 1.00);
    const double p00 = track.getKFx().P(0, 0);
    const double p11 = track.getKFx().P(1, 1);
    std::printf("      a newborn upstream track reports P(0,0) = %s m^2, P(1,1) = %s (m/s)^2\n",
                Number(p00).c_str(), Number(p11).c_str());
    Check(p00 == 1.0 && p11 == 1.0,
          "D2 confirmed: initKF leaves P at the constructor's identity",
          "P=diag(" + Number(p00) + ", " + Number(p11) + ")");
  }

  // -------------------------------------------------------------------------------------
  Section("D3: the association penalty is computed and discarded");
  // -------------------------------------------------------------------------------------
  // This is what licenses the port's cost function to be a plain weighted Euclidean distance: the
  // effective upstream cost already is one, and reproducing the penalty would reproduce dead code.
  {
    upstream_tracker_reference::CircleObstacle old_obstacle;
    old_obstacle.center_x = 2.0;
    old_obstacle.center_y = 0.5;
    old_obstacle.velocity_x = 0.6;
    old_obstacle.velocity_y = 0.4;
    old_obstacle.radius = 0.30;

    upstream_tracker_reference::CircleObstacle new_obstacle = old_obstacle;
    new_obstacle.center_x = 2.09;
    new_obstacle.center_y = 0.56;
    new_obstacle.radius = 0.16;  // A visibility-driven radius step, P8's finding 2.

    const auto result =
        upstream_tracker_reference::obstacleCostFunction(new_obstacle, old_obstacle, 0.15, 10.0);
    const double dx = new_obstacle.center_x - old_obstacle.center_x;
    const double dy = new_obstacle.center_y - old_obstacle.center_y;
    const double dr = new_obstacle.radius - old_obstacle.radius;
    const double plain = std::sqrt(dx * dx + dy * dy + dr * dr);

    std::printf("      returned %s, plain Euclidean %s, discarded penalty %s\n",
                Number(result.returned, 6).c_str(), Number(plain, 6).c_str(),
                Number(result.penalty, 6).c_str());
    Check(std::abs(result.returned - plain) < kTolerance,
          "D3 confirmed: the returned cost is the unweighted Euclidean distance over (dx, dy, dr)",
          Number(std::abs(result.returned - plain)));
    Check(std::abs(result.penalty - 1.0) > 1e-9,
          "the penalty was genuinely computed to something other than 1, and still ignored",
          "penalty=" + Number(result.penalty, 6));

    // The port's cost at weight 1.0 must equal upstream's returned cost exactly - that is what
    // makes association_radius_weight a knob on a faithful port rather than a different rule.
    const double port_cost_w1 = std::sqrt(dx * dx + dy * dy + (1.0 * dr) * (1.0 * dr));
    Check(std::abs(port_cost_w1 - result.returned) < kTolerance,
          "the port's cost at association_radius_weight = 1.0 is upstream's cost");
    // And the shipped weight of 0.25 must reduce it, which is the finding-2 response in one line.
    const double port_cost_shipped = std::sqrt(dx * dx + dy * dy + (0.25 * dr) * (0.25 * dr));
    std::printf("      the same pair costs %s at w=1.0 and %s at w=0.25 (gate 0.30 m)\n",
                Number(port_cost_w1, 6).c_str(), Number(port_cost_shipped, 6).c_str());
    Check(port_cost_shipped < port_cost_w1,
          "the shipped weight reduces the cost of a visibility-driven radius step");
  }

  return Report("perception_tracking_oracle_test");
}
