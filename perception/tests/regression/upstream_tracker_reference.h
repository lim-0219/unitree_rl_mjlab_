// The upstream obstacle_tracker filtering and association arithmetic, as a regression oracle.
// TEST-ONLY. Nothing here is ever linked into production.
//
// SPLIT OF PROVENANCE - which half is upstream's own code and which is transcribed, because the
// two carry different weight and mixing them up would overstate what this file proves:
//
//   * The Kalman filter itself is UPSTREAM'S OWN HEADER, compiled and linked:
//     obstacle_detector/utilities/kalman.h with Armadillo. The audit established that header is
//     ROS-free, so the 1e-12 comparison in tracking_oracle_test.cpp runs against the real
//     `arma::mat` arithmetic rather than against a re-typing of it. There is NO numerical
//     substitution to justify here - unlike P8's pinv-vs-QR swap, this is meant to be the same
//     linear algebra, so the tolerance is a floating-point-association tolerance and nothing more.
//
//   * `TrackedObstacle` and `ObstacleTracker::obstacleCostFunction` are TRANSCRIBED, because
//     both reach ROS: tracked_obstacle.h includes obstacle_detector/Obstacles.h (a generated ROS1
//     message) and math_utilities.h includes geometry_msgs and tf. The transcriptions below keep
//     the arithmetic and the ORDER of operations, and replace only the message types with plain
//     structs.
//
// EVERYTHING IS TRANSCRIBED INCLUDING ITS DEFECTS, on the same principle as P8's
// upstream_extractor_reference.h: an oracle that quietly fixed what it was oracling would certify
// the port against something that never shipped. Three in particular are preserved and are
// asserted by the oracle test, because each is a fact about upstream that the port had to make a
// decision about:
//
//   D1  `TrackedObstacle::updateState()` (tracked_obstacle.h:87-105) calls predictState() and
//       then correctState() - and KalmanFilter::correctState() reads `y`, which is written ONLY by
//       TrackedObstacle::correctState(const CircleObstacle&). So the timer-driven "coast" does not
//       propagate the constant-velocity model at all: it re-applies the LAST RECEIVED MEASUREMENT
//       at full Kalman gain, roughly ten times per scan. This is the substantive reason the timer
//       model could not be ported, over and above the determinism reason in risk R11.
//
//   D2  `initKF()` (tracked_obstacle.h:128-161) never touches P, so every upstream track begins
//       with P = eye from the KalmanFilter constructor (kalman.h:53): 1.0 m^2 of claimed
//       positional variance about a position it has just measured.
//
//   D3  `obstacleCostFunction` (obstacle_tracker.cpp:238-265) computes an elaborate
//       direction-rotated penalty and then DISCARDS it - `return cost / 1.0;`, with
//       `// return cost / penalty;` commented out above a TODO. The effective cost is therefore a
//       plain Euclidean distance over (dx, dy, dr). The dead code also has a bug worth recording
//       so that nobody revives it as-is: `distribution` is built with VARIANCES on its diagonal
//       and then used as though it were an information matrix, in `-0.5 * r^T * distribution * r`,
//       so a more uncertain track would produce a SMALLER penalty and thus a larger cost.
//
// LICENSE: BSD-3, Poznan University of Technology, 2017, Mateusz Przybyla. See
// perception/third_party_notices.md.
#ifndef PERCEPTION_TESTS_REGRESSION_UPSTREAM_TRACKER_REFERENCE_H_
#define PERCEPTION_TESTS_REGRESSION_UPSTREAM_TRACKER_REFERENCE_H_

#include <cmath>
#include <vector>

#include <armadillo>

#include "obstacle_detector/utilities/kalman.h"

namespace upstream_tracker_reference {

// obstacle_detector::CircleObstacle, minus the ROS message machinery. Upstream's field names.
struct CircleObstacle {
  double center_x = 0.0;
  double center_y = 0.0;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double radius = 0.0;
  double true_radius = 0.0;
};

// tracked_obstacle.h, transcribed. The three KalmanFilter members are upstream's real class.
//
// The static parameter block is reproduced as plain members rather than as statics: upstream's
// `static double s_sampling_time_` and friends (tracked_obstacle.h:173-177) mean a second tracker
// in the same process silently uses the first one's numbers, which is a property of upstream's
// lifetime model and not of its algorithm - the same reason P8's reference de-`static`ed
// `groupPoints`'s `sin_dp`.
class TrackedObstacle {
 public:
  TrackedObstacle(const CircleObstacle& obstacle, double sampling_time, int fade_counter_size,
                  double process_variance, double process_rate_variance,
                  double measurement_variance)
      : obstacle_(obstacle),
        kf_x_(0, 1, 2),
        kf_y_(0, 1, 2),
        kf_r_(0, 1, 2),
        sampling_time_(sampling_time),
        process_variance_(process_variance),
        process_rate_variance_(process_rate_variance),
        measurement_variance_(measurement_variance) {
    fade_counter_ = fade_counter_size;
    initKF();
  }

  void predictState() {
    kf_x_.predictState();
    kf_y_.predictState();
    kf_r_.predictState();

    obstacle_.center_x = kf_x_.q_pred(0);
    obstacle_.center_y = kf_y_.q_pred(0);

    obstacle_.velocity_x = kf_x_.q_pred(1);
    obstacle_.velocity_y = kf_y_.q_pred(1);

    obstacle_.radius = kf_r_.q_pred(0);

    fade_counter_--;
  }

  void correctState(const CircleObstacle& new_obstacle) {
    kf_x_.y(0) = new_obstacle.center_x;
    kf_y_.y(0) = new_obstacle.center_y;
    kf_r_.y(0) = new_obstacle.radius;

    kf_x_.correctState();
    kf_y_.correctState();
    kf_r_.correctState();

    obstacle_.center_x = kf_x_.q_est(0);
    obstacle_.center_y = kf_y_.q_est(0);

    obstacle_.velocity_x = kf_x_.q_est(1);
    obstacle_.velocity_y = kf_y_.q_est(1);

    obstacle_.radius = kf_r_.q_est(0);

    fade_counter_ = fade_counter_size_;
  }

  // D1 lives here. `correctState()` on the filters re-uses whatever is already in `y`.
  void updateState() {
    kf_x_.predictState();
    kf_y_.predictState();
    kf_r_.predictState();

    kf_x_.correctState();
    kf_y_.correctState();
    kf_r_.correctState();

    obstacle_.center_x = kf_x_.q_est(0);
    obstacle_.center_y = kf_y_.q_est(0);

    obstacle_.velocity_x = kf_x_.q_est(1);
    obstacle_.velocity_y = kf_y_.q_est(1);

    obstacle_.radius = kf_r_.q_est(0);

    fade_counter_--;
  }

  bool hasFaded() const { return ((fade_counter_ <= 0) ? true : false); }
  const CircleObstacle& getObstacle() const { return obstacle_; }
  const KalmanFilter& getKFx() const { return kf_x_; }
  const KalmanFilter& getKFy() const { return kf_y_; }
  const KalmanFilter& getKFr() const { return kf_r_; }
  KalmanFilter& mutableKFx() { return kf_x_; }

 private:
  void initKF() {
    kf_x_.A(0, 1) = sampling_time_;
    kf_y_.A(0, 1) = sampling_time_;
    kf_r_.A(0, 1) = sampling_time_;

    kf_x_.C(0, 0) = 1.0;
    kf_y_.C(0, 0) = 1.0;
    kf_r_.C(0, 0) = 1.0;

    kf_x_.R(0, 0) = measurement_variance_;
    kf_y_.R(0, 0) = measurement_variance_;
    kf_r_.R(0, 0) = measurement_variance_;

    kf_x_.Q(0, 0) = process_variance_;
    kf_r_.Q(0, 0) = process_variance_;
    kf_y_.Q(0, 0) = process_variance_;

    kf_x_.Q(1, 1) = process_rate_variance_;
    kf_y_.Q(1, 1) = process_rate_variance_;
    kf_r_.Q(1, 1) = process_rate_variance_;

    kf_x_.q_pred(0) = obstacle_.center_x;
    kf_r_.q_pred(0) = obstacle_.radius;
    kf_y_.q_pred(0) = obstacle_.center_y;

    kf_x_.q_pred(1) = obstacle_.velocity_x;
    kf_y_.q_pred(1) = obstacle_.velocity_y;

    kf_x_.q_est(0) = obstacle_.center_x;
    kf_r_.q_est(0) = obstacle_.radius;
    kf_y_.q_est(0) = obstacle_.center_y;

    kf_x_.q_est(1) = obstacle_.velocity_x;
    kf_y_.q_est(1) = obstacle_.velocity_y;
    // D2: P is never touched. It keeps the identity the KalmanFilter constructor gave it.
  }

  CircleObstacle obstacle_;

  KalmanFilter kf_x_;
  KalmanFilter kf_y_;
  KalmanFilter kf_r_;

  int fade_counter_ = 0;
  int fade_counter_size_ = 0;

  double sampling_time_ = 0.0;
  double process_variance_ = 0.0;
  double process_rate_variance_ = 0.0;
  double measurement_variance_ = 0.0;
};

// A single upstream KalmanFilter wired exactly as initKF wires one axis, but with the transition
// step, the process covariance and the measurement covariance supplied per call. This is what the
// 1e-12 comparison drives: it isolates the linear algebra from upstream's timing model, so the
// comparison is about the arithmetic and not about the two designs' different notions of dt.
class AxisOracle {
 public:
  AxisOracle(double value, double rate, double p00, double p11) : kf_(0, 1, 2) {
    kf_.C(0, 0) = 1.0;
    kf_.q_est(0) = value;
    kf_.q_est(1) = rate;
    kf_.q_pred(0) = value;
    kf_.q_pred(1) = rate;
    // Set explicitly rather than left at the constructor's identity, so that the port's
    // divergence D2 is outside this comparison by construction.
    kf_.P(0, 0) = p00;
    kf_.P(0, 1) = 0.0;
    kf_.P(1, 0) = 0.0;
    kf_.P(1, 1) = p11;
  }

  void Predict(double dt, double q_value, double q_rate) {
    kf_.A(0, 1) = dt;
    kf_.Q(0, 0) = q_value;
    kf_.Q(1, 1) = q_rate;
    kf_.predictState();
    // Upstream's correctState() consumes q_pred, so a predict that is not followed by a correct
    // has to be published into q_est by hand for the next predict to build on it. This mirrors
    // exactly what upstream's timer does when it predicts repeatedly.
    kf_.q_est = kf_.q_pred;
  }

  void PredictOnly(double dt, double q_value, double q_rate) {
    kf_.A(0, 1) = dt;
    kf_.Q(0, 0) = q_value;
    kf_.Q(1, 1) = q_rate;
    kf_.predictState();
  }

  void Correct(double measurement, double measurement_variance) {
    kf_.R(0, 0) = measurement_variance;
    kf_.y(0) = measurement;
    kf_.correctState();
  }

  double value() const { return kf_.q_est(0); }
  double rate() const { return kf_.q_est(1); }
  double p00() const { return kf_.P(0, 0); }
  double p01() const { return kf_.P(0, 1); }
  double p10() const { return kf_.P(1, 0); }
  double p11() const { return kf_.P(1, 1); }

 private:
  KalmanFilter kf_;
};

// obstacle_tracker.cpp:238-265, transcribed. `transformPoint`, `length` and `squaredLength` come
// from math_utilities.h, which includes geometry_msgs and tf, so their arithmetic is inlined here:
// transformPoint(p, 0, 0, -theta) is a rotation of p by -theta about the origin.
//
// D3: the penalty is computed in full and then discarded, exactly as upstream discards it.
struct CostResult {
  double returned = 0.0;      // What upstream actually returns.
  double euclidean = 0.0;     // The `cost` variable.
  double penalty = 0.0;       // Computed, then unused.
};

inline CostResult obstacleCostFunction(const CircleObstacle& new_obstacle,
                                      const CircleObstacle& old_obstacle,
                                      double std_correspondence_dev, double sensor_rate) {
  arma::mat distribution = arma::mat(2, 2).zeros();
  arma::vec relative_position = arma::vec(2).zeros();

  double cost = 0.0;
  double penalty = 1.0;
  const double tp = 1.0 / sensor_rate;

  const double direction = std::atan2(old_obstacle.velocity_y, old_obstacle.velocity_x);

  // transformPoint(point, 0.0, 0.0, -direction): rotate by -direction.
  const double cos_d = std::cos(-direction);
  const double sin_d = std::sin(-direction);
  const double new_center_x = new_obstacle.center_x * cos_d - new_obstacle.center_y * sin_d;
  const double new_center_y = new_obstacle.center_x * sin_d + new_obstacle.center_y * cos_d;
  const double old_center_x = old_obstacle.center_x * cos_d - old_obstacle.center_y * sin_d;
  const double old_center_y = old_obstacle.center_x * sin_d + old_obstacle.center_y * cos_d;

  const double old_speed_squared = old_obstacle.velocity_x * old_obstacle.velocity_x +
                                   old_obstacle.velocity_y * old_obstacle.velocity_y;
  const double old_speed = std::sqrt(old_speed_squared);

  distribution(0, 0) = std::pow(std_correspondence_dev, 2.0) + old_speed_squared * std::pow(tp, 2.0);
  distribution(1, 1) = std::pow(std_correspondence_dev, 2.0);

  relative_position(0) = new_center_x - old_center_x - tp * old_speed;
  relative_position(1) = new_center_y - old_center_y;

  cost = std::sqrt(std::pow(new_obstacle.center_x - old_obstacle.center_x, 2.0) +
                   std::pow(new_obstacle.center_y - old_obstacle.center_y, 2.0) +
                   std::pow(new_obstacle.radius - old_obstacle.radius, 2.0));

  const arma::mat a = -0.5 * arma::trans(relative_position) * distribution * relative_position;
  penalty = std::exp(a(0, 0));

  CostResult result;
  result.euclidean = cost;
  result.penalty = penalty;
  // TODO: Check values for cost/penalty in common situations
  // return cost / penalty;
  result.returned = cost / 1.0;
  return result;
}

}  // namespace upstream_tracker_reference

#endif  // PERCEPTION_TESTS_REGRESSION_UPSTREAM_TRACKER_REFERENCE_H_
