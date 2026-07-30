// AxisKalman2 - one axis of the ported obstacle_detector tracker.
//
// A 2-state [value, rate] Kalman filter with a constant-rate transition and a value-only
// measurement. Three of these per obstacle (x, y, radius), exactly as upstream
// (tracked_obstacle.h:46 `kf_x_(0, 1, 2), kf_y_(0, 1, 2), kf_r_(0, 1, 2)` - zero inputs, one
// output, two states).
//
// =========================================================================================
// WHY THIS IS 2x2 SCALAR ARITHMETIC RATHER THAN Eigen::Matrix2d
// =========================================================================================
// The regression gate for this phase is exact agreement with upstream's Armadillo KalmanFilter
// to 1e-12 on identical (dt, measurement) sequences. Upstream's update is four matrix
// expressions (kalman.h:64-73):
//
//     q_pred = A * q_est + B * u        (B, u are zero here - dim_in is 0)
//     P      = A * P * trans(A) + Q
//     K      = P * trans(C) * inv(C * P * trans(C) + R)
//     q_est  = q_pred + K * (y - C * q_pred)
//     P      = (I - K * C) * P
//
// Each is expanded below in the SAME OPERATION ORDER, so the two implementations differ only in
// floating-point association, not in algorithm. Writing it as Eigen expressions would invite
// Eigen to reassociate or vectorise, and then a 1e-12 disagreement would be a fact about two
// linear-algebra libraries rather than about this port. Unlike P8's pinv-vs-QR substitution
// there is NO numerical substitution here to justify a tolerance policy: the arithmetic is
// meant to be the same arithmetic.
//
// =========================================================================================
// THE FULL, UNSYMMETRISED P
// =========================================================================================
// All four entries of P are stored and propagated, and `P = (I - K*C) * P` is applied verbatim.
// That form is not symmetry-preserving - the standard objection to it - and upstream does not
// symmetrise. Neither does this. Storing only the upper triangle, or averaging P with its
// transpose, would each be a silent divergence that the 1e-12 gate would then have to be
// loosened to accommodate. TrackState2D exposes only the diagonal (p00 and p11), which is what
// makes the asymmetry invisible downstream while remaining exactly reproducible here.
//
// =========================================================================================
// Q IS PASSED PER STEP, NOT STORED
// =========================================================================================
// Upstream stores Q once (tracked_obstacle.h:141-147) because its dt never changes - it is
// always `1 / loop_rate`. With measurement-driven dt, a stored Q would mean "this much process
// noise per call regardless of how much time passed", which makes the filter's behaviour depend
// on the scan rate. Predict() therefore takes the ABSOLUTE variances for the step it is about
// to take, and KfCircleTracker is what scales them by dt. Keeping the scaling out of here is
// also what lets the regression oracle drive this filter with upstream's own fixed Q.
#ifndef PERCEPTION_CORE_TRACKING_AXIS_KALMAN_H_
#define PERCEPTION_CORE_TRACKING_AXIS_KALMAN_H_

namespace perception::core {

// The result of one scalar correction, reported rather than discarded: the innovation and its
// predicted variance are the two numbers a NIS consistency check needs, and recomputing them
// outside the filter would mean recomputing S from a P that the correction has already
// overwritten.
struct KalmanCorrection {
  double innovation = 0.0;            // y - value_pred.
  double innovation_variance = 0.0;   // S = P(0,0) + R, the PREDICTED variance of `innovation`.
  double gain_value = 0.0;            // K(0).
  double gain_rate = 0.0;             // K(1).

  // NIS for this scalar channel. Expectation 1.0 when R and Q are correctly scaled.
  double NormalizedInnovationSquared() const {
    return innovation_variance > 0.0 ? (innovation * innovation) / innovation_variance : 0.0;
  }
};

class AxisKalman2 {
 public:
  AxisKalman2() = default;

  // Birth from a first measurement. `value_variance` is normally the measurement variance that
  // produced `value` and `rate_variance` a prior on the unknown rate.
  //
  // DIVERGENCE FROM UPSTREAM, DELIBERATE. Upstream never initialises P at all: KalmanFilter's
  // constructor sets `P = eye(n,n)` (kalman.h:53) and initKF (tracked_obstacle.h:128-161)
  // overwrites A, C, R, Q and both state vectors but leaves P alone. Every upstream track
  // therefore begins claiming 1.0 m^2 of positional variance about a position it has just
  // measured to centimetres, and 1.0 (m/s)^2 about a velocity it has not measured at all - one
  // number far too large, the other arbitrary. Seeding P from the actual measurement variance
  // is what makes the first few updates of a track meaningful instead of a transient, and it
  // is what makes the covariance-weighted fusion merge (which divides by P) weight tracks by
  // something real. The regression oracle sets both filters' initial state and P explicitly, so
  // this divergence is outside the 1e-12 comparison by construction rather than by tolerance.
  void Initialize(double value, double rate, double value_variance, double rate_variance) {
    value_ = value;
    rate_ = rate;
    p00_ = value_variance;
    p01_ = 0.0;
    p10_ = 0.0;
    p11_ = rate_variance;
  }

  // q_pred = A * q_est  and  P = A * P * trans(A) + Q,  with A = [[1, dt], [0, 1]].
  //
  // The B * u term is omitted rather than computed: upstream constructs these filters with
  // dim_in = 0, so B is 2x0 and u is empty, and `B * u` is the zero vector for every call.
  void Predict(double dt, double q_value, double q_rate) {
    // q_pred = A * q_est.
    value_ = value_ + dt * rate_;
    // rate_ is unchanged: the second row of A is [0, 1].

    // A * P, row by row. Row 1 of A is [0, 1], so the second row of the product is P's own.
    const double ap00 = p00_ + dt * p10_;
    const double ap01 = p01_ + dt * p11_;
    const double ap10 = p10_;
    const double ap11 = p11_;

    // (A * P) * trans(A), with trans(A) = [[1, 0], [dt, 1]]. Then + Q.
    p00_ = ap00 + dt * ap01 + q_value;
    p01_ = ap01;
    p10_ = ap10 + dt * ap11;
    p11_ = ap11 + q_rate;
  }

  // K = P * trans(C) * inv(C * P * trans(C) + R), then the state and P updates, with
  // C = [1, 0] so that C * P * trans(C) is the 1x1 matrix [P(0,0)].
  //
  // Returns the innovation and its predicted variance, taken BEFORE P is overwritten.
  KalmanCorrection Correct(double measurement, double measurement_variance) {
    KalmanCorrection result;
    // S = C * P * trans(C) + R. A 1x1 matrix, so inv() is a reciprocal.
    const double s = p00_ + measurement_variance;
    result.innovation_variance = s;

    // A non-positive S can only come from a non-positive R paired with a collapsed P, both of
    // which the config validation and Initialize() forbid. Guarding rather than dividing keeps
    // one broken axis from turning the whole track into NaN, which TrackState2D::Validate()
    // would then reject wholesale and give no clue about.
    if (!(s > 0.0)) {
      result.innovation = measurement - value_;
      result.innovation_variance = 0.0;
      return result;
    }

    const double s_inv = 1.0 / s;
    const double k0 = p00_ * s_inv;
    const double k1 = p10_ * s_inv;
    result.gain_value = k0;
    result.gain_rate = k1;

    // q_est = q_pred + K * (y - C * q_pred).
    const double innovation = measurement - value_;
    result.innovation = innovation;
    value_ = value_ + k0 * innovation;
    rate_ = rate_ + k1 * innovation;

    // P = (I - K * C) * P, with (I - K*C) = [[1 - k0, 0], [-k1, 1]]. The zero entry is written
    // out as a dropped term rather than a multiplication by 0.0 - the product is identical, and
    // multiplying by an exact zero cannot change a finite result.
    const double new_p00 = (1.0 - k0) * p00_;
    const double new_p01 = (1.0 - k0) * p01_;
    const double new_p10 = p10_ - k1 * p00_;
    const double new_p11 = p11_ - k1 * p01_;
    p00_ = new_p00;
    p01_ = new_p01;
    p10_ = new_p10;
    p11_ = new_p11;

    return result;
  }

  double value() const { return value_; }
  double rate() const { return rate_; }
  double value_variance() const { return p00_; }
  double rate_variance() const { return p11_; }

  // For the fusion merge, which needs to overwrite a merged mean onto a merged covariance.
  void SetState(double value, double rate) {
    value_ = value;
    rate_ = rate;
  }
  void SetCovariance(double p00, double p01, double p10, double p11) {
    p00_ = p00;
    p01_ = p01;
    p10_ = p10;
    p11_ = p11;
  }
  double p01() const { return p01_; }
  double p10() const { return p10_; }

 private:
  double value_ = 0.0;
  double rate_ = 0.0;

  // P, all four entries. See the header comment on why the asymmetry is preserved.
  double p00_ = 1.0;
  double p01_ = 0.0;
  double p10_ = 0.0;
  double p11_ = 1.0;
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_TRACKING_AXIS_KALMAN_H_
