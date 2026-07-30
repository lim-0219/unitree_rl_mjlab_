// LineFitAccumulator - the Armadillo->Eigen substitution, isolated.
//
// PORTED FROM: obstacle_detector/include/obstacle_detector/utilities/figure_fitting.h,
// `fitSegment` (both overloads). Upstream solves
//
//     [x_0 y_0]           [1]
//     [ ...  ] * [A;B] =  [.]      i.e.  A*x + B*y = 1,  with C pinned to -1
//     [x_N y_N]           [1]
//
// via `arma::pinv(input) * output` - the MINIMUM-NORM least-squares solution with SVD rank
// truncation. This header is the substitute, and it is a separate translation unit from the
// detector precisely because the substitution is the one place where "faithful port" is a
// numerical claim rather than a structural one (architecture doc section 10, P8: "pinv vs QR
// may differ on degenerate clusters -> tolerance policy section 11").
//
// WHY NOT Eigen::JacobiSVD / ColPivHouseholderQR DIRECTLY. Two reasons, both binding:
//
//   1. ALLOCATION. The P8 acceptance gate is "zero heap allocation in steady state". Every
//      Eigen decomposition object allocates its own workspace for a dynamic-row matrix, and
//      it reallocates whenever the row count changes - which for this detector is every
//      single cluster. Preallocating one decomposition per possible cluster size is not a
//      design, it is a workaround.
//
//   2. THE DESIGN MATRIX HAS TWO COLUMNS. That is small enough that the textbook streaming
//      form is not a compromise: Givens rotations fold each row into a 2x2 upper-triangular
//      R and a 2-vector Q^T b as the row arrives, so nothing but 5 doubles is ever stored,
//      the N x 2 matrix is never materialised, and the result is a genuine QR - backward
//      stable, with NO squaring of the condition number. Forming the 2x2 normal equations
//      X^T X would have been simpler still and is the obvious thing to reach for; it is
//      rejected here on purpose, because cond(X^T X) = cond(X)^2 turns a merely awkward
//      cluster into a fabricated singularity, which is exactly the failure this module has
//      to be trusted not to invent.
//
// PINV SEMANTICS ARE REPRODUCED, NOT APPROXIMATED. After the streaming QR the problem is a
// 2x2 one, and the 2x2 SVD is closed-form. The rank test then uses Armadillo's OWN default
// rule, transcribed from op_pinv_meat.hpp:135 of the installed version:
//
//     tol = max(n_rows, n_cols) * s[0] * eps          keep s[i] >= tol
//
// with n_rows = the point count and n_cols = 2. Because the singular values of R are exactly
// those of X, this truncates at exactly the same rank upstream does - so on a rank-deficient
// cluster the two do not "differ on degenerate input", they agree by construction. That is
// what makes the <= 1e-9 endpoint target reachable across the whole corpus rather than
// something the degenerate cases have to be excused from.
//
// WHAT DEGENERACY MEANS HERE, GEOMETRICALLY. The model A*x + B*y = 1 cannot represent a line
// through the origin, and the design matrix has no intercept column. So rank(X) < 2 iff every
// point is a scalar multiple of one direction - i.e. the cluster is collinear WITH THE SENSOR,
// not merely collinear. A radially-viewed edge, a duplicated point pair, and a cluster
// collapsed onto one bearing are the three ways to reach it. Upstream's response is the
// min-norm solution, which for rank 1 is the line perpendicular to that bearing at distance
// 1/|p|; this reproduces it.
#ifndef PERCEPTION_CORE_DETECTION_LINE_FIT_H_
#define PERCEPTION_CORE_DETECTION_LINE_FIT_H_

#include <cstdint>

namespace perception::core {

// The fitted line, in upstream's normalisation: a*x + b*y + c = 0 with c fixed at -1, so the
// line is `a*x + b*y = 1`. Kept in that form rather than converted to a unit normal because
// the endpoint projection formula this feeds is transcribed from upstream and is written in
// exactly these coefficients.
struct LineFit2D {
  double a = 0.0;
  double b = 0.0;

  // Upstream's C. A constant, not a fitted parameter - it is pinned to -1 by the choice of
  // right-hand side, which is what makes the fit unable to represent lines through the origin.
  static constexpr double kC = -1.0;

  // Rank AFTER pinv truncation: 2 normally, 1 for a sensor-collinear cluster, 0 only if every
  // point is at the origin. Not a diagnostic afterthought - the detector reports it, because a
  // rank-1 fit is the one case where the emitted segment is a projection artefact rather than
  // a measurement.
  int rank = 0;

  // Singular values of the N x 2 design matrix, largest first. Exposed so the tolerance
  // experiment can characterise the corpus rather than assert about it.
  double singular_max = 0.0;
  double singular_min = 0.0;

  int point_count = 0;

  // Sum of squared LS residuals in the ALGEBRAIC sense (||X p - 1||^2), accumulated by the
  // rotations for free. This is NOT the orthogonal-distance residual the FittedPrimitive2D
  // contract defines; that one is computed by a second pass over the points against the final
  // line, because the two differ by the 1/(a^2+b^2) scaling and the contract is explicit that
  // its residual is in metres.
  double algebraic_residual_sq = 0.0;

  bool IsDegenerate() const { return rank < 2; }

  // Upstream's `D`. Zero exactly when the fit produced no line at all (rank 0), which is the
  // case upstream guards with `if (D > 0.0)` before projecting the endpoints.
  double D() const { return a * a + b * b; }

  // sigma_max / sigma_min, or infinity when the matrix is exactly rank-deficient.
  double ConditionNumber() const;
};

// Streaming Givens QR of the N x 2 system, plus the closed-form 2x2 pinv solve.
//
// Reset() -> Add() per point -> Solve(). Allocation-free by construction: the state is five
// doubles and a count, and Solve() touches nothing else.
class LineFitAccumulator {
 public:
  void Reset();

  // One row of the design matrix, with the implicit right-hand side 1.0.
  void Add(double x, double y);

  LineFit2D Solve() const;

  int point_count() const { return point_count_; }

 private:
  // R (2x2 upper triangular) and Q^T b (its first two entries).
  double r11_ = 0.0;
  double r12_ = 0.0;
  double r22_ = 0.0;
  double qtb1_ = 0.0;
  double qtb2_ = 0.0;

  // The part of Q^T b that fell below the second row - the LS residual norm, squared.
  double residual_sq_ = 0.0;

  int point_count_ = 0;
};

// The closed-form SVD of an upper-triangular 2x2 [[f, g], [0, h]], exposed for its own unit
// test. Returns singular values (s1 >= s2 >= 0) and the two rotation angles such that
// M = U(phi) * diag(s1, s2) * V(theta)^T, where the rotation matrices are
// [[cos, -sin], [sin, cos]].
struct UpperTriangular2x2Svd {
  double s1 = 0.0;  // Always >= 0.

  // SIGNED, and this is not an oversight. The closed form produces `q - r`, which can be
  // negative; the sign belongs to the factorisation and taking the absolute value here would
  // reflect the second singular direction without telling anyone. `V * diag(1/s1, 1/s2) * U^T`
  // is the pseudo-inverse for a signed diagonal too, so callers use the signed value for the
  // reciprocal and |s2| for the rank comparison.
  double s2 = 0.0;

  double phi = 0.0;    // Left rotation (U).
  double theta = 0.0;  // Right rotation (V).
};
UpperTriangular2x2Svd SvdUpperTriangular2x2(double f, double g, double h);

}  // namespace perception::core

#endif  // PERCEPTION_CORE_DETECTION_LINE_FIT_H_
