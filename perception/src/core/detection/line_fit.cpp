#include "perception/core/detection/line_fit.h"

#include <cmath>
#include <limits>

namespace perception::core {

double LineFit2D::ConditionNumber() const {
  if (singular_min <= 0.0) return std::numeric_limits<double>::infinity();
  return singular_max / singular_min;
}

void LineFitAccumulator::Reset() {
  r11_ = 0.0;
  r12_ = 0.0;
  r22_ = 0.0;
  qtb1_ = 0.0;
  qtb2_ = 0.0;
  residual_sq_ = 0.0;
  point_count_ = 0;
}

void LineFitAccumulator::Add(double x, double y) {
  ++point_count_;

  // The incoming row, [x y | 1], is rotated into R one column at a time. `std::hypot` rather
  // than sqrt(a*a + b*b): the coordinates here are metres in a 10 m window, so overflow is not
  // the concern - the concern is that hypot is correctly rounded and sqrt-of-sum is not, and
  // this rotation is applied once per point per fit, so the error would accumulate over the
  // cluster rather than cancel.
  double row_y = y;
  double row_b = 1.0;

  {
    const double r = std::hypot(r11_, x);
    if (r > 0.0) {
      const double c = r11_ / r;
      const double s = x / r;
      r11_ = r;

      const double new_r12 = c * r12_ + s * row_y;
      row_y = -s * r12_ + c * row_y;
      r12_ = new_r12;

      const double new_qtb1 = c * qtb1_ + s * row_b;
      row_b = -s * qtb1_ + c * row_b;
      qtb1_ = new_qtb1;
    }
  }

  {
    const double r = std::hypot(r22_, row_y);
    if (r > 0.0) {
      const double c = r22_ / r;
      const double s = row_y / r;
      r22_ = r;

      const double new_qtb2 = c * qtb2_ + s * row_b;
      row_b = -s * qtb2_ + c * row_b;
      qtb2_ = new_qtb2;
    }
  }

  // Whatever is left of the row's right-hand side after both rotations is orthogonal to the
  // column space, and therefore is exactly the contribution of this point to the LS residual.
  residual_sq_ += row_b * row_b;
}

UpperTriangular2x2Svd SvdUpperTriangular2x2(double f, double g, double h) {
  // The standard closed form. Writing M = [[f, g], [0, h]] as the sum of a scaled rotation and
  // a scaled reflection: the rotation part has magnitude q and angle a2, the reflection part
  // magnitude r and angle a1, and the singular values are their sum and the absolute
  // difference. Reference: the 2x2 case of the Golub-Kahan SVD, in the form usually
  // attributed to Blinn ("Consider the lowly 2x2 matrix", IEEE CG&A 1996).
  const double e = 0.5 * (f + h);
  const double d = 0.5 * (f - h);
  const double gg = 0.5 * g;   // (m21 + m12) / 2 with m21 == 0.
  const double hh = -0.5 * g;  // (m21 - m12) / 2 with m21 == 0.

  const double q = std::hypot(e, hh);
  const double r = std::hypot(d, gg);

  UpperTriangular2x2Svd out;
  out.s1 = q + r;
  // SIGNED, deliberately. `q - r` can be negative, and taking the absolute value here would
  // break the factorisation - the sign belongs to one of the two rotations and dropping it
  // silently reflects the second singular direction. The pseudo-inverse formula
  // V * diag(1/s) * U^T is valid for a signed diagonal too, so the caller uses the signed value
  // for the reciprocal and its magnitude for the rank test. See the note on the struct.
  out.s2 = q - r;

  // M decomposes as Q*Rot(a2) + R*Rot(a1)*S1, where S1 = diag(1, -1); the target form
  // Rot(phi)*diag(s1, s2)*Rot(theta)^T expands to Q*Rot(phi - theta) + R*Rot(phi + theta)*S1.
  // Matching the two gives a2 = phi - theta and a1 = phi + theta, hence the half-sum and
  // half-DIFFERENCE below. Getting the sign of theta backwards reconstructs a matrix with the
  // right singular values and the wrong entries, which is exactly what the reconstruction check
  // in the detection test exists to catch.
  const double a1 = std::atan2(gg, d);
  const double a2 = std::atan2(hh, e);
  out.theta = 0.5 * (a1 - a2);
  out.phi = 0.5 * (a1 + a2);
  return out;
}

LineFit2D LineFitAccumulator::Solve() const {
  LineFit2D fit;
  fit.point_count = point_count_;
  fit.algebraic_residual_sq = residual_sq_;
  if (point_count_ == 0) return fit;

  const UpperTriangular2x2Svd svd = SvdUpperTriangular2x2(r11_, r12_, r22_);
  const double magnitude_2 = std::abs(svd.s2);
  fit.singular_max = svd.s1;
  fit.singular_min = magnitude_2;

  // ARMADILLO'S OWN RULE, transcribed rather than approximated: op_pinv_meat.hpp:135,
  //   tol = max(n_rows, n_cols) * s[0] * eps,  keep s[i] >= tol.
  // n_rows is the POINT COUNT, not the 2 of the triangular factor, which is why the
  // accumulator has to carry the count at all.
  const double rows = static_cast<double>(point_count_ > 2 ? point_count_ : 2);
  const double tol = rows * svd.s1 * std::numeric_limits<double>::epsilon();

  const double cos_phi = std::cos(svd.phi);
  const double sin_phi = std::sin(svd.phi);
  const double cos_theta = std::cos(svd.theta);
  const double sin_theta = std::sin(svd.theta);

  // p = V * diag(1/s_i, truncated) * U^T * (Q^T b).
  const double ut_b1 = cos_phi * qtb1_ + sin_phi * qtb2_;
  const double ut_b2 = -sin_phi * qtb1_ + cos_phi * qtb2_;

  double z1 = 0.0;
  double z2 = 0.0;
  if (svd.s1 >= tol && svd.s1 > 0.0) {
    z1 = ut_b1 / svd.s1;
    ++fit.rank;
  }
  if (magnitude_2 >= tol && magnitude_2 > 0.0) {
    z2 = ut_b2 / svd.s2;  // The SIGNED reciprocal; the magnitude only gates the truncation.
    ++fit.rank;
  }

  fit.a = cos_theta * z1 - sin_theta * z2;
  fit.b = sin_theta * z1 + cos_theta * z2;
  return fit;
}

}  // namespace perception::core
