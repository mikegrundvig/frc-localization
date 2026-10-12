package com.michaelgrundvig.frc.localization;

/**
 * The few small-matrix operations localization needs. A matrix here is an array of its numbers row
 * by row, so the number in row {@code r} and column {@code c} of a 3x3 matrix is {@code m[3 * r +
 * c]}. Working on plain arrays means the estimator's maths for every sample makes nothing.
 */
final class Matrices {
  /**
   * A determinant this small beside the matrix's own size counts as zero. The size is taken as the
   * sum of the diagonal's magnitudes, cubed, because a 3x3 determinant is a sum of products of
   * three of the matrix's numbers.
   */
  private static final double SINGULAR_BELOW = 1e-12;

  private Matrices() {}

  /**
   * Inverts a 3x3 matrix into {@code out}, by its cofactors:
   *
   * <pre>inverse = transpose of the cofactors / determinant</pre>
   *
   * <p>The cofactor of a number is the determinant of the 2x2 matrix left when its row and column
   * are struck out, with a sign that alternates across the matrix. The determinant is the first row
   * times its cofactors.
   *
   * @return false, leaving {@code out} unchanged, if the matrix is singular
   */
  static boolean invert3(double[] m, double[] out) {
    double cofactor00 = m[4] * m[8] - m[5] * m[7];
    double cofactor01 = m[5] * m[6] - m[3] * m[8];
    double cofactor02 = m[3] * m[7] - m[4] * m[6];
    double determinant = m[0] * cofactor00 + m[1] * cofactor01 + m[2] * cofactor02;
    double size = Math.abs(m[0]) + Math.abs(m[4]) + Math.abs(m[8]);
    // Written so that a determinant that isn't a number counts as singular too.
    boolean invertible = Math.abs(determinant) > SINGULAR_BELOW * size * size * size;
    if (!invertible) {
      return false;
    }
    double inverse = 1 / determinant;
    out[0] = cofactor00 * inverse;
    out[1] = (m[2] * m[7] - m[1] * m[8]) * inverse;
    out[2] = (m[1] * m[5] - m[2] * m[4]) * inverse;
    out[3] = cofactor01 * inverse;
    out[4] = (m[0] * m[8] - m[2] * m[6]) * inverse;
    out[5] = (m[2] * m[3] - m[0] * m[5]) * inverse;
    out[6] = cofactor02 * inverse;
    out[7] = (m[1] * m[6] - m[0] * m[7]) * inverse;
    out[8] = (m[0] * m[4] - m[1] * m[3]) * inverse;
    return true;
  }

  /** The number {@code v^T M v}, for a 3x3 matrix {@code M} and a vector {@code v} of three. */
  static double quadratic3(double[] m, double[] v) {
    double sum = 0;
    for (int row = 0; row < 3; row++) {
      for (int column = 0; column < 3; column++) {
        sum += v[row] * m[row * 3 + column] * v[column];
      }
    }
    return sum;
  }
}
