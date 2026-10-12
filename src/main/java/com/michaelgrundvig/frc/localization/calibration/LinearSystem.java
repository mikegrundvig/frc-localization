package com.michaelgrundvig.frc.localization.calibration;

import java.util.Optional;

/** Solves the small square systems of linear equations the calibration's least squares needs. */
final class LinearSystem {
  /** A pivot smaller than this is zero: the matrix is singular, and the system has no solution. */
  private static final double SMALLEST_PIVOT = 1e-300;

  private LinearSystem() {}

  /**
   * Solves {@code matrix x = right} for x, by Gaussian elimination with partial pivoting: each
   * column in turn is cleared below the diagonal, using the row with the largest value in that
   * column as the pivot, and the unknowns are then found from the last row up.
   *
   * @param matrix a square matrix, as rows; it is not changed
   * @param right the right-hand side, one value for each row; it is not changed
   * @return the solution, or empty if the matrix is singular
   */
  static Optional<double[]> solve(double[][] matrix, double[] right) {
    int size = right.length;
    double[][] a = new double[size][];
    double[] b = right.clone();
    for (int row = 0; row < size; row++) {
      a[row] = matrix[row].clone();
    }
    for (int column = 0; column < size; column++) {
      int pivot = column;
      for (int row = column + 1; row < size; row++) {
        if (Math.abs(a[row][column]) > Math.abs(a[pivot][column])) {
          pivot = row;
        }
      }
      if (Math.abs(a[pivot][column]) < SMALLEST_PIVOT) {
        return Optional.empty();
      }
      double[] pivotRow = a[pivot];
      a[pivot] = a[column];
      a[column] = pivotRow;
      double pivotRight = b[pivot];
      b[pivot] = b[column];
      b[column] = pivotRight;
      for (int row = column + 1; row < size; row++) {
        double factor = a[row][column] / a[column][column];
        for (int k = column; k < size; k++) {
          a[row][k] -= factor * a[column][k];
        }
        b[row] -= factor * b[column];
      }
    }
    double[] solution = new double[size];
    for (int row = size - 1; row >= 0; row--) {
      double sum = b[row];
      for (int k = row + 1; k < size; k++) {
        sum -= a[row][k] * solution[k];
      }
      solution[row] = sum / a[row][row];
    }
    return Optional.of(solution);
  }
}
