package com.michaelgrundvig.frc.localization;

import java.util.Arrays;

/**
 * The errors one camera measurement shares with other measurements: its camera's turn and shifts,
 * which are in every image that camera takes, and its tags' placements, which are in every image
 * any camera takes of them.
 *
 * <p>For each error it holds three things: its column among the errors the filter considers ({@link
 * ConsideredErrors#NO_COLUMN} for one the filter keeps no column for, which is then taken as a new,
 * unrelated error in each image), its variance, and how much of it each row of the measurement
 * sees. That last is the measurement's Jacobian with respect to the error: an error of one unit
 * moves the measured x, the measured y and the measured heading by those amounts.
 *
 * <p>A {@link PoseFilter} owns one of these and refills it for each measurement, so that nothing is
 * allocated as measurements are applied. It grows only if a measurement names more tags than any
 * before it.
 */
final class SharedErrors {
  /** Room for a camera's four errors and eight tags' two each, before it has to grow. */
  private static final int INITIAL_CAPACITY = 20;

  private static final int MEASUREMENT_ROWS = 3;

  private int count;
  private int[] column = new int[INITIAL_CAPACITY];
  private double[] variance = new double[INITIAL_CAPACITY];

  /** For each row of the measurement (x, y, heading), how much of each error it sees. */
  private final double[][] seenByRow = new double[MEASUREMENT_ROWS][INITIAL_CAPACITY];

  void clear() {
    count = 0;
  }

  /**
   * Adds an error the measurement shares. One with no variance is left out: it is not an error at
   * all, and nothing can depend on it.
   *
   * @param column its column among the considered errors, or {@link ConsideredErrors#NO_COLUMN}
   * @param seenByX how far one unit of it moves the measured x
   * @param seenByY and the measured y
   * @param seenByHeading and the measured heading
   */
  void add(int column, double variance, double seenByX, double seenByY, double seenByHeading) {
    boolean isAnError = variance > 0;
    if (!isAnError) {
      return;
    }
    if (count == this.column.length) {
      grow();
    }
    this.column[count] = column;
    this.variance[count] = variance;
    seenByRow[StateLayout.X][count] = seenByX;
    seenByRow[StateLayout.Y][count] = seenByY;
    seenByRow[StateLayout.HEADING][count] = seenByHeading;
    count++;
  }

  int count() {
    return count;
  }

  int column(int error) {
    return column[error];
  }

  double variance(int error) {
    return variance[error];
  }

  /** How much of an error a row of the measurement sees. */
  double seenBy(int row, int error) {
    return seenByRow[row][error];
  }

  private void grow() {
    int capacity = 2 * column.length;
    column = Arrays.copyOf(column, capacity);
    variance = Arrays.copyOf(variance, capacity);
    for (int row = 0; row < MEASUREMENT_ROWS; row++) {
      seenByRow[row] = Arrays.copyOf(seenByRow[row], capacity);
    }
  }
}
