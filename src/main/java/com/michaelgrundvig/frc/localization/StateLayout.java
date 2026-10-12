package com.michaelgrundvig.frc.localization;

/**
 * How the {@link PoseFilter}'s state and its square matrices are laid out in their arrays. The
 * filter and the working of its camera measurements ({@link CameraCorrection}) share these names,
 * so that {@code covariance[cell(X, HEADING)]} reads as what it is, the covariance of x with the
 * heading.
 *
 * <p>A camera measurement's own matrices are smaller, and laid out by {@link MeasurementLayout}.
 */
final class StateLayout {
  private StateLayout() {}

  // The state's five values, in the order every row and column follows: where the robot is, the
  // heading offset, and the gyro's two errors, its scale and its drift. A camera measurement's
  // rows follow the first three: its x, its y, and its heading if it has one. No measurement has
  // a row for either of the gyro's errors.
  static final int X = 0;
  static final int Y = 1;
  static final int HEADING = 2;
  static final int GYRO_SCALE = 3;
  static final int GYRO_DRIFT = 4;
  static final int STATES = 5;

  /** A measurement of position alone has two rows; one with a heading of its own has three. */
  static final int POSITION_ROWS = 2;

  static final int POSE_ROWS = 3;

  // The cells of the pose's own block of the 5 by 5 covariance, stored row by row, named by the
  // row's and the column's value: XH is the covariance of x with the heading.
  static final int XX = 0;
  static final int XY = 1;
  static final int XH = 2;
  static final int YX = 5;
  static final int YY = 6;
  static final int YH = 7;
  static final int HX = 10;
  static final int HY = 11;
  static final int HH = 12;
  static final int CELLS = STATES * STATES;

  /**
   * Where a cell sits in an array with one row or column for each value of the state, stored row by
   * row with {@link #STATES} to a row. The state's own covariance is one, and so is any matrix
   * between the state and a measurement's rows, whichever of the two its rows are: a measurement
   * has no more rows than the state has values.
   */
  static int cell(int row, int column) {
    return row * STATES + column;
  }
}
