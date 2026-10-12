package com.michaelgrundvig.frc.localization;

/**
 * How the {@link PoseFilter}'s state and its 3 by 3 matrices are laid out in their arrays. The
 * filter and the working of its camera measurements ({@link CameraCorrection}) share these names,
 * so that {@code covariance[XH]} reads as what it is, the covariance of x with the heading.
 */
final class StateLayout {
  private StateLayout() {}

  // The state's three values, in the order every row and column follows. A camera
  // measurement's rows follow the same order: its x, its y, and its heading if it has one.
  static final int X = 0;
  static final int Y = 1;
  static final int HEADING = 2;
  static final int STATES = 3;

  /** A measurement of position alone has two rows; one with a heading of its own has three. */
  static final int POSITION_ROWS = 2;

  static final int POSE_ROWS = 3;

  // The cells of a 3 by 3 matrix stored row by row, named by the row's and the column's value:
  // XH is the covariance of x with the heading.
  static final int XX = 0;
  static final int XY = 1;
  static final int XH = 2;
  static final int YX = 3;
  static final int YY = 4;
  static final int YH = 5;
  static final int HX = 6;
  static final int HY = 7;
  static final int HH = 8;
  static final int CELLS = 9;

  /** Where a cell of a 3 by 3 matrix sits in its array, stored row by row. */
  static int cell(int row, int column) {
    return row * STATES + column;
  }
}
