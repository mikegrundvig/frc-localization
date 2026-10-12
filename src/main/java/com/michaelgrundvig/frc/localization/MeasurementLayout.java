package com.michaelgrundvig.frc.localization;

/**
 * How a camera measurement's own square matrices are laid out in their arrays: the innovation's
 * covariance and its inverse, 3 by 3, stored row by row. Its rows are the measurement's, in the
 * order of {@link StateLayout}'s first three values: x, y, and the heading.
 */
final class MeasurementLayout {
  private MeasurementLayout() {}

  /** The most rows a measurement has, which is how many numbers each stored row holds. */
  static final int ROWS = 3;

  static final int CELLS = ROWS * ROWS;

  // The cells the working names: the position's own block, and the heading's own variance.
  static final int X_WITH_X = 0;
  static final int X_WITH_Y = 1;
  static final int Y_WITH_X = 3;
  static final int Y_WITH_Y = 4;
  static final int HEADING_WITH_HEADING = 8;

  /** Where the cell for two of the measurement's rows sits in its array. */
  static int between(int rowA, int rowB) {
    return rowA * ROWS + rowB;
  }
}
