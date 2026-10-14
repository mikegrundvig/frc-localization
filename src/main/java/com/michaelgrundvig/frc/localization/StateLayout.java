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

  // The state's values, in the order every row and column follows: where the robot is, the heading
  // offset, the gyro's two errors, its scale and its drift, and then each camera's turn on its
  // mount, one for each camera the filter keeps apart. A camera measurement's rows follow the first
  // three: its x, its y, and its heading if it has one. No measurement has a row of its own for the
  // gyro's errors or a camera's turn.
  static final int X = 0;
  static final int Y = 1;
  static final int HEADING = 2;
  static final int GYRO_SCALE = 3;
  static final int GYRO_DRIFT = 4;

  /** The turn on its mount of the camera with place 0; the next camera's follows it, and so on. */
  static final int FIRST_CAMERA_TURN = 5;

  static final int STATES = FIRST_CAMERA_TURN + ConsideredErrors.MAX_CAMERAS;

  /** A measurement of position alone has two rows; one with a heading of its own has three. */
  static final int POSITION_ROWS = 2;

  static final int POSE_ROWS = 3;

  static final int CELLS = STATES * STATES;

  // The cells of the pose's own block of the covariance, named by the row's and the column's
  // value: XY is the covariance of x with y.
  static final int XX = cell(X, X);
  static final int XY = cell(X, Y);
  static final int YX = cell(Y, X);
  static final int YY = cell(Y, Y);
  static final int HH = cell(HEADING, HEADING);

  /**
   * Where a cell sits in an array with one row or column for each value of the state, stored row by
   * row with {@link #STATES} to a row. The state's own covariance is one, and so is any matrix
   * between the state and a measurement's rows, whichever of the two its rows are: a measurement
   * has no more rows than the state has values.
   */
  static int cell(int row, int column) {
    return row * STATES + column;
  }

  /** Where a camera's turn on its mount sits in the state, by the camera's place. */
  static int turnOf(int camera) {
    return FIRST_CAMERA_TURN + camera;
  }

  /** Whether a value of the state is a camera's turn on its mount. */
  static boolean isACameraTurn(int state) {
    return state >= FIRST_CAMERA_TURN;
  }
}
