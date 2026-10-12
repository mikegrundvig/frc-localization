package com.michaelgrundvig.frc.localization;

/**
 * Where each error the {@link PoseFilter} considers sits among its columns.
 *
 * <p>For every error it considers but never estimates, the filter keeps how the estimate's own
 * errors have come to depend on it. Those are the columns of one table, in this order:
 *
 * <ol>
 *   <li>the gyro's scale;
 *   <li>the backup IMU's scale;
 *   <li>four columns for each of {@link #MAX_CAMERAS} cameras: the camera's turn on its mount, its
 *       shift on its mount along the robot's forward, the same along the robot's left, and its
 *       headings' shift;
 *   <li>two columns for each of the field's tags: the tag's placement along the field's x, and
 *       along its y.
 * </ol>
 *
 * <p>A camera or a tag is known here by its <em>place</em>: a number from zero that the estimator
 * gives each camera as it first appears and each field tag at the start. One the estimator has no
 * place for is given {@link #NO_PLACE}, and has no columns.
 */
final class ConsideredErrors {
  private ConsideredErrors() {}

  /** The most cameras whose errors the filter keeps apart. Any more are treated as independent. */
  static final int MAX_CAMERAS = 8;

  /** The place of a camera or tag the filter keeps no columns for. */
  static final int NO_PLACE = -1;

  /** The column of an error the filter keeps no column for. */
  static final int NO_COLUMN = -1;

  static final int GYRO_SCALE = 0;
  static final int BACKUP_SCALE = 1;

  private static final int IMU_COLUMNS = 2;
  private static final int COLUMNS_PER_CAMERA = 4;
  private static final int COLUMNS_PER_TAG = 2;

  /** How many columns there are for a field with this many tags. */
  static int count(int tags) {
    return IMU_COLUMNS + COLUMNS_PER_CAMERA * MAX_CAMERAS + COLUMNS_PER_TAG * tags;
  }

  /** Whether a place is a real one, with columns of its own. */
  static boolean hasPlace(int place) {
    return place >= 0;
  }

  /** Whether an error has a column: the filter keeps how the estimate depends on it. */
  static boolean hasColumn(int column) {
    return column >= 0;
  }

  static int cameraTurn(int camera) {
    return IMU_COLUMNS + COLUMNS_PER_CAMERA * camera;
  }

  static int cameraShiftForward(int camera) {
    return cameraTurn(camera) + 1;
  }

  static int cameraShiftLeft(int camera) {
    return cameraTurn(camera) + 2;
  }

  static int cameraHeadingsShift(int camera) {
    return cameraTurn(camera) + 3;
  }

  static int tagAlongX(int tag) {
    return IMU_COLUMNS + COLUMNS_PER_CAMERA * MAX_CAMERAS + COLUMNS_PER_TAG * tag;
  }

  static int tagAlongY(int tag) {
    return tagAlongX(tag) + 1;
  }
}
