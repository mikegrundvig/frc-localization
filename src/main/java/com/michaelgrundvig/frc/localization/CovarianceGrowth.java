package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.localization.StateLayout.GYRO_DRIFT;
import static com.michaelgrundvig.frc.localization.StateLayout.GYRO_SCALE;
import static com.michaelgrundvig.frc.localization.StateLayout.HEADING;
import static com.michaelgrundvig.frc.localization.StateLayout.HH;
import static com.michaelgrundvig.frc.localization.StateLayout.X;
import static com.michaelgrundvig.frc.localization.StateLayout.XX;
import static com.michaelgrundvig.frc.localization.StateLayout.Y;
import static com.michaelgrundvig.frc.localization.StateLayout.YY;
import static com.michaelgrundvig.frc.localization.StateLayout.cell;
import static com.michaelgrundvig.frc.localization.StateLayout.turnOf;

import java.util.Arrays;

/**
 * The filter's time update of its uncertainty: how each part of an odometry step grows P, the
 * state's covariance, and C, its cross-covariance with the considered errors. {@link PoseFilter}
 * moves the state itself, and shares both arrays with this.
 */
final class CovarianceGrowth {
  /** P, the filter's, which this grows in place. */
  private final double[] covariance;

  /** C, the filter's, which this grows in place. */
  private final double[] crossCovariance;

  /** How many errors the filter considers, which is how many columns C has. */
  private final int consideredCount;

  /** The backup IMU's scale's variance: considered, never estimated. */
  private final double backupScaleVariance;

  /**
   * How far each camera's turn may wander, by its place: the variance each second adds to it, in
   * radians squared.
   */
  private final double[] turnVariancePerSecond = new double[ConsideredErrors.MAX_CAMERAS];

  private final double shiftCorrelationMeters;
  private final double shiftCorrelationRadians;

  // How fast each of the gyro's errors may wander: the variance each second adds to it.
  private final double scaleVariancePerSecond;
  private final double driftVariancePerSecond;

  /** How many of the state's values are in use, as of the step being predicted. */
  private int activeStates = StateLayout.STATES;

  CovarianceGrowth(
      ConsideredSizes sizes,
      EstimatorConfig.GyroErrors gyroErrors,
      GyroDoubt gyroDoubt,
      double turnWanderPerSqrtSecond,
      double[] covariance,
      double[] crossCovariance,
      int consideredCount) {
    this.covariance = covariance;
    this.crossCovariance = crossCovariance;
    this.consideredCount = consideredCount;
    boolean estimated = gyroErrors == EstimatorConfig.GyroErrors.ESTIMATED;
    scaleVariancePerSecond = estimated ? square(gyroDoubt.scaleWanderPerSqrtSecond()) : 0;
    driftVariancePerSecond = estimated ? square(gyroDoubt.driftWanderPerSqrtSecond()) : 0;
    backupScaleVariance = square(sizes.backupScaleStdDev());
    shiftCorrelationMeters = sizes.shiftCorrelationMeters();
    shiftCorrelationRadians = sizes.shiftCorrelationRadians();
    Arrays.fill(turnVariancePerSecond, square(turnWanderPerSqrtSecond));
  }

  /** How far a camera's turn may wander: the variance each second adds to it. */
  void setTurnWander(int camera, double variancePerSecond) {
    turnVariancePerSecond[camera] = variancePerSecond;
  }

  /**
   * Grows the uncertainty by what part of a step's move adds.
   *
   * @param moveX how far the position moved along the field's x, in meters
   * @param moveY and along y
   * @param turn how far the gyro turned over it
   * @param seconds how long it lasted
   * @param positionNoise the variance to add to each position axis
   * @param headingNoise the variance to add to the heading
   * @param turnedBy what turned the robot
   * @param activeStates how many of the state's values are in use
   */
  void grow(
      double moveX,
      double moveY,
      double turn,
      double seconds,
      double positionNoise,
      double headingNoise,
      YawSource turnedBy,
      int activeStates) {
    this.activeStates = activeStates;
    // A heading offset that is a little off swings the move about its start: an offset off by dθ
    // puts x off by -moveY dθ and y off by moveX dθ.
    errorGrowsBy(X, HEADING, -moveY);
    errorGrowsBy(Y, HEADING, moveX);
    switch (turnedBy) {
      case GYRO -> {
        // The gyro's scale error adds the turn times itself to the heading, and its drift the
        // time times itself.
        errorGrowsBy(HEADING, GYRO_SCALE, turn);
        errorGrowsBy(HEADING, GYRO_DRIFT, seconds);
        covariance[cell(GYRO_SCALE, GYRO_SCALE)] += scaleVariancePerSecond * seconds;
        covariance[cell(GYRO_DRIFT, GYRO_DRIFT)] += driftVariancePerSecond * seconds;
      }
      case BACKUP -> addScaleError(turn, ConsideredErrors.BACKUP_SCALE, backupScaleVariance);
      case WHEELS -> {}
    }
    letGoOfHeadingsShifts(Math.hypot(moveX, moveY), Math.abs(turn));
    letCameraTurnsWander(seconds);

    covariance[XX] += positionNoise;
    covariance[YY] += positionNoise;
    covariance[HH] += headingNoise;
  }

  /**
   * The covariance's prediction, P = F P F^T, for one way an error feeds another: the error of one
   * value of the state grows by a factor times the error of another. F, the Jacobian of the new
   * state with respect to the old, is then the identity with that factor in the first value's row
   * and the other's column. A heading error swinging a move into x is one such F, and the gyro's
   * scale error turning into heading is another.
   *
   * <p>F P adds the factor times the other's row to the first value's row, and then (F P) F^T adds
   * the factor times the other's column to the first value's column. Between them the first value's
   * own variance gains twice the factor times the two's covariance, plus the factor squared times
   * the other's variance. C, the dependence on the considered errors, is predicted by the same F: C
   * = F C adds the factor times the other's row to the first value's row.
   *
   * @param grows the value of the state whose error grows
   * @param from the value whose error feeds it, which is another one
   * @param factor how much of that error is added
   */
  private void errorGrowsBy(int grows, int from, double factor) {
    for (int column = 0; column < activeStates; column++) {
      covariance[cell(grows, column)] += factor * covariance[cell(from, column)];
    }
    for (int row = 0; row < activeStates; row++) {
      covariance[cell(row, grows)] += factor * covariance[cell(row, from)];
    }
    for (int column = 0; column < consideredCount; column++) {
      crossCovariance[cross(grows, column)] += factor * crossCovariance[cross(from, column)];
    }
  }

  /**
   * The backup IMU's scale error, which is considered and never estimated. An IMU that reads its
   * turns a little large or small adds its scale error times the turn to the heading's error. It is
   * the same error every turn, so the heading comes to depend on it: with s the scale error and v
   * its variance, the heading's error grows by turn times s, which adds
   *
   * <pre>
   *   to the heading's variance                    2 turn cov(heading, s) + turn² v
   *   to cov(any other value, heading)             turn cov(that value, s)
   *   to cov(heading, s)                           turn v
   * </pre>
   *
   * <p>A scale with no variance is not considered at all.
   */
  private void addScaleError(double turn, int scaleColumn, double scaleVariance) {
    boolean considered = scaleVariance > 0;
    if (!considered) {
      return;
    }
    for (int state = 0; state < activeStates; state++) {
      if (state == HEADING) {
        continue;
      }
      covariance[cell(state, HEADING)] += turn * crossCovariance[cross(state, scaleColumn)];
      covariance[cell(HEADING, state)] = covariance[cell(state, HEADING)];
    }
    covariance[HH] +=
        2 * turn * crossCovariance[cross(HEADING, scaleColumn)] + turn * turn * scaleVariance;
    crossCovariance[cross(HEADING, scaleColumn)] += turn * scaleVariance;
  }

  /**
   * As the robot moves and turns, each camera sees other tags, or the same ones from elsewhere, and
   * its headings' shift becomes a different one. So the state's dependence on each camera's
   * headings' shift decays, by a factor of e for each correlation distance moved and each
   * correlation angle turned. A camera's shift on its mount and each tag's placement stay what they
   * are, and their columns are left alone.
   */
  private void letGoOfHeadingsShifts(double movedMeters, double turnedRadians) {
    double kept =
        Math.exp(-movedMeters / shiftCorrelationMeters - turnedRadians / shiftCorrelationRadians);
    if (kept < 1) {
      for (int camera = 0; camera < ConsideredErrors.MAX_CAMERAS; camera++) {
        for (int state = 0; state < activeStates; state++) {
          crossCovariance[cross(state, ConsideredErrors.cameraHeadingsShift(camera))] *= kept;
        }
      }
    }
  }

  /** How many cameras' turns are in use: places 0 up. */
  private int activeCameras() {
    return activeStates - StateLayout.FIRST_CAMERA_TURN;
  }

  /** Each camera's turn may wander a little each second: its variance grows by the time. */
  private void letCameraTurnsWander(double seconds) {
    for (int camera = 0; camera < activeCameras(); camera++) {
      int turn = turnOf(camera);
      covariance[cell(turn, turn)] += turnVariancePerSecond[camera] * seconds;
    }
  }

  /** Where a cell of C sits in its array, stored row by row. */
  private int cross(int row, int column) {
    return row * consideredCount + column;
  }

  private static double square(double value) {
    return value * value;
  }
}
