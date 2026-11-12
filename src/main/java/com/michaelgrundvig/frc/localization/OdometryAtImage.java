package com.michaelgrundvig.frc.localization;

/**
 * What odometry says of the moment a camera's image was taken: where it alone puts the robot, and
 * how the robot was moving. The replay works it out for each measurement it applies, and the gate
 * and the watch on the cameras judge the measurement by it.
 *
 * @param x where odometry alone puts the robot along the field's x, in meters: the estimate as it
 *     stood at the start of the history, carried on by the wheels and the gyro
 * @param y and along y
 * @param variance how unsure that place is along each axis, as a variance: what the steps since the
 *     start of the history have added
 * @param velocityX how fast the robot was moving along the field's x, in meters a second. An image
 *     stamped a little late is off by this times the lateness ({@link CameraWatch#lateArmX}).
 * @param velocityY and along y
 * @param yawRate how fast the robot was turning, in radians a second
 * @param yaw the gyro's continuous yaw, in radians
 */
record OdometryAtImage(
    double x,
    double y,
    double variance,
    double velocityX,
    double velocityY,
    double yawRate,
    double yaw) {
  /** At a fraction of the way through a step of the history. */
  static OdometryAtImage partWayThrough(OdometryHistory history, int step, double fraction) {
    return new OdometryAtImage(
        history.xByOdometryAlone(step, fraction),
        history.yByOdometryAlone(step, fraction),
        history.varianceByOdometryAlone(step),
        history.velocityX(step),
        history.velocityY(step),
        history.yawRate(step),
        history.yawPartWayThrough(step, fraction));
  }

  /**
   * At the history's only step, which happens just after it starts again: odometry alone puts the
   * robot where the filter has it, nothing has added to its doubt, and nothing says how it moves.
   */
  static OdometryAtImage atTheOnlyStep(OdometryHistory history, PoseFilter filter) {
    return new OdometryAtImage(
        filter.x(), filter.y(), 0, 0, 0, 0, history.yawAfter(history.newestStep()));
  }
}
