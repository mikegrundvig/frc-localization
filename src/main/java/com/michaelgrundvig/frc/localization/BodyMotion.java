package com.michaelgrundvig.frc.localization;

/**
 * How the robot moved between two odometry samples, in its own frame, as its wheels tell it. The
 * drivetrain's kinematics (swerve, differential, mecanum) produces it; the estimator turns it by
 * the gyro's heading and adds it to the pose.
 *
 * @param dxMeters forward
 * @param dyMeters to the left
 * @param dthetaRadians counterclockwise. The estimator takes rotation from the gyro, and uses this
 *     only to check it, and when the gyro is lost.
 * @param residualMeters how far the wheels' disagreement with any one rigid motion of the robot may
 *     have put this motion off: zero when they all agree, more when a wheel slips, lifts, or is
 *     pushed. The estimator doubts the motion by as much.
 */
public record BodyMotion(
    double dxMeters, double dyMeters, double dthetaRadians, double residualMeters) {
  public BodyMotion {
    if (!Double.isFinite(dxMeters)
        || !Double.isFinite(dyMeters)
        || !Double.isFinite(dthetaRadians)) {
      throw new IllegalArgumentException(
          "a motion must be numbers: " + dxMeters + ", " + dyMeters + ", " + dthetaRadians);
    }
    if (!(residualMeters >= 0) || Double.isInfinite(residualMeters)) {
      throw new IllegalArgumentException("the residual must be finite, and zero or more");
    }
  }
}
