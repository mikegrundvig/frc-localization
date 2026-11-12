package com.michaelgrundvig.frc.localization;

import org.wpilib.math.geometry.Pose2d;

/**
 * Where the wheels and the IMU alone put the robot since it was last placed: the estimate's
 * odometry with nothing the cameras said, and no correction of the IMU's errors. Beside the
 * estimate in the log, it shows how far the cameras moved the estimate, and when: a jump in the
 * estimate that this pose doesn't make came from a camera or a recovery.
 *
 * <p>The estimator keeps it as it steps, and it's read-only from outside: three numbers, so the
 * robot can log it every loop without making anything.
 */
public final class OdometryOnly {
  private double xMeters;
  private double yMeters;

  /** What's added to the IMU's yaw for the heading: set as the robot is placed. */
  private double headingOffset;

  /** The yaw as of the newest step. */
  private double yaw;

  OdometryOnly(Pose2d start) {
    place(start, 0);
  }

  /** Where it starts again: where the robot was placed, with the yaw it read then. */
  void place(Pose2d pose, double yawRadians) {
    xMeters = pose.getX();
    yMeters = pose.getY();
    yaw = yawRadians;
    headingOffset = pose.getRotation().getRadians() - yawRadians;
  }

  /** A heading the drive team gave, its position kept. */
  void face(double headingRadians) {
    headingOffset = headingRadians - yaw;
  }

  /**
   * One odometry step: the wheels' travel, turned by the heading halfway through the step's turn.
   *
   * @param newYawRadians the yaw at the step's end, unwrapped as the estimator keeps it
   */
  void step(BodyMotion motion, double newYawRadians) {
    double heading = (yaw + newYawRadians) / 2 + headingOffset;
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);
    xMeters += cos * motion.dxMeters() - sin * motion.dyMeters();
    yMeters += sin * motion.dxMeters() + cos * motion.dyMeters();
    yaw = newYawRadians;
  }

  public double xMeters() {
    return xMeters;
  }

  public double yMeters() {
    return yMeters;
  }

  /** Its heading, counterclockwise, in radians: unwrapped, as the IMU turned. */
  public double headingRadians() {
    return yaw + headingOffset;
  }
}
