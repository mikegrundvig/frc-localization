package com.michaelgrundvig.frc.localization;

/**
 * A drivetrain's kinematics, as localization needs it: how the robot moved between two readings of
 * its wheels. One implementation per kind of drivetrain (swerve, differential, mecanum); the
 * estimator only ever sees the {@link BodyMotion}, so it works with any of them.
 */
public interface DriveKinematicsModel {
  /**
   * How the robot moved between two samples of its wheels.
   *
   * @throws IllegalArgumentException if the samples don't have this drivetrain's wheels
   */
  BodyMotion motion(OdometrySample from, OdometrySample to);
}
