package com.michaelgrundvig.frc.localization.calibration;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The wheels' radius, measured by spinning in place against the gyro (one top team's procedure).
 *
 * <p>Turning in place, every wheel rolls along a circle around the robot's center. When the robot
 * turns by an angle, a wheel at a distance from the center rolls that angle times the distance, and
 * the wheel's radius is how far it rolled over how far the wheel itself rotated:
 *
 * <pre>radius = (robot's turn × wheel's distance from the center) / wheel's rotation</pre>
 *
 * <p>Tread wears, so the radius shrinks through a season. Measure again before each event.
 *
 * <p>The gyro is the reference, so its scale error carries straight into the result: a gyro that
 * reads 0.3% high gives a radius 0.3% too large. Calibrate the gyro's scale first. Spin slowly, so
 * that the wheels roll instead of scrubbing.
 */
public final class WheelRadiusCalibration {
  /** How far each wheel touches the floor from the robot's center, in meters. */
  private final double[] distancesFromCenterMeters;

  /** Each wheel's rotation at the start of the spin, and at the latest reading, in radians. */
  private final double[] startRotationsRadians;

  private final double[] latestRotationsRadians;

  /** How far the robot has turned since the start, by the gyro, in radians, signed. */
  private double turnedRadians;

  private double lastYawRadians;
  private boolean started;

  /**
   * @param moduleLocations where each wheel touches the floor, from the robot's center
   */
  public WheelRadiusCalibration(Translation2d[] moduleLocations) {
    distancesFromCenterMeters = new double[moduleLocations.length];
    for (int i = 0; i < moduleLocations.length; i++) {
      distancesFromCenterMeters[i] = moduleLocations[i].getNorm();
      boolean awayFromTheCenter = distancesFromCenterMeters[i] > 0;
      if (!awayFromTheCenter) {
        throw new IllegalArgumentException("a wheel can't sit at the robot's center");
      }
    }
    startRotationsRadians = new double[distancesFromCenterMeters.length];
    latestRotationsRadians = new double[distancesFromCenterMeters.length];
  }

  /**
   * Where the spin starts: the gyro's yaw, and each wheel's rotation, in radians.
   *
   * @param gyroYawRadians from any zero, wrapped or not
   */
  public void start(double gyroYawRadians, double[] wheelRotationsRadians) {
    requireEveryWheel(wheelRotationsRadians);
    System.arraycopy(
        wheelRotationsRadians, 0, startRotationsRadians, 0, startRotationsRadians.length);
    System.arraycopy(
        wheelRotationsRadians, 0, latestRotationsRadians, 0, latestRotationsRadians.length);
    lastYawRadians = gyroYawRadians;
    turnedRadians = 0;
    started = true;
  }

  /**
   * A reading during the spin. Give one at least once a loop, so that the gyro's turn is followed
   * across wraps.
   */
  public void update(double gyroYawRadians, double[] wheelRotationsRadians) {
    if (!started) {
      throw new IllegalStateException("start the spin first");
    }
    requireEveryWheel(wheelRotationsRadians);
    turnedRadians += MathUtil.angleModulus(gyroYawRadians - lastYawRadians);
    lastYawRadians = gyroYawRadians;
    System.arraycopy(
        wheelRotationsRadians, 0, latestRotationsRadians, 0, latestRotationsRadians.length);
  }

  /** How far the robot has turned since the start, by the gyro, either way. */
  public double turnedRadians() {
    return Math.abs(turnedRadians);
  }

  /**
   * The wheel radius the spin says, in meters, from all the wheels together: the distance they
   * rolled, by the gyro, over how far they rotated.
   *
   * @return NaN before the wheels have rotated
   */
  public double radiusMeters() {
    double rolledMeters = 0;
    double rotatedRadians = 0;
    for (int i = 0; i < distancesFromCenterMeters.length; i++) {
      rolledMeters += rolledMeters(i);
      rotatedRadians += rotatedRadians(i);
    }
    if (rotatedRadians > 0) {
      return rolledMeters / rotatedRadians;
    }
    return Double.NaN;
  }

  /**
   * Each wheel's own radius, in meters, worked out the same way. A wheel far from the rest is worn
   * differently, or slipping.
   *
   * @return for each wheel its radius, or NaN for a wheel that hasn't rotated
   */
  public double[] radiiMeters() {
    double[] radii = new double[distancesFromCenterMeters.length];
    for (int i = 0; i < distancesFromCenterMeters.length; i++) {
      double rotatedRadians = rotatedRadians(i);
      if (rotatedRadians > 0) {
        radii[i] = rolledMeters(i) / rotatedRadians;
      } else {
        radii[i] = Double.NaN;
      }
    }
    return radii;
  }

  /** How far a wheel has rolled along its circle, by the gyro: the robot's turn times its arm. */
  private double rolledMeters(int wheel) {
    return Math.abs(turnedRadians) * distancesFromCenterMeters[wheel];
  }

  /** How far a wheel has rotated since the start, either way. */
  private double rotatedRadians(int wheel) {
    return Math.abs(latestRotationsRadians[wheel] - startRotationsRadians[wheel]);
  }

  private void requireEveryWheel(double[] wheelRotationsRadians) {
    if (wheelRotationsRadians.length != distancesFromCenterMeters.length) {
      throw new IllegalArgumentException("one rotation for each wheel");
    }
  }
}
