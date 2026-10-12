package com.michaelgrundvig.frc.localization;

import java.util.Arrays;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;

/**
 * What the wheels read at one moment: for each wheel, how far it has rolled and which way it
 * points, relative to the robot. One shape for every drivetrain: a swerve module is a distance and
 * its steering angle, each side of a tank drive a distance at angle zero, a mecanum wheel a
 * distance at its fixed angle. The drivetrain's kinematics turns two samples into a {@link
 * BodyMotion}.
 *
 * <p>A wheel that didn't report (its motor controllers off the bus, say) reads a value that isn't a
 * number, its distance or its angle: the kinematics leaves it out and fits the motion from the
 * rest.
 *
 * <p>Immutable: the arrays are copied in, and read one wheel at a time.
 */
public final class OdometrySample {
  private final long timestampNanos;
  private final double[] distancesMeters;
  private final double[] anglesRadians;

  /**
   * @param timestampNanos when the wheels were read
   * @param distancesMeters how far each wheel has rolled since the robot started, in meters; not a
   *     number for a wheel that didn't report
   * @param anglesRadians which way each wheel points, counterclockwise from the robot's forward;
   *     not a number for a wheel that didn't report
   */
  public OdometrySample(long timestampNanos, double[] distancesMeters, double[] anglesRadians) {
    if (timestampNanos < 0) {
      throw new IllegalArgumentException("a time on the robot's clock: " + timestampNanos);
    }
    if (distancesMeters.length == 0 || distancesMeters.length != anglesRadians.length) {
      throw new IllegalArgumentException(
          "one distance and one angle for each wheel: "
              + distancesMeters.length
              + " distances, "
              + anglesRadians.length
              + " angles");
    }
    this.timestampNanos = timestampNanos;
    this.distancesMeters = distancesMeters.clone();
    this.anglesRadians = anglesRadians.clone();
  }

  /** A swerve drivetrain's modules, as WPILib's odometry takes them. */
  public static OdometrySample ofSwerve(long timestampNanos, SwerveModulePosition... modules) {
    double[] distances = new double[modules.length];
    double[] angles = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      distances[i] = modules[i].distance;
      angles[i] = modules[i].angle.getRadians();
    }
    return new OdometrySample(timestampNanos, distances, angles);
  }

  public long timestampNanos() {
    return timestampNanos;
  }

  public int wheels() {
    return distancesMeters.length;
  }

  public double distanceMeters(int wheel) {
    return distancesMeters[wheel];
  }

  public double angleRadians(int wheel) {
    return anglesRadians[wheel];
  }

  /** Whether a wheel reported: both its distance and its angle are numbers. */
  public boolean reported(int wheel) {
    return Double.isFinite(distancesMeters[wheel]) && Double.isFinite(anglesRadians[wheel]);
  }

  /** The wheels as swerve modules, for WPILib's kinematics. */
  public SwerveModulePosition[] toSwerveModulePositions() {
    SwerveModulePosition[] modules = new SwerveModulePosition[distancesMeters.length];
    for (int i = 0; i < modules.length; i++) {
      modules[i] = new SwerveModulePosition(distancesMeters[i], new Rotation2d(anglesRadians[i]));
    }
    return modules;
  }

  @Override
  public boolean equals(@Nullable Object other) {
    return other instanceof OdometrySample sample
        && timestampNanos == sample.timestampNanos
        && Arrays.equals(distancesMeters, sample.distancesMeters)
        && Arrays.equals(anglesRadians, sample.anglesRadians);
  }

  @Override
  public int hashCode() {
    return 31 * (31 * Long.hashCode(timestampNanos) + Arrays.hashCode(distancesMeters))
        + Arrays.hashCode(anglesRadians);
  }

  @Override
  public String toString() {
    return "OdometrySample[t="
        + timestampNanos
        + " ns, distances="
        + Arrays.toString(distancesMeters)
        + ", angles="
        + Arrays.toString(anglesRadians)
        + "]";
  }
}
