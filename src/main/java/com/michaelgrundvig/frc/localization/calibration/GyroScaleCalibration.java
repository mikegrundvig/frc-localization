package com.michaelgrundvig.frc.localization.calibration;

import java.util.OptionalDouble;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The gyro's scale, measured from whole turns in place against a heading reference. The reference
 * is the robot's heading by the cameras before and after, standing still where they see several
 * tags.
 *
 * <p>A gyro whose scale is off by 0.3% misreads every turn by 0.3%. That is a few degrees over a
 * match of spinning, and the same share of every wheel radius measured against it.
 *
 * <p>The gyro counts the whole turns. The reference only has to say where the robot stopped, within
 * half a turn of what the gyro says. So spin enough turns for the error to show (five turns at 0.3%
 * is 5.4°), and no more than the error could stretch to half a turn.
 */
public final class GyroScaleCalibration {
  /** How far the gyro says the robot has turned since the start, in radians, signed. */
  private double gyroTurnedRadians;

  private double lastYawRadians;

  /** The reference heading at the start, in radians. */
  private double startReferenceRadians;

  /** How far the robot really turned, by the reference. It is empty until the spin is finished. */
  private OptionalDouble trueTurnRadians = OptionalDouble.empty();

  private boolean started;

  /** Standing still before the spin: the gyro's yaw, and the reference heading. */
  public void start(double gyroYawRadians, Rotation2d reference) {
    lastYawRadians = gyroYawRadians;
    startReferenceRadians = reference.getRadians();
    gyroTurnedRadians = 0;
    trueTurnRadians = OptionalDouble.empty();
    started = true;
  }

  /**
   * A reading during the spin. Give one at least once a loop, so that whole turns are counted: each
   * reading's step from the last is taken the short way around, which is right only while the robot
   * turns less than half a turn between readings.
   */
  public void update(double gyroYawRadians) {
    if (!started) {
      throw new IllegalStateException("start the spin first");
    }
    gyroTurnedRadians += MathUtil.angleModulus(gyroYawRadians - lastYawRadians);
    lastYawRadians = gyroYawRadians;
  }

  /**
   * Standing still after the spin: the gyro's yaw, and the reference heading again.
   *
   * <p>The reference says how far the robot turned only up to whole turns. The true turn is the one
   * of those that is nearest the gyro's count:
   *
   * <pre>true turn = gyro's turn + (reference's turn - gyro's turn, wrapped to within half a turn)
   * </pre>
   */
  public void finish(double gyroYawRadians, Rotation2d reference) {
    update(gyroYawRadians);
    double referenceTurnRadians = reference.getRadians() - startReferenceRadians;
    trueTurnRadians =
        OptionalDouble.of(
            gyroTurnedRadians + MathUtil.angleModulus(referenceTurnRadians - gyroTurnedRadians));
  }

  /** How far the gyro says the robot turned, in radians, signed. */
  public double gyroTurnedRadians() {
    return gyroTurnedRadians;
  }

  /**
   * How far the robot really turned, by the reference, in radians.
   *
   * @return NaN until finished
   */
  public double trueTurnedRadians() {
    return trueTurnRadians.orElse(Double.NaN);
  }

  /**
   * The factor that corrects the gyro. Its readings times this are the true rotation:
   *
   * <pre>scale = true turn / gyro's turn</pre>
   *
   * @return NaN until finished, or if the gyro says the robot didn't turn
   */
  public double scale() {
    if (trueTurnRadians.isEmpty() || gyroTurnedRadians == 0) {
      return Double.NaN;
    }
    return trueTurnRadians.getAsDouble() / gyroTurnedRadians;
  }
}
