package com.michaelgrundvig.frc.localization.calibration;

import java.util.Locale;

/**
 * Whether the IMU reads level when the robot is. On a level floor, standing still, its pitch and
 * roll should average zero. If they don't, the IMU's mount calibration is off (the Pigeon 2's is
 * done in Phoenix Tuner X with the robot level), and every tilt the robot reads is off by as much.
 */
public final class LevelCheck {
  private double sumPitch;
  private double sumRoll;
  private int count;

  public void add(double pitchRadians, double rollRadians) {
    sumPitch += pitchRadians;
    sumRoll += rollRadians;
    count++;
  }

  public int count() {
    return count;
  }

  /** The average pitch read so far, or zero with nothing read. */
  public double pitchRadians() {
    if (count == 0) {
      return 0;
    }
    return sumPitch / count;
  }

  /** The average roll read so far, or zero with nothing read. */
  public double rollRadians() {
    if (count == 0) {
      return 0;
    }
    return sumRoll / count;
  }

  /** Whether both are within a tolerance of zero. */
  public boolean isLevel(double toleranceRadians) {
    return Math.abs(pitchRadians()) <= toleranceRadians
        && Math.abs(rollRadians()) <= toleranceRadians;
  }

  /** One line for the pit. */
  public String describe(double toleranceRadians) {
    String reading =
        String.format(
            Locale.ROOT,
            "pitch %.2f°, roll %.2f° over %d readings",
            Math.toDegrees(pitchRadians()),
            Math.toDegrees(rollRadians()),
            count);
    if (isLevel(toleranceRadians)) {
      return "Level: " + reading;
    }
    return "Not level: " + reading + "; on a level floor, recalibrate the IMU's mount";
  }
}
