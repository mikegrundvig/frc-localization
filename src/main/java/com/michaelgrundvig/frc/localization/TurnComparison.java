package com.michaelgrundvig.frc.localization;

import java.util.ArrayDeque;

/**
 * The {@link SlipCheck}'s comparison of the wheels' turning with the gyro's.
 *
 * <p>Over a sliding window of the latest loops, the gyro and the wheels should turn the robot
 * alike. Two things are allowed for: the gyro's noise and unevenly worn wheels, as a fixed angle,
 * and worn tread or a gyro's scale, as a share of the turning. What the two differ by beyond those
 * is how far the wheels slid around the robot's center. That angle, at the wheels' reach from the
 * center and over the time the window covers, is how fast they slid.
 *
 * <p>A tank drive's two sides always agree with each other, so on one this is the only check that
 * sees its wheels slip.
 */
final class TurnComparison {
  /** One loop's turns: how far the wheels, and the IMU, turned the robot over it, in radians. */
  private record Turn(double seconds, double wheels, double imu) {}

  private final double windowSeconds;
  private final double toleranceRadians;
  private final double scaleTolerance;
  private final double wheelReachMeters;

  /** The loops the window holds, oldest first, and how long they took together. */
  private final ArrayDeque<Turn> turns = new ArrayDeque<>();

  private double turnsSeconds;

  private double apartRadians;
  private double slipMetersPerSecond;

  TurnComparison(SlipCheck.Settings settings, double wheelReachMeters) {
    windowSeconds = settings.windowSeconds();
    toleranceRadians = Math.toRadians(settings.turnToleranceDegrees());
    scaleTolerance = settings.turnScaleTolerance();
    this.wheelReachMeters = wheelReachMeters;
  }

  /** Forgets the window: there is nothing to compare, or another IMU reads from now on. */
  void startOver() {
    turns.clear();
    turnsSeconds = 0;
    apartRadians = 0;
    slipMetersPerSecond = 0;
  }

  /**
   * Takes one loop's turns and compares the window's. A turn that is not a number starts the
   * comparison over.
   */
  void add(double seconds, double wheelTurnRadians, double imuTurnRadians) {
    if (!Double.isFinite(wheelTurnRadians) || !Double.isFinite(imuTurnRadians)) {
      startOver();
      return;
    }
    turns.addLast(new Turn(seconds, wheelTurnRadians, imuTurnRadians));
    turnsSeconds += seconds;
    // Keep the fewest loops that cover the window: drop the oldest while the rest still do.
    while (turns.size() > 1 && turnsSeconds - turns.getFirst().seconds() >= windowSeconds) {
      turnsSeconds -= turns.removeFirst().seconds();
    }
    double wheelsBeyondImu = 0;
    double imuTurned = 0;
    for (Turn turn : turns) {
      wheelsBeyondImu += turn.wheels() - turn.imu();
      imuTurned += turn.imu();
    }
    apartRadians = Math.abs(wheelsBeyondImu);
    double beyondTolerance = apartRadians - toleranceRadians - scaleTolerance * Math.abs(imuTurned);
    if (beyondTolerance > 0) {
      slipMetersPerSecond = beyondTolerance * wheelReachMeters / turnsSeconds;
    } else {
      slipMetersPerSecond = 0;
    }
  }

  /** How far the wheels turned the robot beyond the gyro over the window, either way. */
  double apartRadians() {
    return apartRadians;
  }

  /** How fast the wheels may be sliding, as the gyro sees them turning: zero within tolerance. */
  double slipMetersPerSecond() {
    return slipMetersPerSecond;
  }
}
