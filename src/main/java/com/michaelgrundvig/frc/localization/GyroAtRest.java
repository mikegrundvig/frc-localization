package com.michaelgrundvig.frc.localization;

/**
 * The gyro's drift, read off while the robot stands still. A robot whose wheels are not moving is
 * not turning, so whatever its gyro turns by in that time is the gyro's own drift. This is how a
 * gyro is calibrated at rest, done again every time the robot stands for a moment: before a match,
 * and each time it stops to score.
 *
 * <p>The drift is read over a stretch of standing rather than a step, since a gyro's noise over one
 * step is far more than any drift. Over a stretch of T seconds the gyro's own noise adds a heading
 * variance of q T, with q the heading variance it adds each second, so the drift read off it, a
 * turn over T, has a variance of q / T.
 *
 * <p>A robot lifted and carried turns with its wheels still. So a stretch counts only where what it
 * read is a drift a gyro could have: within three standard deviations of the drift the
 * configuration allows and the stretch's own noise together. A carry turns the robot many times
 * faster than that.
 *
 * <p>{@link #step} runs for every wheel reading, 250 times a second, so it works on plain numbers
 * and makes nothing.
 */
final class GyroAtRest {
  /**
   * How long a stretch of standing is read over, in seconds. With the default noise (0.5° in a
   * minute) one second reads the drift to 4° a minute, which is about what the estimator allows a
   * gyro before anything is known of it, so each stretch tells it something; and it is short enough
   * to fit in a stop to score.
   */
  static final double STRETCH_SECONDS = 1.0;

  /**
   * The robot stands still while its wheels carry it slower than this, in meters a second, and turn
   * it slower than the next, in radians a second: wheels held by their motors or their brakes, with
   * room for the noise of their encoders.
   */
  private static final double STILL_METERS_PER_SECOND = 0.02;

  private static final double STILL_RADIANS_PER_SECOND = 0.05;

  /** A stretch counts where its drift is within this many standard deviations of none. */
  private static final double BELIEVABLE_STD_DEVS = 3;

  /** The variance of a drift read over one stretch, in (radians a second) squared: q / T. */
  private final double varianceOfAStretch;

  /** The fastest drift a stretch may read and count, in radians a second. */
  private final double fastestBelievable;

  // The stretch of standing so far: how long, and how far the gyro and the wheels each turned.
  private double seconds;
  private double gyroTurnRadians;
  private double wheelTurnRadians;

  /**
   * @param headingVariancePerSecond the heading variance the gyro's own noise adds each second, in
   *     square radians
   * @param allowedDriftStdDev how fast the gyro may drift before anything is known of it, in
   *     radians a second
   */
  GyroAtRest(double headingVariancePerSecond, double allowedDriftStdDev) {
    varianceOfAStretch = headingVariancePerSecond / STRETCH_SECONDS;
    fastestBelievable =
        BELIEVABLE_STD_DEVS
            * Math.sqrt(allowedDriftStdDev * allowedDriftStdDev + varianceOfAStretch);
  }

  /** Starts the stretch again: the robot moved, or the gyro did not turn this step. */
  void clear() {
    seconds = 0;
    gyroTurnRadians = 0;
    wheelTurnRadians = 0;
  }

  /**
   * One odometry step the gyro turned.
   *
   * @param stepSeconds how long it lasted
   * @param motion how the wheels say the robot moved over it
   * @param gyroTurn how far the gyro turned over it, in radians
   * @return the drift a stretch of standing that ends with this step read, in radians a second: how
   *     much faster the robot turned than its gyro said. Not a number while a stretch is still
   *     running, when the robot is moving, and for a stretch whose drift no gyro could have
   */
  double step(double stepSeconds, BodyMotion motion, double gyroTurn) {
    boolean standing =
        Math.hypot(motion.dxMeters(), motion.dyMeters()) < STILL_METERS_PER_SECOND * stepSeconds
            && Math.abs(motion.dthetaRadians()) < STILL_RADIANS_PER_SECOND * stepSeconds;
    if (!standing) {
      clear();
      return Double.NaN;
    }
    seconds += stepSeconds;
    gyroTurnRadians += gyroTurn;
    wheelTurnRadians += motion.dthetaRadians();
    if (seconds < STRETCH_SECONDS) {
      return Double.NaN;
    }
    // The robot turned by what its wheels say, which is next to nothing. The gyro's drift is what
    // the gyro's turn falls short of that by, each second.
    double drift = (wheelTurnRadians - gyroTurnRadians) / seconds;
    clear();
    boolean believable = Math.abs(drift) <= fastestBelievable;
    return believable ? drift : Double.NaN;
  }

  /** The variance of a drift {@link #step} returns, in (radians a second) squared. */
  double variance() {
    return varianceOfAStretch;
  }
}
