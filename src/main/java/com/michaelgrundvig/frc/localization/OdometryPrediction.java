package com.michaelgrundvig.frc.localization;

/**
 * How each odometry step moves the filter: the wheels' travel along the arc the gyro turned
 * through, and the uncertainty the step adds. The estimator predicts through each step as it comes,
 * and the replay predicts through the steps again after a measurement.
 *
 * <p>The uncertainty a whole step adds is:
 *
 * <ul>
 *   <li>to the position, a variance for each meter driven and each radian turned, plus the step's
 *       own added variance for slip, tilt and collisions;
 *   <li>to the heading, a variance for each second passed (the gyro's drift) and for each radian
 *       turned (the noise of whatever turned the step).
 * </ul>
 *
 * <p>The filter itself adds what the gyro's scale and drift may have wandered over the time.
 */
final class OdometryPrediction {
  private static final double RADIANS_PER_TURN = 2 * Math.PI;
  private static final double SECONDS_PER_MINUTE = 60;

  // How fast the uncertainty grows, from the configuration. Each is a variance for one unit of what
  // grows it: a meter driven, a radian turned, a second passed.
  private final double positionVariancePerMeter;
  private final double positionVariancePerRadian;
  private final double headingVariancePerSecond;

  /** The heading variance a radian of turning adds when an IMU turned the step. */
  private final double imuHeadingVariancePerRadian;

  /** And when the wheels turned it, which drift far faster. */
  private final double wheelHeadingVariancePerRadian;

  private final OdometryHistory history;
  private final PoseFilter filter;

  OdometryPrediction(EstimatorConfig config, OdometryHistory history, PoseFilter filter) {
    this.history = history;
    this.filter = filter;
    positionVariancePerMeter = square(config.translationStdDevPerMeter());
    positionVariancePerRadian = square(config.translationStdDevPerRadian());
    headingVariancePerSecond = headingVariancePerSecond(config);
    imuHeadingVariancePerRadian =
        square(Math.toRadians(config.headingNoiseDegreesPerTurn())) / RADIANS_PER_TURN;
    wheelHeadingVariancePerRadian =
        square(Math.toRadians(config.wheelHeadingStdDevDegreesPerTurn())) / RADIANS_PER_TURN;
  }

  /** The heading variance each second adds, for the gyro's drift. */
  static double headingVariancePerSecond(EstimatorConfig config) {
    return square(Math.toRadians(config.headingDriftDegreesPerMinute())) / SECONDS_PER_MINUTE;
  }

  /**
   * Predicts through part of a step in the history, from one fraction of the way through it to
   * another; a whole step is from 0 to 1. A replay predicts up to each measurement inside a step,
   * applies it, and carries on from there. Each part takes its fraction of the step's travel, turn
   * and added uncertainty.
   */
  void predict(int step, double fromFraction, double toFraction) {
    double fraction = toFraction - fromFraction;
    if (fraction <= 0) {
      return;
    }
    double stepTurn = history.turnOver(step);
    double stepForward = history.travelForward(step);
    double stepLeft = history.travelLeft(step);
    double stepSeconds = Timestamps.toSeconds(history.durationNanos(step));
    YawSource turnedBy = history.turnedBy(step);

    double stepPositionVariance = positionVarianceOf(step);
    double headingVariancePerRadian =
        turnedBy == YawSource.WHEELS ? wheelHeadingVariancePerRadian : imuHeadingVariancePerRadian;
    double stepHeadingVariance =
        headingVariancePerSecond * stepSeconds + headingVariancePerRadian * Math.abs(stepTurn);

    filter.predict(
        history.yawAfter(step - 1) + fromFraction * stepTurn,
        fraction * stepTurn,
        fraction * stepForward,
        fraction * stepLeft,
        fraction * stepSeconds,
        fraction * stepPositionVariance,
        fraction * stepHeadingVariance,
        turnedBy);
  }

  /** The position variance a whole step adds. */
  double positionVarianceOf(int step) {
    double distance = Math.hypot(history.travelForward(step), history.travelLeft(step));
    return positionVariancePerMeter * distance
        + positionVariancePerRadian * Math.abs(history.turnOver(step))
        + history.addedPositionVariance(step);
  }

  /**
   * Corrects the gyro's drift by what a stretch of standing still read of it, where one ended with
   * this step ({@link GyroAtRest}). It runs as each step ends, the first time and in every replay,
   * so a replay comes to the same drift.
   */
  void learnDrift(int step) {
    double driftSeen = history.driftSeenAtRest(step);
    if (Double.isNaN(driftSeen)) {
      return;
    }
    filter.sawGyroDrift(driftSeen, history.driftSeenVariance(step));
  }

  private static double square(double value) {
    return value * value;
  }
}
