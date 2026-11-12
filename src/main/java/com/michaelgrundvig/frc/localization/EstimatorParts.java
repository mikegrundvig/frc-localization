package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.robot.CameraConfig;

/**
 * The estimator's parts as its configuration sizes and weighs them: each value turned from the
 * configuration's units (degrees a turn, percent a minute) into the part's own (radians a radian, a
 * share a second).
 */
final class EstimatorParts {
  /** The history has room for its seconds of odometry at up to this many steps a second. */
  private static final int MOST_STEPS_PER_SECOND = 1000;

  /**
   * It starts with room for this many, growing only if odometry runs faster. A robot's odometry
   * runs at 250 steps a second, and a quarter more leaves room for a loop that brings a step or two
   * more than its share.
   */
  private static final int USUAL_STEPS_PER_SECOND = 313;

  private static final int FEWEST_STEPS_KEPT = 16;

  private static final double RADIANS_PER_TURN = 2 * Math.PI;
  private static final double SECONDS_PER_MINUTE = 60;
  private static final double PERCENT = 100;

  private EstimatorParts() {}

  /** The filter, as the configuration weighs the gyro, the cameras and the tags. */
  static PoseFilter filter(EstimatorConfig config, HeadingCorrection heading, int tagCount) {
    CameraConfig.MountTolerance measured = CameraConfig.MountTolerance.MEASURED;
    return new PoseFilter(
        new ConsideredSizes(
            Math.toRadians(config.headingStdDevDegreesPerTurn()) / RADIANS_PER_TURN,
            Math.toRadians(config.backupHeadingStdDevDegreesPerTurn()) / RADIANS_PER_TURN,
            measured.turnStdDevRadians(),
            Math.sqrt(CameraMounts.shiftVariance(config, measured)),
            Math.toRadians(config.cameraShiftStdDevDegrees()),
            config.tagPlacementStdDevMeters(),
            config.cameraShiftCorrelationMeters(),
            Math.toRadians(config.cameraShiftCorrelationDegrees())),
        tagCount,
        heading,
        config.gyroErrors(),
        gyroDoubt(config),
        measured.turnStdDevRadians() / Math.sqrt(CameraMounts.MATCH_SECONDS));
  }

  /** The watch on the cameras, every camera starting as a measured mount would. */
  static CameraWatch watch(EstimatorConfig config) {
    CameraConfig.MountTolerance measured = CameraConfig.MountTolerance.MEASURED;
    return new CameraWatch(
        config.cameraWatchMeasurements(),
        measured.turnStdDevRadians(),
        Math.sqrt(CameraMounts.shiftVariance(config, measured)),
        config.cameraClockStdDevSeconds());
  }

  /** The history, with room for its seconds of odometry. */
  static OdometryHistory history(EstimatorConfig config, PoseFilter filter) {
    int mostSteps = (int) Math.ceil(config.historySeconds() * MOST_STEPS_PER_SECOND) + 1;
    int usualSteps = (int) Math.ceil(config.historySeconds() * USUAL_STEPS_PER_SECOND) + 1;
    return new OdometryHistory(
        Math.max(FEWEST_STEPS_KEPT, usualSteps),
        Math.max(FEWEST_STEPS_KEPT, mostSteps),
        filter.uncertaintyLength());
  }

  /**
   * The configuration's doubts about the gyro in the filter's units. A wander "per minute" is how
   * far the value may have moved after a minute, and its variance grows with time, so after one
   * second it is that over the square root of 60.
   */
  private static GyroDoubt gyroDoubt(EstimatorConfig config) {
    double sqrtSecondsPerMinute = Math.sqrt(SECONDS_PER_MINUTE);
    double driftRadiansPerSecond =
        Math.toRadians(config.gyroDriftStdDevDegreesPerMinute()) / SECONDS_PER_MINUTE;
    double scaleWander = config.gyroScaleWanderPercentPerMinute() / PERCENT / sqrtSecondsPerMinute;
    double driftWanderRadiansPerSecond =
        Math.toRadians(config.gyroDriftWanderDegreesPerMinute())
            / SECONDS_PER_MINUTE
            / sqrtSecondsPerMinute;
    return new GyroDoubt(driftRadiansPerSecond, scaleWander, driftWanderRadiansPerSecond);
  }
}
