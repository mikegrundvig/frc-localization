package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.world.config.Rules.atLeast;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;

/**
 * How the vision pipeline filters camera results and how much it trusts them. The defaults start
 * from a team's 2026 code (itself AdvantageKit's template) and what top teams did in 2025 and 2026;
 * fit them from logged residuals, never from one run.
 *
 * @param maxAmbiguity a single tag's result is used only below this ambiguity (2026: 0.2)
 * @param maxHeightErrorMeters how far a solve's height may miss the floor, or a tag's known height,
 *     before the result is thrown out (2026: 0.2)
 * @param maxDistanceMeters results from tags farther than this, on average, are thrown out: the
 *     noise grows with distance squared
 * @param maxYawRateRadiansPerSecond results taken while the robot turned faster than this, lately,
 *     are thrown out: motion blur and rolling shutter (a top team's: 5 rad/s over the last 0.3 s)
 * @param linearStdDevBaseline position uncertainty in meters for one tag at one meter (2026: 0.02)
 * @param angularStdDevBaseline heading uncertainty in radians for one tag at one meter, used only
 *     when a result may correct heading (2026: 0.03)
 * @param distanceExponent uncertainty grows with the tags' distance to this power (2026: 2; teams
 *     used 2 to 4, so fit it)
 * @param headingConstrainedFactor how much less uncertain a position is when the gyro's heading
 *     fixes the orientation (a top team's factor for the same idea: 0.5)
 * @param speedStdDevFactor uncertainty grows by this fraction per meter per second of robot speed
 * @param yawRateStdDevFactor and by this fraction per radian per second of turning
 * @param maxHeadingStdDevToConstrain the gyro's heading is used to solve only while the estimate is
 *     this sure of it, in radians; until then (a robot not yet placed), full solves find it. A
 *     driver's heading reset no camera has backed is never used, however sure it says it is
 * @param minTagsForHeading a result may correct heading only from a multi-tag solution with at
 *     least this many tags
 * @param maxYawRateForHeading and only while the robot turns slower than this
 * @param maxReprojectionErrorForHeadingPixels and only when the solution fits its tags this well
 * @param mountToleranceMeters how far, on average, a camera's tags may land from where they are
 *     before the camera counts as out of calibration: an alert says to recalibrate its mount, and
 *     the level part beyond this makes its measurements that much less certain (see {@link
 *     MountCheck}). Above what a calibrated camera shows while the robot drives, mostly the
 *     estimate's own error.
 * @param latencyMarginSeconds an image is thrown out when it reaches the robot more than this after
 *     its camera's usual latency ({@code CameraConfig.Sensor.latency}), or before it was taken: the
 *     camera's clock and the robot's are out of step (PhotonVision's time sync), or the coprocessor
 *     is falling behind. It's judged against the loop before the one that read the image, so the
 *     robot's own wait, or a loop that ran long, doesn't count. What it can't catch: a clock off by
 *     less than the margin, whose images are applied that much too early or late (at 3 m/s, 30 cm
 *     for 100 ms).
 */
@ConfigRecord
public record VisionConfig(
    double maxAmbiguity,
    double maxHeightErrorMeters,
    double maxDistanceMeters,
    double maxYawRateRadiansPerSecond,
    double linearStdDevBaseline,
    double angularStdDevBaseline,
    double distanceExponent,
    double headingConstrainedFactor,
    double speedStdDevFactor,
    double yawRateStdDevFactor,
    double maxHeadingStdDevToConstrain,
    int minTagsForHeading,
    double maxYawRateForHeading,
    double maxReprojectionErrorForHeadingPixels,
    double mountToleranceMeters,
    double latencyMarginSeconds) {
  /** Starting points: a team's 2026 values where it had them, top teams' where it didn't. */
  public static final VisionConfig DEFAULTS =
      builder()
          .maxAmbiguity(0.2)
          .maxHeightErrorMeters(0.2)
          .maxDistanceMeters(6.0)
          .maxYawRateRadiansPerSecond(5.0)
          .linearStdDevBaseline(0.02)
          .angularStdDevBaseline(0.03)
          .distanceExponent(2.0)
          .headingConstrainedFactor(0.5)
          .speedStdDevFactor(0.25)
          .yawRateStdDevFactor(0.5)
          .maxHeadingStdDevToConstrain(Math.toRadians(5))
          .minTagsForHeading(2)
          .maxYawRateForHeading(0.5)
          .maxReprojectionErrorForHeadingPixels(1.0)
          .mountToleranceMeters(0.05)
          .latencyMarginSeconds(0.1)
          .build();

  public VisionConfig {
    Checks.on(VisionConfig.class)
        .field("maxAmbiguity", maxAmbiguity, positive())
        .field("maxHeightErrorMeters", maxHeightErrorMeters, positive())
        .field("maxDistanceMeters", maxDistanceMeters, positive())
        .field("maxYawRateRadiansPerSecond", maxYawRateRadiansPerSecond, positive())
        .field("linearStdDevBaseline", linearStdDevBaseline, positive())
        .field("angularStdDevBaseline", angularStdDevBaseline, positive())
        .field("distanceExponent", distanceExponent, positive())
        .field("headingConstrainedFactor", headingConstrainedFactor, positive())
        .field("speedStdDevFactor", speedStdDevFactor, nonNegative())
        .field("yawRateStdDevFactor", yawRateStdDevFactor, nonNegative())
        .field("maxHeadingStdDevToConstrain", maxHeadingStdDevToConstrain, positive())
        .field("minTagsForHeading", minTagsForHeading, atLeast(2)) // one tag can't fix heading
        .field("maxYawRateForHeading", maxYawRateForHeading, positive())
        .field(
            "maxReprojectionErrorForHeadingPixels",
            maxReprojectionErrorForHeadingPixels,
            positive())
        .field("mountToleranceMeters", mountToleranceMeters, positive())
        .field("latencyMarginSeconds", latencyMarginSeconds, positive())
        .orThrow();
  }

  /** Starts building vision settings, one named value at a time. */
  public static VisionConfigBuilder.MaxAmbiguityStage builder() {
    return VisionConfigBuilder.builder();
  }
}
