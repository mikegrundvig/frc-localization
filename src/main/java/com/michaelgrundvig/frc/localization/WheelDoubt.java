package com.michaelgrundvig.frc.localization;

/**
 * How far the wheels' travel is doubted, one odometry step at a time. The {@link PoseEstimator}
 * asks it two things of each step: how much of the wheels' travel counts ({@link
 * #shareOfTravelThatCounts}), and how much position variance the step adds for everything that
 * makes the wheels suspect ({@link #addedVariance}).
 *
 * <p>What makes them suspect comes from the {@link DriveConditions} the robot's other sensors
 * report, and from the wheels themselves:
 *
 * <ul>
 *   <li>A tilt the ground does not explain may have lifted the wheels, so part of their travel is
 *       left out, and that part is suspect.
 *   <li>The wheels' disagreement with each other (the kinematics' residual) and the slide the IMU
 *       sees are suspect travel too.
 *   <li>A collision may have moved the robot without its wheels seeing it at all.
 *   <li>A disabled robot may be picked up and carried.
 * </ul>
 *
 * <p>Suspect travel is carried from step to step and added up before it is squared, because slip
 * keeps its direction for a while: ten steps that each slid a centimeter the same way are ten
 * centimeters off, not the three that ten independent centimeters would add up to. What is carried
 * fades over {@link EstimatorConfig#slipCorrelationSeconds}.
 *
 * <p>It runs at the odometry rate, so it holds plain numbers and allocates nothing.
 */
final class WheelDoubt {
  private final double tiltStartRadians;
  private final double tiltZeroRadians;
  private final double residualScale;
  private final double slipCorrelationSeconds;
  private final double collisionVariance;
  private final long collisionWindowNanos;
  private final double handlingVariancePerSecond;

  private DriveConditions conditions = DriveConditions.NORMAL;

  /** The suspect travel carried from the steps before, in meters. */
  private double suspectTravelMeters;

  /** The variance of collisions felt since the last step, which the next step takes. */
  private double collisionVarianceForNextStep;

  /** Until when further jolts count as the same collision. */
  private long sameCollisionUntilNanos = Long.MIN_VALUE;

  WheelDoubt(EstimatorConfig config) {
    tiltStartRadians = Math.toRadians(config.tiltStartDegrees());
    tiltZeroRadians = Math.toRadians(config.tiltZeroDegrees());
    residualScale = config.residualScale();
    slipCorrelationSeconds = config.slipCorrelationSeconds();
    collisionVariance = square(config.collisionStdDevMeters());
    collisionWindowNanos = Timestamps.fromSeconds(config.collisionWindowSeconds());
    handlingVariancePerSecond = square(config.handlingStdDevPerSqrtSecond());
  }

  /** The conditions the next steps are taken under, before odometry has started. */
  void conditionsBeforeOdometry(DriveConditions conditions) {
    this.conditions = conditions;
  }

  /**
   * The conditions the next steps are taken under, and any collision they report.
   *
   * <p>A whole collision outside the window of the last one is a new collision, and adds a
   * collision's variance to the next step. Inside the window it is the same collision still shaking
   * the robot, and only keeps the window open. A share of a collision (a jolt the wheels partly
   * explain) adds that share of a collision's spread, so its square of the variance, and opens no
   * window. Inside a window it adds nothing: the collision's doubt already covers it.
   *
   * @param newestStepNanos when the latest odometry step ended
   */
  void conditions(DriveConditions conditions, long newestStepNanos) {
    this.conditions = conditions;
    boolean sameCollision = newestStepNanos <= sameCollisionUntilNanos;
    if (conditions.collision()) {
      if (!sameCollision) {
        collisionVarianceForNextStep += collisionVariance;
      }
      sameCollisionUntilNanos = newestStepNanos + collisionWindowNanos;
      return;
    }
    if (!sameCollision) {
      collisionVarianceForNextStep += square(conditions.collisionShare()) * collisionVariance;
    }
  }

  DriveConditions conditions() {
    return conditions;
  }

  double suspectTravelMeters() {
    return suspectTravelMeters;
  }

  /** Whether a collision is still shaking the robot as of a moment. */
  boolean inCollisionAt(long nanos) {
    return nanos <= sameCollisionUntilNanos;
  }

  /** Forgets the suspect travel carried so far: the estimate was just told where the robot is. */
  void forgetSuspectTravel() {
    suspectTravelMeters = 0;
  }

  /**
   * How much of the wheels' travel counts, from all of it (1) to none (0). A tilt the ground
   * explains takes nothing off. Tilt beyond that may have lifted the wheels: all of their travel
   * counts up to the start angle, less from there on, and none from the zero angle.
   */
  double shareOfTravelThatCounts() {
    double tilt = conditions.unexplainedTiltRadians();
    double shareLeftOut =
        Math.clamp((tilt - tiltStartRadians) / (tiltZeroRadians - tiltStartRadians), 0.0, 1.0);
    return 1 - shareLeftOut;
  }

  /**
   * How much of the wheels' travel is travel across the field. On a slope the ground explains, the
   * wheels still roll on the ground, and what they roll along the slope covers its cosine across
   * the field.
   */
  double shareOfTravelAcrossTheField() {
    return Math.cos(conditions.groundSlopeRadians());
  }

  /**
   * The position variance a step adds beyond what its distance and turning add. Call it once for
   * each step, in order: it carries the suspect travel forward and hands the step any collision's
   * variance that was waiting.
   *
   * @param seconds how long the step took
   * @param distanceMeters how far the wheels say the robot went over it
   * @param shareThatCounts how much of that counts ({@link #shareOfTravelThatCounts}): the rest is
   *     suspect
   * @param residualMeters how far the wheels' disagreement may have put the step off
   */
  double addedVariance(
      double seconds, double distanceMeters, double shareThatCounts, double residualMeters) {
    double carried = suspectTravelMeters * Math.exp(-seconds / slipCorrelationSeconds);
    suspectTravelMeters =
        carried
            + (1 - shareThatCounts) * distanceMeters
            + residualScale * residualMeters
            + conditions.slipMetersPerSecond() * seconds;
    // The variance of the suspect travel so far, less what the steps before already added of it.
    double variance = square(suspectTravelMeters) - square(carried);
    if (conditions.disabled()) {
      variance += handlingVariancePerSecond * seconds;
    }
    variance += collisionVarianceForNextStep;
    collisionVarianceForNextStep = 0;
    return variance;
  }

  private static double square(double value) {
    return value * value;
  }
}
