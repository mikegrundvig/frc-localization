package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Compares the wheels with the IMU. It catches the wheels sliding over the floor together, which
 * their disagreement with each other (the kinematic residual) cannot show, and it tells a collision
 * from hard driving. It makes three judgments each loop:
 *
 * <ul>
 *   <li><b>The turn</b> ({@link TurnComparison}). Over a short window, the gyro and the wheels
 *       should turn the robot alike. What they differ by beyond a tolerance is how far the wheels
 *       slid around the robot's center.
 *   <li><b>The speed.</b> The wheels' velocity, carried to where the IMU sits, should change as the
 *       accelerometer feels it change. What the accelerometer does not feel, added up loop by loop,
 *       is how much faster or slower than the robot the wheels roll: wheels spinning in a launch,
 *       or a push they do not see ({@link #compareSpeeds}). The accelerometer's bias is learned
 *       while the robot stands still and taken off first ({@link AccelerometerBias}).
 *   <li><b>A jolt.</b> What the accelerometer feels that the wheels' own change of velocity does
 *       not explain is a collision, as far as it is past what the tread's grip could explain
 *       ({@link #collisionShare(boolean, boolean, double)}).
 * </ul>
 *
 * <p>The turn and the speed each say how fast the wheels may be sliding over the floor. Together
 * they give the root of the sum of their squares, as a slide's turn at the wheels and its push add.
 * The doubt this adds to odometry is the distance the wheels may have slid, so it grows with the
 * evidence.
 *
 * <p><b>What it cannot see.</b> A slide that neither turns the robot nor changes its speed (wheels
 * spinning at a steady speed, wheels in the air) shows in none of the three: only the cameras find
 * that. Nor does it ask whether the drive motors are at their current limit. That says the motors
 * push as hard as they may, not that the wheels slip, and a limit set below the tread's grip keeps
 * them from slipping.
 *
 * <p><b>An accelerometer that has not been checked.</b> Until an IMU's accelerometer is known to
 * read along the robot's axes ({@link ImuMount#accelerometerChecked}), its direction cannot be
 * trusted. The speed is then compared by size instead of direction ({@link #compareSizes}), and
 * only once its bias has been learned, and a jolt is judged by its size alone.
 */
public final class SlipCheck {
  /**
   * How closely the wheels must agree with the IMU.
   *
   * @param windowSeconds how far back the turn's comparison looks: long enough for the gyro's noise
   *     to average out, short enough that a slide shows while it happens
   * @param turnToleranceDegrees how far apart the wheels and the gyro may turn over the window,
   *     however little it turned: the gyro's noise, and wheels worn unevenly, which turn the
   *     wheels' reckoning as the robot drives straight
   * @param turnScaleTolerance and this share of the turning on top: worn tread turns the wheels'
   *     reckoning a little further than the robot, the gyro's scale a little less
   * @param speedBiasMetersPerSecSq the most the accelerometer's bias may be: what it didn't feel is
   *     let go at this rate and no faster, so a bias up to it never builds up, and a slide stays
   *     seen until it stops
   * @param speedToleranceMetersPerSecond how far the wheels' speed and the accelerometer's
   *     reckoning of the robot's may drift apart without a slide: its noise, and its lag as the
   *     robot's acceleration changes. Past it, the whole of it is the slide's speed.
   * @param stillMetersPerSecond the robot is standing still when its wheels roll slower than this,
   *     and turn it slower than this at their reach, and the IMU turns it slower than this too
   * @param steadySeconds standing still, a reading the accelerometer holds steady this long is its
   *     bias, whatever its size, and its average over the older half of it is taken for it: longer
   *     than a push keeps the robot speeding up past wheels that don't roll, and long enough to
   *     average out the noise
   * @param steadyMetersPerSecond the reading holds steady when the speed it adds up to over that
   *     stretch strays no further than this from what its average adds up to: further, it rose or
   *     fell (a push building, or come and gone), and nothing is learned
   * @param largestBiasMetersPerSecSq the largest bias an accelerometer has: one learned past this
   *     is gravity's share, the IMU turned on the robot otherwise than its definition says
   * @param maxSlideMetersPerSecond the fastest the wheels and the robot can be sliding apart: past
   *     it, what the speed comparison holds is no slide (a bias it hasn't learned, adding up), and
   *     it counts no faster
   * @param joltMetersPerSecSq what the IMU feels and the wheels don't explain, past this, is a
   *     whole collision; past the tread's grip and short of this, a share of one. Past a grip at or
   *     above it, a whole one
   * @param accelerometerRangeMetersPerSecSq the most the accelerometer reports on each axis: a
   *     reading at it is a jolt past what it can say, whatever the wheels did
   */
  @ConfigRecord
  public record Settings(
      double windowSeconds,
      double turnToleranceDegrees,
      double turnScaleTolerance,
      double speedBiasMetersPerSecSq,
      double speedToleranceMetersPerSecond,
      double stillMetersPerSecond,
      double steadySeconds,
      double steadyMetersPerSecond,
      double largestBiasMetersPerSecSq,
      double maxSlideMetersPerSecond,
      double joltMetersPerSecSq,
      double accelerometerRangeMetersPerSecSq) {
    /**
     * The turn over a quarter second, within half a degree and 5% of it; the speed let go at 0.15
     * m/s² and within 0.3 m/s, and no slide past 5 m/s; a jolt past 15 m/s², under a Pigeon 2's ±2
     * g range.
     *
     * <p>The accelerometer's bias is a reading held for 2 s standing still, the speed it adds up to
     * keeping within 0.1 m/s of a steady one. A push sets the robot going in well under a second
     * (the simulator's at 3 m/s²) and then holds it at a speed, which reads as no acceleration, so
     * the bias is learned true; one that kept it speeding up steadily for 2 s, at even 0.5 m/s²,
     * would have it sliding 1 m/s past wheels that don't roll. The simulator's 0.3 m/s² of
     * vibration on each reading adds up to a speed about 0.06 m/s off the line, past 0.1 in one
     * window in 40, which only waits a loop to learn; a push building at 1 m/s³ strays past it half
     * a second in, still all in the newer half. One building slower than 0.2 m/s³ passes for
     * steady, and the bias follows it, 1.5 s behind. And 2 s learns a checked accelerometer's bias
     * of up to twice what the comparison lets go of (0.3 m/s²) before it holds the tolerance's
     * worth of it. Past 1 m/s², a bias is gravity: a Pigeon 2's is a tenth of that, and a case
     * tilted 6° on its mount leaks 1 m/s².
     */
    public static final Settings DEFAULTS =
        builder()
            .windowSeconds(0.25)
            .turnToleranceDegrees(0.5)
            .turnScaleTolerance(0.05)
            .speedBiasMetersPerSecSq(0.15)
            .speedToleranceMetersPerSecond(0.3)
            .stillMetersPerSecond(0.05)
            .steadySeconds(2)
            .steadyMetersPerSecond(0.1)
            .largestBiasMetersPerSecSq(1)
            .maxSlideMetersPerSecond(5)
            .joltMetersPerSecSq(15)
            .accelerometerRangeMetersPerSecSq(2 * ImuReadings.STANDARD_GRAVITY)
            .build();

    public Settings {
      Checks.on(Settings.class)
          .field("windowSeconds", windowSeconds, positive())
          .field("turnToleranceDegrees", turnToleranceDegrees, nonNegative())
          .field("turnScaleTolerance", turnScaleTolerance, nonNegative())
          .field("speedBiasMetersPerSecSq", speedBiasMetersPerSecSq, nonNegative())
          .field("speedToleranceMetersPerSecond", speedToleranceMetersPerSecond, nonNegative())
          .field("stillMetersPerSecond", stillMetersPerSecond, positive())
          .field("steadySeconds", steadySeconds, positive())
          .field("steadyMetersPerSecond", steadyMetersPerSecond, positive())
          .field("largestBiasMetersPerSecSq", largestBiasMetersPerSecSq, positive())
          .field("maxSlideMetersPerSecond", maxSlideMetersPerSecond, positive())
          .field("joltMetersPerSecSq", joltMetersPerSecSq, positive())
          .field("accelerometerRangeMetersPerSecSq", accelerometerRangeMetersPerSecSq, positive())
          .check(
              "joltMetersPerSecSq",
              () ->
                  joltMetersPerSecSq < accelerometerRangeMetersPerSecSq
                      ? null
                      : joltMetersPerSecSq
                          + " must be under the accelerometer's range ("
                          + accelerometerRangeMetersPerSecSq
                          + "), or a jolt along one axis can never pass it")
          .orThrow();
    }

    /** Starts building the settings, one named value at a time. */
    public static SlipCheckSettingsBuilder.WindowSecondsStage builder() {
      return SlipCheckSettingsBuilder.builder();
    }
  }

  /**
   * One loop of what the wheels and one IMU read. A part not a number starts its comparison over.
   *
   * @param seconds how long the loop was: since the last sample's wheel reading
   * @param wheelTurnRadians how far the wheels turned the robot over the loop
   * @param imuTurnRadians how far the IMU turned it over the same readings; not a number when this
   *     IMU didn't read both ends of them
   * @param yawRadians the IMU's yaw at the loop's end, which holds still while the robot turns: the
   *     frame the speed is compared in. Not a number without an IMU.
   * @param yawRateRadiansPerSecond how fast the IMU turns at the loop's end, counterclockwise; not
   *     a number if it can't say, and the wheels' turn rate stands in
   * @param imuPosition where the IMU sits, from the robot's origin (its center, on the floor)
   * @param accelerometerChecked whether this IMU's accelerometer is known to read along the robot's
   *     axes ({@link ImuMount#accelerometerChecked}): without it, speeds are compared by size, not
   *     direction, and a jolt is judged by its size alone
   * @param wheels the robot's velocity as its wheels measure it, robot-relative, at the loop's end
   * @param forwardAccelerationMetersPerSecSq the acceleration the IMU felt over the loop along the
   *     robot's forward axis, without gravity's share; not a number when it can't say
   * @param leftAccelerationMetersPerSecSq the same, along the robot's left axis
   * @param tiltRadians how far the robot is tilted from level, by the IMU's attitude: an
   *     accelerometer not yet checked may have gravity's share as the robot tilts taken off the
   *     wrong axis. Not a number, it's taken as level.
   */
  public record Sample(
      double seconds,
      double wheelTurnRadians,
      double imuTurnRadians,
      double yawRadians,
      double yawRateRadiansPerSecond,
      Translation3d imuPosition,
      boolean accelerometerChecked,
      ChassisVelocities wheels,
      double forwardAccelerationMetersPerSecSq,
      double leftAccelerationMetersPerSecSq,
      double tiltRadians) {
    public Sample {
      if (!(seconds > 0) || Double.isInfinite(seconds)) {
        throw new IllegalArgumentException("a loop takes time: " + seconds);
      }
    }
  }

  /**
   * A reading this close to the accelerometer's range counts as being at it: the share of the range
   * a reading must reach.
   */
  private static final double SATURATED_SHARE_OF_RANGE = 0.99;

  /** How the wheels' speed can be compared with the accelerometer this loop. */
  private enum SpeedComparison {
    /** Nothing can be compared, and what was added up is forgotten. */
    START_OVER,
    /** Nothing can be compared this loop, and what was added up is kept. */
    NOT_THIS_LOOP,
    /** The accelerometer reads along the robot's axes: the two are compared as vectors. */
    BY_DIRECTION,
    /** The accelerometer's direction is not known, but its bias is: their sizes are compared. */
    BY_SIZE
  }

  private final double windowSeconds;
  private final double speedBiasMetersPerSecSq;
  private final double speedToleranceMetersPerSecond;
  private final double largestBiasMetersPerSecSq;
  private final double maxSlideMetersPerSecond;

  /** The most the tread's grip can change the robot's velocity, which the floor could explain. */
  private final double gripMetersPerSecSq;

  /**
   * An unexplained acceleration past this is a whole collision: the jolt's limit from the settings,
   * or the tread's grip where that is more.
   */
  private final double wholeCollisionMetersPerSecSq;

  /** A reading at or past this on either axis is at the accelerometer's range. */
  private final double saturationMetersPerSecSq;

  private final TurnComparison turns;
  private final AccelerometerBias bias;

  /**
   * The wheels' velocity carried to the IMU, at the last loop's end, on the field as the IMU's yaw
   * has it. Not a number until a loop has given one.
   */
  private double lastVelocityX = Double.NaN;

  private double lastVelocityY = Double.NaN;

  /**
   * With a checked accelerometer: how much faster than the robot the wheels roll, as the
   * accelerometer reckons it, in the same frame, in meters per second.
   */
  private double slideX;

  private double slideY;

  /**
   * With an accelerometer not checked: how far the wheels' velocity has changed past what the
   * accelerometer felt ("ahead"), and how far what it felt has changed past the wheels ("moved").
   * After each loop one of the two is zero.
   */
  private double aheadX;

  private double aheadY;
  private double movedX;
  private double movedY;

  private double speedSlipMetersPerSecond;
  private double collisionShare;
  private double unexplainedMetersPerSecSq;

  /** How long since the wheels were last seen sliding, in seconds. */
  private double sinceSlideSeconds = Double.POSITIVE_INFINITY;

  /**
   * @param settings how closely the wheels must agree with the IMU
   * @param wheelReachMeters how far the wheels sit from the robot's center: a turn they see and the
   *     gyro doesn't, times this, is how far they slid
   * @param gripMetersPerSecSq the most the tread's grip can change the robot's velocity: its
   *     friction times g. Felt by the IMU and unexplained by the wheels, an acceleration up to it
   *     may be the floor bringing the robot to its wheels' speed, and is no collision. At {@link
   *     Settings#joltMetersPerSecSq} or past it, a collision is only what's past it.
   */
  public SlipCheck(Settings settings, double wheelReachMeters, double gripMetersPerSecSq) {
    if (!(wheelReachMeters > 0) || Double.isInfinite(wheelReachMeters)) {
      throw new IllegalArgumentException("the wheels' reach must be positive: " + wheelReachMeters);
    }
    if (!(gripMetersPerSecSq > 0) || Double.isInfinite(gripMetersPerSecSq)) {
      throw new IllegalArgumentException(
          "the tread's grip must be positive: " + gripMetersPerSecSq);
    }
    windowSeconds = settings.windowSeconds();
    speedBiasMetersPerSecSq = settings.speedBiasMetersPerSecSq();
    speedToleranceMetersPerSecond = settings.speedToleranceMetersPerSecond();
    largestBiasMetersPerSecSq = settings.largestBiasMetersPerSecSq();
    maxSlideMetersPerSecond = settings.maxSlideMetersPerSecond();
    this.gripMetersPerSecSq = gripMetersPerSecSq;
    wholeCollisionMetersPerSecSq = Math.max(settings.joltMetersPerSecSq(), gripMetersPerSecSq);
    saturationMetersPerSecSq =
        SATURATED_SHARE_OF_RANGE * settings.accelerometerRangeMetersPerSecSq();
    turns = new TurnComparison(settings, wheelReachMeters);
    bias = new AccelerometerBias(settings, wheelReachMeters);
  }

  /** One loop's readings. */
  public void update(Sample sample) {
    turns.add(sample.seconds(), sample.wheelTurnRadians(), sample.imuTurnRadians());
    compareSpeeds(sample);
  }

  /**
   * A loop without a sample to compare: no wheel readings, or no IMU. Everything added up so far is
   * forgotten, and a collision is judged by the size of what an IMU felt alone.
   *
   * @param forwardAccelerationMetersPerSecSq what an IMU felt along the robot's forward axis,
   *     without gravity's share, or not a number without one
   * @param leftAccelerationMetersPerSecSq the same, along its left axis
   */
  public void startOver(
      double forwardAccelerationMetersPerSecSq, double leftAccelerationMetersPerSecSq) {
    turns.startOver();
    forgetSpeeds();
    lastVelocityX = Double.NaN;
    lastVelocityY = Double.NaN;
    bias.forget();
    sinceSlideSeconds = Double.POSITIVE_INFINITY;
    boolean joltBySize =
        Math.hypot(forwardAccelerationMetersPerSecSq, leftAccelerationMetersPerSecSq)
                > wholeCollisionMetersPerSecSq
            || saturated(forwardAccelerationMetersPerSecSq, leftAccelerationMetersPerSecSq);
    collisionShare = joltBySize ? 1 : 0;
  }

  // ---------------------------------------------------------------------------------------------
  // What it found.
  // ---------------------------------------------------------------------------------------------

  /**
   * How large the accelerometer's bias is, as learned standing still, in m/s²: zero until it's
   * learned.
   */
  public double biasMetersPerSecSq() {
    return Math.hypot(bias.forward(), bias.left());
  }

  /**
   * Whether the accelerometer's bias, learned standing still, is past the largest a part has
   * ({@link Settings#largestBiasMetersPerSecSq}): gravity's share, leaking in because the IMU is
   * turned on the robot otherwise than its definition says, a case tilted on its mount.
   */
  public boolean leaksGravity() {
    return biasMetersPerSecSq() > largestBiasMetersPerSecSq;
  }

  /**
   * How far the wheels turned the robot beyond the gyro over the window, in radians, either way:
   * before the tolerances, for tuning them.
   */
  public double turnApartRadians() {
    return turns.apartRadians();
  }

  /**
   * How much faster or slower than the robot the wheels roll, as the accelerometer reckons it, in
   * meters per second: before the tolerance, for tuning it.
   */
  public double slideMetersPerSecond() {
    return Math.hypot(slideX, slideY) + Math.hypot(aheadX, aheadY) + Math.hypot(movedX, movedY);
  }

  /** How fast the wheels may be sliding, as the gyro sees them turning, in meters per second. */
  public double turnSlipMetersPerSecond() {
    return turns.slipMetersPerSecond();
  }

  /**
   * How fast the wheels may be sliding, as the accelerometer sees their speed, in meters per
   * second.
   */
  public double speedSlipMetersPerSecond() {
    return speedSlipMetersPerSecond;
  }

  /** How fast the wheels may be sliding over the floor, in meters per second: zero if they grip. */
  public double slipMetersPerSecond() {
    return Math.hypot(turns.slipMetersPerSecond(), speedSlipMetersPerSecond);
  }

  /**
   * What the last loop's IMU felt that the wheels' own change of velocity doesn't explain, in m/s²:
   * zero when they explain it, or it felt less than they changed, or its accelerometer isn't
   * checked. For tuning the collision's limits.
   */
  public double unexplainedMetersPerSecSq() {
    return unexplainedMetersPerSecSq;
  }

  /**
   * How much of a collision the last loop's IMU felt, from none to a whole one: a jolt the wheels
   * don't explain past the limit, or one at the accelerometer's range, is a whole one; a jolt past
   * the limit the wheels partly explain, what they don't against the limit.
   */
  public double collisionShare() {
    return collisionShare;
  }

  // ---------------------------------------------------------------------------------------------
  // The speed and the jolt.
  // ---------------------------------------------------------------------------------------------

  /**
   * Compares how the wheels' velocity changed over the loop with what the accelerometer felt, and
   * judges whether the IMU felt a collision.
   *
   * <p>Both are taken where the IMU sits and in a frame that does not turn with the robot (the
   * field, as the IMU's yaw has it), so that a robot turning on the spot, whose forward axis swings
   * round, does not look as if its velocity changed.
   */
  private void compareSpeeds(Sample sample) {
    double yaw = sample.yawRadians();
    ChassisVelocities wheels = sample.wheels();
    if (!Double.isFinite(yaw) || !Double.isFinite(wheels.vx) || !Double.isFinite(wheels.vy)) {
      startOver(
          sample.forwardAccelerationMetersPerSecSq(), sample.leftAccelerationMetersPerSecSq());
      return;
    }
    bias.learnFrom(sample);
    double readForward = sample.forwardAccelerationMetersPerSecSq();
    double readLeft = sample.leftAccelerationMetersPerSecSq();
    double feltForward = readForward - bias.forward();
    double feltLeft = readLeft - bias.left();

    // The wheels' velocity where the IMU sits: the robot's, plus its turning swinging the IMU
    // around the center, which is the turn rate crossed with the IMU's position. Only the yaw rate
    // is read, which swings the IMU level whatever its height. (A robot pitching or rolling would
    // swing a high IMU too, and that is not allowed for.) A gyro that cannot say how fast it turns
    // leaves the wheels' own turn rate to stand in.
    double yawRate =
        Double.isFinite(sample.yawRateRadiansPerSecond())
            ? sample.yawRateRadiansPerSecond()
            : wheels.omega;
    Translation3d imuPosition = sample.imuPosition();
    double atImuForward = wheels.vx - yawRate * imuPosition.getY();
    double atImuLeft = wheels.vy + yawRate * imuPosition.getX();
    // Both turned from the robot's axes onto the field by the IMU's yaw.
    double cos = Math.cos(yaw);
    double sin = Math.sin(yaw);
    double velocityX = cos * atImuForward - sin * atImuLeft;
    double velocityY = sin * atImuForward + cos * atImuLeft;
    double feltX = cos * feltForward - sin * feltLeft;
    double feltY = sin * feltForward + cos * feltLeft;
    double seconds = sample.seconds();

    boolean haveLastVelocity = Double.isFinite(lastVelocityX);
    boolean feltIsANumber = Double.isFinite(feltX) && Double.isFinite(feltY);
    boolean checked = sample.accelerometerChecked();
    // The range is the part's own, so it is what the accelerometer read, bias and all, that
    // reaches it.
    boolean saturated = saturated(readForward, readLeft);

    unexplainedMetersPerSecSq = 0;
    if (checked && haveLastVelocity && feltIsANumber) {
      double wheelsAccelerationX = (velocityX - lastVelocityX) / seconds;
      double wheelsAccelerationY = (velocityY - lastVelocityY) / seconds;
      unexplainedMetersPerSecSq =
          unexplainedByTheWheels(feltX, feltY, wheelsAccelerationX, wheelsAccelerationY);
    }
    boolean wheelsCanExplain = checked && haveLastVelocity;
    collisionShare = collisionShare(saturated, wheelsCanExplain, Math.hypot(feltForward, feltLeft));

    switch (howSpeedsCompare(feltIsANumber, haveLastVelocity, checked)) {
      case START_OVER -> forgetSpeeds();
      case NOT_THIS_LOOP -> {}
      case BY_DIRECTION -> {
        // What the wheels' velocity changed by over the loop, less what the accelerometer felt it
        // change by, is added to the slide...
        slideX += velocityX - lastVelocityX - feltX * seconds;
        slideY += velocityY - lastVelocityY - feltY * seconds;
        // ...which is then let go toward none, by the most a bias could have added and no more.
        double size = Math.hypot(slideX, slideY);
        double kept = shareKept(size, speedBiasMetersPerSecSq * seconds);
        slideX *= kept;
        slideY *= kept;
        speedSlipMetersPerSecond = slipFrom(size * kept);
      }
      case BY_SIZE ->
          // Gravity's share as the robot tilts may be taken off the wrong axis of a mount that has
          // not been checked, by at most twice that share, so that much more is let go of.
          compareSizes(
              velocityX - lastVelocityX,
              velocityY - lastVelocityY,
              feltX * seconds,
              feltY * seconds,
              (speedBiasMetersPerSecSq
                      + 2 * ImuReadings.STANDARD_GRAVITY * Math.sin(tiltOrLevel(sample)))
                  * seconds);
    }
    lastVelocityX = velocityX;
    lastVelocityY = velocityY;
    boolean sliding = speedSlipMetersPerSecond > 0 || turns.slipMetersPerSecond() > 0;
    sinceSlideSeconds = sliding ? 0 : sinceSlideSeconds + seconds;
  }

  /**
   * How the wheels' speed can be compared with the accelerometer this loop.
   *
   * <p>After a whole collision the comparison starts over: the collision's doubt covers whatever
   * slide it began. A share of a collision does not start it over, so a slide it began is still
   * seen. An accelerometer that has not been checked is compared only once its bias is learned:
   * until then, gravity leaking into one tilted on its mount reads as the robot speeding up
   * steadily, and would add up as a slide for as long as it ran.
   */
  private SpeedComparison howSpeedsCompare(
      boolean feltIsANumber, boolean haveLastVelocity, boolean checked) {
    if (collisionShare >= 1 || !feltIsANumber) {
      return SpeedComparison.START_OVER;
    }
    if (!haveLastVelocity) {
      return SpeedComparison.NOT_THIS_LOOP;
    }
    if (checked) {
      return SpeedComparison.BY_DIRECTION;
    }
    if (bias.learned()) {
      return SpeedComparison.BY_SIZE;
    }
    return SpeedComparison.NOT_THIS_LOOP;
  }

  /**
   * What the IMU felt that the wheels' own change of velocity does not explain, in m/s². A robot
   * stopping hard is felt by both. A hit is felt only by the IMU.
   *
   * @param feltX what the accelerometer felt, on the field
   * @param wheelsAccelerationX how fast the wheels' velocity changed over the loop, on the field
   */
  private double unexplainedByTheWheels(
      double feltX, double feltY, double wheelsAccelerationX, double wheelsAccelerationY) {
    boolean slidingOrJustDone = sinceSlideSeconds < windowSeconds;
    if (slidingOrJustDone) {
      // While the wheels slide, and for a moment after (what the IMU feels of the robot catching
      // up lags a little), their change of velocity says nothing of the robot's. All that was felt
      // is unexplained.
      return Math.hypot(feltX, feltY);
    }
    boolean feltAtLeastAsMuchAsTheWheelsChanged =
        Math.hypot(feltX, feltY) >= Math.hypot(wheelsAccelerationX, wheelsAccelerationY);
    if (feltAtLeastAsMuchAsTheWheelsChanged) {
      return Math.hypot(feltX - wheelsAccelerationX, feltY - wheelsAccelerationY);
    }
    // The IMU felt less than the wheels changed: the wheels are changing faster than the robot can
    // follow. That is a slide, not a hit.
    return 0;
  }

  /**
   * How much of a collision the IMU felt this loop, from none (0) to a whole one (1).
   *
   * <ul>
   *   <li>A reading at the accelerometer's range is a whole one, whatever the wheels did: the jolt
   *       was past what it can say.
   *   <li>When the wheels cannot explain anything (the accelerometer is not checked, or there is no
   *       last loop to compare with), the size of what was felt decides alone: past a whole
   *       collision's, it is one.
   *   <li>Otherwise it is what the wheels do not explain that counts. Up to the tread's grip, the
   *       floor can explain it (the robot catching up with wheels that got ahead of it), and it is
   *       no collision. From the grip to a whole collision's, it is a share of one. Treads said to
   *       grip as hard as a whole collision leave no share between: past the grip is a whole one.
   * </ul>
   *
   * @param wheelsCanExplain whether what the wheels do not explain has been worked out
   * @param feltMetersPerSecSq the size of what the accelerometer felt, its bias taken off
   */
  private double collisionShare(
      boolean saturated, boolean wheelsCanExplain, double feltMetersPerSecSq) {
    if (saturated) {
      return 1;
    }
    if (!wheelsCanExplain) {
      return feltMetersPerSecSq > wholeCollisionMetersPerSecSq ? 1 : 0;
    }
    boolean aShareIsPossible = wholeCollisionMetersPerSecSq > gripMetersPerSecSq;
    if (aShareIsPossible) {
      return Math.clamp(
          (unexplainedMetersPerSecSq - gripMetersPerSecSq)
              / (wholeCollisionMetersPerSecSq - gripMetersPerSecSq),
          0.0,
          1.0);
    }
    return unexplainedMetersPerSecSq > gripMetersPerSecSq ? 1 : 0;
  }

  /** Whether the accelerometer read at its range on either axis: past what it can say. */
  private boolean saturated(double forward, double left) {
    return Math.abs(forward) >= saturationMetersPerSecSq
        || Math.abs(left) >= saturationMetersPerSecSq;
  }

  private void forgetSpeeds() {
    slideX = 0;
    slideY = 0;
    aheadX = 0;
    aheadY = 0;
    movedX = 0;
    movedY = 0;
    speedSlipMetersPerSecond = 0;
  }

  /** The robot's tilt from level, or level if the IMU can't say. */
  private static double tiltOrLevel(Sample sample) {
    return Double.isFinite(sample.tiltRadians()) ? Math.abs(sample.tiltRadians()) : 0;
  }

  /**
   * Compares the wheels with an accelerometer that is not known to read along the robot's axes, by
   * size instead of direction.
   *
   * <p>What such an accelerometer feels, turned on its mount, is the robot's every change of
   * velocity turned the same way. So the wheels' changes of velocity and what it felt, each added
   * up on its own, add up to the same size whichever way it is turned. What both have added up
   * alike is driving, and is let go. What one holds past the other is a slide: the wheels getting
   * ahead of the robot, or the robot moved without them. A slide is at least that large.
   *
   * @param wheelsX how the wheels' velocity changed over the loop, on the field
   * @param wheelsY and along y
   * @param feltX what the accelerometer felt over the loop, as a change of velocity on the field
   * @param feltY and along y
   * @param letGo how much of a slide to let go of this loop, in meters per second
   */
  private void compareSizes(
      double wheelsX, double wheelsY, double feltX, double feltY, double letGo) {
    aheadX += wheelsX;
    aheadY += wheelsY;
    movedX += feltX;
    movedY += feltY;
    double ahead = Math.hypot(aheadX, aheadY);
    double moved = Math.hypot(movedX, movedY);
    // Take what the two have alike off both. One of them is then zero, and the other is the slide.
    double alike = Math.min(ahead, moved);
    double shareOfAheadLeft = ahead > 0 ? (ahead - alike) / ahead : 0;
    double shareOfMovedLeft = moved > 0 ? (moved - alike) / moved : 0;
    aheadX *= shareOfAheadLeft;
    aheadY *= shareOfAheadLeft;
    movedX *= shareOfMovedLeft;
    movedY *= shareOfMovedLeft;
    double aheadSize = ahead - alike;
    double movedSize = moved - alike;
    double shareOfAheadKept = shareKept(aheadSize, letGo);
    double shareOfMovedKept = shareKept(movedSize, letGo);
    aheadX *= shareOfAheadKept;
    aheadY *= shareOfAheadKept;
    movedX *= shareOfMovedKept;
    movedY *= shareOfMovedKept;
    speedSlipMetersPerSecond =
        slipFrom(Math.max(aheadSize * shareOfAheadKept, movedSize * shareOfMovedKept));
  }

  /**
   * The share of a slide of some size to keep this loop. Up to {@code letGo} of it is let go toward
   * none, and no slide is kept faster than the robot can go: past that, what is held is not a slide
   * but a bias not yet learned, adding up.
   */
  private double shareKept(double size, double letGo) {
    double held = Math.min(Math.max(size - letGo, 0), maxSlideMetersPerSecond);
    return size > 0 ? held / size : 0;
  }

  /**
   * How fast the wheels slide, from the slide held. Within the tolerance it is the accelerometer's
   * noise, and counts as none. Past the tolerance all of it counts, not just the part beyond: the
   * tolerance tells a slide from the noise, and is not a share of every slide.
   */
  private double slipFrom(double heldMetersPerSecond) {
    return heldMetersPerSecond > speedToleranceMetersPerSecond ? heldMetersPerSecond : 0;
  }
}
