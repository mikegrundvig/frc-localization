package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.tuple;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;

/**
 * The estimator, fed by hand: odometry steps at 250 Hz, and camera measurements with the capture
 * times and uncertainties a camera would give.
 */
class PoseEstimatorTest {
  private static final long STEP = 4_000_000L; // 250 Hz
  private static final long T0 = 1_000_000_000L;
  private static final Pose2d START = new Pose2d(1, 1, Rotation2d.ZERO);
  private static final PoseCovariance PLACED = PoseCovariance.fromStdDevs(0.05, 0.05, 0.02);

  /** The field's tags, as 2026's numbers them. */
  private static final Set<Integer> TAGS = tags(32);

  /**
   * Cameras whose images' errors are each new, nothing repeating: the textbook filter, so a test
   * can work its numbers out by hand. (PoseFilterTest covers what repeats.)
   */
  private static final EstimatorConfig TEXTBOOK =
      EstimatorConfigBuilder.builder(EstimatorConfig.DEFAULTS)
          .cameraTurnStdDevDegrees(0)
          .cameraShiftStdDevMeters(0)
          .cameraShiftStdDevDegrees(0)
          .tagPlacementStdDevMeters(0)
          .build();

  /** A robot driven by hand: a clock, a gyro, and wheels that report steady motion. */
  private static final class Robot {
    final PoseEstimator estimator;
    long time = T0;
    double gyroYaw = 0.7; // the gyro started somewhere arbitrary

    Robot(EstimatorConfig config, PoseReset start) {
      estimator = new PoseEstimator(config, point -> field(point), TAGS, start);
      estimator.addOdometry(time, new BodyMotion(0, 0, 0, 0), gyroYaw, YawSource.GYRO);
    }

    Robot() {
      this(TEXTBOOK, new PoseReset(0, START, PLACED, "start"));
      estimator.reset(new PoseReset(T0, START, PLACED, "placed"));
    }

    /** From now on, the robot's other sensors say this about its wheels. */
    Robot under(DriveConditions conditions) {
      estimator.setConditions(conditions);
      return this;
    }

    /** Drives at robot-relative velocities for some steps. */
    Robot drive(int steps, double vx, double vy, double omega) {
      return drive(steps, vx, vy, omega, 0, true);
    }

    Robot drive(int steps, double vx, double vy, double omega, double residual, boolean gyro) {
      double dt = STEP / 1e9;
      for (int i = 0; i < steps; i++) {
        time += STEP;
        gyroYaw += omega * dt;
        estimator.addOdometry(
            time,
            new BodyMotion(vx * dt, vy * dt, omega * dt, residual),
            gyroYaw,
            gyro ? YawSource.GYRO : YawSource.WHEELS);
      }
      return this;
    }

    List<VisionDecision> see(Pose2d pose, long when, PoseCovariance noise) {
      estimator.addVision(
          new VisionMeasurement(
              when,
              pose,
              noise,
              VisionMeasurement.Lever.NONE,
              "front",
              List.of(26),
              VisionMeasurement.Solver.HEADING_CONSTRAINED));
      return estimator.update();
    }

    /**
     * A position solved with the gyro's heading, from tags this far from the robot on the field,
     * taken now.
     */
    VisionDecision seeLeaning(Pose2d pose, Translation2d robotToTags, double stdDev) {
      estimator.addVision(
          new VisionMeasurement(
              time,
              pose,
              position(stdDev),
              new VisionMeasurement.Lever(estimator.pose().getRotation(), robotToTags),
              "front",
              List.of(26),
              VisionMeasurement.Solver.HEADING_CONSTRAINED));
      return estimator.update().get(0);
    }

    /** A camera measurement with heading, from several tags, taken now. */
    VisionDecision seeWithHeading(Pose2d pose) {
      estimator.addVision(
          new VisionMeasurement(
              time,
              pose,
              PoseCovariance.fromStdDevs(0.05, 0.05, Math.toRadians(2)),
              VisionMeasurement.Lever.NONE,
              "front",
              List.of(25, 26),
              VisionMeasurement.Solver.MULTI_TAG));
      return estimator.update().get(0);
    }

    Pose2d pose() {
      return estimator.pose();
    }

    LocalizationEstimate estimate() {
      return estimator.estimate();
    }
  }

  private static DriveConditions tilted(double degrees) {
    return new DriveConditions(Math.toRadians(degrees), 0, 0, 0, false);
  }

  private static final DriveConditions HIT = new DriveConditions(0, 0, 1, 0, false);

  /** The IMU says the wheels slide over the floor at half a meter a second. */
  private static final DriveConditions SLIDING = new DriveConditions(0, 0, 0, 0.5, false);

  private static final DriveConditions DISABLED = new DriveConditions(0, 0, 0, 0, true);

  private static boolean field(Translation2d point) {
    return point.getX() >= 0 && point.getX() <= 16 && point.getY() >= 0 && point.getY() <= 8;
  }

  private static PoseCovariance position(double stdDev) {
    return PoseCovariance.fromStdDevs(stdDev, stdDev, Double.POSITIVE_INFINITY);
  }

  private static void assertSame(Pose2d actual, Pose2d expected, double tolerance) {
    assertThat(actual.getTranslation().getDistance(expected.getTranslation()))
        .as(actual + " vs " + expected)
        .isLessThan(tolerance);
    assertThat(actual.getRotation().minus(expected.getRotation()).getRadians())
        .isCloseTo(0.0, within(tolerance));
  }

  @Test
  void odometryFollowsTheWheelsAlongTheGyrosArc() {
    Robot robot = new Robot().drive(500, 1.0, 0.2, 0.5);

    Pose2d expected = START;
    double dt = STEP / 1e9;
    for (int i = 0; i < 500; i++) {
      expected = expected.plus(new Twist2d(1.0 * dt, 0.2 * dt, 0.5 * dt).exp());
    }
    assertSame(robot.pose(), expected, 1e-9);
  }

  @Test
  void uncertaintyGrowsWithDistanceAndHeadingUncertaintySwingsItSideways() {
    Robot robot = new Robot().drive(1000, 1.0, 0, 0); // 4 m straight ahead, along x

    PoseCovariance covariance = robot.estimate().covariance();
    // Placed to 5 cm, plus 3 cm per square root of meter: sqrt(0.05^2 + 0.03^2 * 4).
    assertThat(covariance.xStdDev()).isCloseTo(Math.sqrt(0.0025 + 0.0036), within(0.001));
    assertThat(covariance.yStdDev()).as("a 0.02 rad heading doubt over 4 m").isGreaterThan(0.09);
  }

  @Test
  void theGyroDriftsSoHeadingGrowsUnsureOverTime() {
    Robot robot = new Robot().drive(15_000, 0, 0, 0); // a minute, sitting still

    double headingStdDev = robot.estimate().covariance().headingStdDev();
    // 0.02 rad when placed, plus 0.5 degrees a minute of drift.
    assertThat(headingStdDev).isCloseTo(Math.hypot(0.02, Math.toRadians(0.5)), within(1e-4));
  }

  @Test
  void aMeasurementPullsTheEstimateByTheKalmanGain() {
    Robot robot = new Robot(TEXTBOOK, new PoseReset(0, START, position(0.1), "x"));
    robot.estimator.reset(
        new PoseReset(T0, START, PoseCovariance.fromStdDevs(0.1, 0.1, 0.02), "x"));
    robot.drive(1, 0, 0, 0);

    List<VisionDecision> decisions =
        robot.see(new Pose2d(1.2, 1, Rotation2d.ZERO), robot.time, position(0.1));

    assertThat(decisions).singleElement().extracting(VisionDecision::accepted).isEqualTo(true);
    assertThat(robot.pose().getX()).as("equal trust: halfway").isCloseTo(1.1, within(1e-6));
    assertThat(robot.estimate().covariance().xStdDev()).isCloseTo(Math.sqrt(0.005), within(1e-5));
    assertThat(robot.pose().getRotation().getRadians())
        .as("a position measurement leaves heading alone")
        .isCloseTo(0.0, within(1e-9));
  }

  /** A camera's result arrives tens of milliseconds after its image: it counts at the image. */
  @Test
  void aLateMeasurementLandsWhereAnOnTimeOneWould() {
    Robot onTime = new Robot();
    Robot late = new Robot();
    long captured = T0 + 25 * STEP + STEP / 3; // between two odometry steps
    Pose2d seen = new Pose2d(1.2, 1.05, Rotation2d.ZERO);

    onTime.drive(26, 2.0, 0, 0.8);
    onTime.see(seen, captured, position(0.05));
    onTime.drive(125, 2.0, 0, 0.8);
    late.drive(151, 2.0, 0, 0.8);
    late.see(seen, captured, position(0.05));

    assertSame(late.pose(), onTime.pose(), 1e-9);
    assertThat(late.estimate().covariance().xx())
        .isCloseTo(onTime.estimate().covariance().xx(), within(1e-12));
  }

  @Test
  void measurementsGiveTheSameResultInAnyOrderOfArrival() {
    Robot inOrder = new Robot().drive(100, 1.0, 0.5, 0.3);
    Robot reversed = new Robot().drive(100, 1.0, 0.5, 0.3);
    long first = T0 + 40 * STEP + 1_000_000;
    long second = T0 + 70 * STEP + 2_500_000;
    // A few centimeters from where the robot was then: both pass the gate.
    Pose2d one = new Pose2d(1.18, 1.10, Rotation2d.ZERO);
    Pose2d two = new Pose2d(1.29, 1.13, Rotation2d.ZERO);

    inOrder.see(one, first, position(0.05));
    inOrder.see(two, second, position(0.05));
    reversed.see(two, second, position(0.05));
    reversed.see(one, first, position(0.05));

    assertSame(reversed.pose(), inOrder.pose(), 1e-9);
    assertThat(inOrder.estimate().health().accepted()).isEqualTo(2);
  }

  /**
   * Two images that rule each other out, placed to 30 cm: the earlier puts the robot 60 cm ahead,
   * the later 40 cm behind. Weighed in order, the earlier passes the gate and the later, a meter
   * from it, doesn't. Arriving the other way round, the later passes first; when the earlier comes,
   * the replay weighs the later again against what the earlier says, and it no longer passes, so
   * it's taken back, and says so. Either way the estimate is where the earlier image put it.
   */
  @Test
  void anAppliedImageThatALateEarlierOneRulesOutIsTakenBack() {
    PoseReset rough = new PoseReset(T0, START, PoseCovariance.fromStdDevs(0.3, 0.3, 0.02), "x");
    Robot inOrder = new Robot(TEXTBOOK, rough);
    Robot reversed = new Robot(TEXTBOOK, rough);
    for (Robot robot : List.of(inOrder, reversed)) {
      robot.estimator.reset(rough);
      robot.drive(20, 0, 0, 0);
    }
    long earlier = T0 + 5 * STEP + 1_000_000;
    long later = T0 + 10 * STEP + 1_000_000;
    Pose2d ahead = new Pose2d(1.6, 1, Rotation2d.ZERO);
    Pose2d behind = new Pose2d(0.6, 1, Rotation2d.ZERO);

    boolean aheadFirst = inOrder.see(ahead, earlier, position(0.05)).get(0).accepted();
    boolean behindSecond = inOrder.see(behind, later, position(0.05)).get(0).accepted();
    boolean behindFirst = reversed.see(behind, later, position(0.05)).get(0).accepted();
    List<VisionDecision> lateArrival = reversed.see(ahead, earlier, position(0.05));

    assertThat(List.of(aheadFirst, behindSecond, behindFirst)).containsExactly(true, false, true);
    assertThat(lateArrival)
        .extracting(decision -> decision.measurement().pose(), VisionDecision::verdict)
        .containsExactly(tuple(ahead, Verdict.ACCEPTED), tuple(behind, Verdict.GATED));
    assertThat(inOrder.pose().getX()).isGreaterThan(1.5);
    assertSame(reversed.pose(), inOrder.pose(), 1e-9);
    assertThat(reversed.estimate().covariance().xx())
        .isCloseTo(inOrder.estimate().covariance().xx(), within(1e-12));
  }

  @Test
  void aMeasurementFarOutsideTheUncertaintyIsGatedOut() {
    Robot robot = new Robot().drive(10, 0, 0, 0);

    List<VisionDecision> decisions =
        robot.see(new Pose2d(3, 1, Rotation2d.ZERO), robot.time, position(0.05));

    VisionDecision decision = decisions.get(0);
    assertThat(decision.accepted()).isFalse();
    assertThat(decision.reason()).startsWith("gate: ").endsWith("> 13.8");
    assertThat(decision.innovationMeters()).isCloseTo(2.0, within(1e-9));
    assertSame(robot.pose(), START, 1e-9);
  }

  /**
   * A measurement whose uncertainty leaves nothing to measure a distance in can't be compared with
   * the estimate. It's left out, with a verdict of its own and a count among the rejected, new or
   * taken back; the estimate stays where it was, and the next good measurement is weighed as ever.
   * Until 2026-10-11 it vanished: no decision, and no count.
   */
  @Test
  void aMeasurementThatCannotBeComparedIsLeftOutWithAVerdictOfItsOwn() {
    Robot robot = new Robot().drive(10, 0, 0, 0);
    // So wide, and its x and y so bound together, that the spread's determinant is infinity less
    // infinity: not a number.
    PoseCovariance noSpread = new PoseCovariance(1e200, 1e200, 1e200, Double.POSITIVE_INFINITY);

    List<VisionDecision> first =
        robot.see(new Pose2d(1.02, 1, Rotation2d.ZERO), robot.time, noSpread);
    robot.drive(5, 0, 0, 0);
    List<VisionDecision> second =
        robot.see(new Pose2d(3, 1, Rotation2d.ZERO), robot.time, noSpread);
    Pose2d after = robot.pose();
    LocalizationEstimate.Mode afterTwo = robot.estimate().mode();
    List<VisionDecision> good =
        robot.see(new Pose2d(1.02, 1, Rotation2d.ZERO), robot.time, position(0.05));

    assertThat(first).hasSize(1);
    assertThat(second).hasSize(1);
    for (VisionDecision decision : List.of(first.get(0), second.get(0))) {
      assertThat(decision.verdict()).isEqualTo(Verdict.NOT_COMPARABLE);
      assertThat(decision.accepted()).isFalse();
      assertThat(decision.reason())
          .isEqualTo(
              "can't be compared with the estimate: its uncertainty leaves no spread to measure"
                  + " in");
      assertThat(decision.innovationMeters()).isNaN();
      assertThat(decision.chiSquared()).isNaN();
      assertThat(decision.threshold()).isNaN();
    }
    assertSame(after, START, 1e-12);
    assertThat(good).singleElement().satisfies(d -> assertThat(d.accepted()).isTrue());
    LocalizationEstimate.Health health = robot.estimate().health();
    assertThat(health.rejected()).as("both, counted").isEqualTo(2);
    assertThat(health.accepted()).isEqualTo(1);
    assertThat(afterTwo)
        .as("two in a row aren't the cameras disagreeing with the estimate")
        .isNotEqualTo(LocalizationEstimate.Mode.LOST);
  }

  @Test
  void measurementsTheHistoryCannotPlaceAreRejected() {
    Robot robot = new Robot().drive(500, 0, 0, 0); // 2 s: the history keeps 1.5 s
    long now = robot.time;

    assertThat(robot.see(START, T0 + STEP, position(0.05)).get(0).reason())
        .isEqualTo("too late: older than the history kept");
    assertThat(robot.see(START, now + 100_000_000L, position(0.05)).get(0).reason())
        .isEqualTo("from the future: after the newest odometry");
    assertThat(robot.see(new Pose2d(-1, 1, Rotation2d.ZERO), now, position(0.05)).get(0).reason())
        .isEqualTo("off the field");
    robot.estimator.reset(new PoseReset(now, START, PLACED, "placed again"));
    robot.drive(5, 0, 0, 0);
    assertThat(robot.see(START, now, position(0.05)).get(0).reason())
        .isEqualTo("before the robot was last placed or recovered");
    assertThat(robot.estimate().health().rejected()).isEqualTo(4);
  }

  @Test
  void aMeasurementBeforeOdometryStartsIsRejected() {
    PoseEstimator estimator =
        new PoseEstimator(
            EstimatorConfig.DEFAULTS,
            point -> true,
            TAGS,
            new PoseReset(0, START, PLACED, "start"));

    estimator.addVision(
        new VisionMeasurement(
            5,
            START,
            position(0.05),
            VisionMeasurement.Lever.NONE,
            "front",
            List.of(1),
            VisionMeasurement.Solver.MULTI_TAG));

    assertThat(estimator.update().get(0).reason()).isEqualTo("before odometry started");
    assertThat(estimator.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.INITIALIZING);
  }

  /** Knocked a meter sideways, the wheels saw nothing; the cameras all agree it moved. */
  @Test
  void whenRejectedMeasurementsAgreeOdometryWasWrongAndTheEstimateRecovers() {
    Robot robot = new Robot().drive(10, 0, 0, 0);
    Pose2d truth = new Pose2d(1, 2, Rotation2d.ZERO);

    robot.drive(10, 0, 0, 0).see(truth, robot.time, position(0.05));
    robot.drive(10, 0, 0, 0).see(truth, robot.time, position(0.05));
    assertThat(robot.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.LOST);
    robot.drive(10, 0, 0, 0).see(truth, robot.time, position(0.05));

    assertThat(robot.pose().getTranslation().getDistance(truth.getTranslation())).isLessThan(1e-9);
    assertThat(robot.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.TRACKING);
    assertThat(robot.estimate().health().recoveries()).isEqualTo(1);
    assertThat(robot.estimate().covariance().xStdDev()).isCloseTo(0.25, within(1e-9));
  }

  /**
   * Three images agree the robot was knocked a meter sideways, but a fourth just before them, two
   * meters off, holds the recovery back until it's older than the recovery window. Then the camera
   * is unplugged, and the three recover the estimate a second after the first was taken. The images
   * are as old as when they were taken, not when they were acted on: a second after the last of
   * them, the estimate is dead reckoning, not tracking tags.
   */
  @Test
  void aRecoveryCountsItsImagesAsOldAsWhenTheyWereTaken() {
    Robot robot = new Robot().drive(10, 0, 0, 0);
    int loop = 5; // steps a robot loop, 20 ms
    robot.drive(loop, 0, 0, 0).see(new Pose2d(1, 3, Rotation2d.ZERO), robot.time, position(0.05));
    for (int i = 0; i < 3; i++) {
      robot.drive(loop, 0, 0, 0).see(new Pose2d(1, 2, Rotation2d.ZERO), robot.time, position(0.05));
    }
    long lastTaken = robot.time;
    assertThat(robot.estimate().health().recoveries()).isZero();

    while (robot.estimate().health().recoveries() == 0) {
      robot.drive(loop, 0, 0, 0).estimator.update();
    }
    double recoveredAfter = (robot.time - lastTaken) / 1e9;
    while (robot.time - lastTaken <= 1e9 * TEXTBOOK.visionTimeoutSeconds()) {
      robot.drive(loop, 0, 0, 0).estimator.update();
    }

    assertThat(recoveredAfter).as("seconds after the last image").isBetween(0.9, 1.0);
    assertThat(robot.pose().getTranslation().getDistance(new Translation2d(1, 2))).isLessThan(1e-9);
    assertThat(robot.estimate().health().visionAgeSeconds())
        .isCloseTo((robot.time - lastTaken) / 1e9, within(1e-9));
    assertThat(robot.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.DEAD_RECKONING);
  }

  @Test
  void rejectedMeasurementsThatDisagreeWithEachOtherChangeNothing() {
    Robot robot = new Robot().drive(10, 0, 0, 0);

    robot.drive(10, 0, 0, 0).see(new Pose2d(1, 2, Rotation2d.ZERO), robot.time, position(0.05));
    robot.drive(10, 0, 0, 0).see(new Pose2d(3, 1, Rotation2d.ZERO), robot.time, position(0.05));
    robot.drive(10, 0, 0, 0).see(new Pose2d(1, 4, Rotation2d.ZERO), robot.time, position(0.05));

    assertSame(robot.pose(), START, 1e-9);
    assertThat(robot.estimate().health().recoveries()).isZero();
  }

  /**
   * Rejections count toward a recovery only while they're recent: three that agree, the first more
   * than {@link EstimatorConfig#recoveryWindowSeconds} before the third, are two, and the estimate
   * stays where it was. A fourth within the window of the second makes three, and it recovers.
   */
  @Test
  void rejectionsOlderThanTheRecoveryWindowDontCount() {
    Robot robot = new Robot().drive(10, 0, 0, 0);
    Pose2d truth = new Pose2d(1, 2, Rotation2d.ZERO);
    int window = (int) (TEXTBOOK.recoveryWindowSeconds() / (STEP / 1e9)); // in steps

    robot.see(truth, robot.time, position(0.05));
    robot.drive(window / 2 + 10, 0, 0, 0).see(truth, robot.time, position(0.05));
    robot.drive(window / 2 + 10, 0, 0, 0).see(truth, robot.time, position(0.05));

    assertThat(robot.estimate().health().recoveries()).isZero();
    assertSame(robot.pose(), START, 1e-9);
    assertThat(robot.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.LOST);

    robot.drive(10, 0, 0, 0).see(truth, robot.time, position(0.05));

    assertThat(robot.estimate().health().recoveries()).isEqualTo(1);
    assertThat(robot.pose().getTranslation().getDistance(truth.getTranslation())).isLessThan(1e-9);
  }

  /**
   * A step no newer than the last (a reading taken twice, or out of order) is ignored, however far
   * it says the robot moved: the estimate and its doubt stay as they were, and the next step
   * carries on as if it never came.
   */
  @Test
  void aStepNoNewerThanTheLastChangesNothing() {
    Robot robot = new Robot().drive(25, 1, 0, 0.5);
    Robot never = new Robot().drive(25, 1, 0, 0.5);
    Pose2d pose = robot.pose();
    PoseCovariance covariance = robot.estimate().covariance();

    BodyMotion far = new BodyMotion(0.5, 0.2, 0.3, 0.1);
    robot.estimator.addOdometry(robot.time, far, robot.gyroYaw + 0.3, YawSource.GYRO);
    robot.estimator.addOdometry(robot.time - STEP, far, robot.gyroYaw + 0.3, YawSource.GYRO);

    assertThat(robot.pose()).isEqualTo(pose);
    assertThat(robot.estimate().covariance()).isEqualTo(covariance);
    robot.drive(25, 1, 0, 0.5);
    never.drive(25, 1, 0, 0.5);
    assertThat(robot.pose()).isEqualTo(never.pose());
    assertThat(robot.estimate().covariance()).isEqualTo(never.estimate().covariance());
  }

  /**
   * A position solved with the gyro's heading is off by the heading's error times the distance to
   * its tags, square to them: with the heading unsure by 0.02 rad and tags 3 m ahead (along x), the
   * fix may be 6 cm off along y on top of its own noise. So it pulls the estimate less along y than
   * a fix that doesn't lean on the heading, and as much along x.
   */
  @Test
  void aFixThatLeansOnAnUnsureHeadingCountsForLessSquareToItsTags() {
    Robot leaning = new Robot().drive(1, 0, 0, 0);
    Robot upright = new Robot().drive(1, 0, 0, 0);
    Pose2d off = new Pose2d(START.getX() + 0.05, START.getY() + 0.05, Rotation2d.ZERO);

    leaning.seeLeaning(off, new Translation2d(3, 0), 0.05);
    upright.seeLeaning(off, Translation2d.ZERO, 0.05);

    // Placed to 5 cm, the fix 5 cm: halfway along x for both. Along y the leaning one's spread is
    // hypot(5, 6) cm, so it moves the estimate 25 / (25 + 25 + 36) of the way.
    assertThat(leaning.pose().getX() - START.getX()).isCloseTo(0.025, within(1e-4));
    assertThat(upright.pose().getX() - START.getX()).isCloseTo(0.025, within(1e-4));
    assertThat(upright.pose().getY() - START.getY()).isCloseTo(0.025, within(1e-4));
    assertThat(leaning.pose().getY() - START.getY()).isCloseTo(0.05 * 25 / 86, within(1e-4));
  }

  /**
   * Driving with an unsure heading ties the position's error to the heading's, but a position fix
   * still never turns the heading, nor makes it surer: that's the gyro's, and the measurements with
   * a heading of their own.
   */
  @Test
  void aPositionFixNeverTurnsTheHeading() {
    Robot robot = new Robot().drive(500, 1.0, 0, 0); // 2 m ahead, the heading unsure by 0.02 rad
    Rotation2d heading = robot.pose().getRotation();
    double headingStdDev = robot.estimate().covariance().headingStdDev();

    Pose2d aside = new Pose2d(robot.pose().getX(), robot.pose().getY() + 0.05, Rotation2d.ZERO);
    VisionDecision decision = robot.seeLeaning(aside, new Translation2d(2, 1), 0.03);

    assertThat(decision.accepted()).isTrue();
    assertThat(robot.pose().getY()).as("the position moved").isGreaterThan(START.getY() + 0.01);
    assertThat(robot.pose().getRotation()).isEqualTo(heading);
    assertThat(robot.estimate().covariance().headingStdDev()).isEqualTo(headingStdDev);
  }

  /**
   * Standing still, a thousand fixes from tags 3 m ahead, each 5 cm unsure: along the tags they
   * average down to millimeters, but square to them each is off by the heading's error times 3 m,
   * the same in every one, and no number of them can know the position better than that. (Placed
   * only roughly, so the placement adds nothing that does.)
   */
  @Test
  void manyFixesCantMakeThePositionSurerThanTheHeadingAllows() {
    PoseReset rough = new PoseReset(T0, START, PoseCovariance.fromStdDevs(1, 1, 0.02), "rough");
    Robot robot = new Robot(TEXTBOOK, rough);
    robot.estimator.reset(rough);
    robot.drive(1, 0, 0, 0);

    for (int i = 0; i < 1000; i++) {
      robot.drive(1, 0, 0, 0).seeLeaning(START, new Translation2d(3, 0), 0.05);
    }

    PoseCovariance covariance = robot.estimate().covariance();
    double lever = 3 * covariance.headingStdDev(); // 6 cm
    assertThat(covariance.xStdDev()).as("along the tags").isLessThan(0.002);
    assertThat(covariance.yStdDev()).as("square to them").isCloseTo(lever, within(0.02 * lever));
  }

  /**
   * A camera the mount check doubts by 5 cm, its tags 2 m away: corrected heading measurements
   * only, the doubt is the vision pipeline's business; with the doubt added to what the camera
   * repeats, a thousand of its fixes can't place the robot better than the doubt along its tags.
   */
  @Test
  void aCamerasMountDoubtCanRaiseWhatItRepeats() {
    double[] alongTheTags = new double[2];
    EstimatorConfig.HeadingCorrection[] ways = {
      EstimatorConfig.HeadingCorrection.HEADING_MEASUREMENTS,
      EstimatorConfig.HeadingCorrection.HEADING_MEASUREMENTS_AND_MOUNT_DOUBT
    };
    for (int way = 0; way < 2; way++) {
      PoseReset rough = new PoseReset(T0, START, PoseCovariance.fromStdDevs(1, 1, 0.02), "rough");
      Robot robot = new Robot(with().headingCorrection(ways[way]).build(), rough);
      robot.estimator.reset(rough);
      robot.drive(1, 0, 0, 0);
      robot.estimator.mountDoubt("front", 0.05, 2); // doubted before its first fix
      robot.estimator.update();
      for (int i = 0; i < 1000; i++) {
        robot.estimator.mountDoubt("front", 0.05, 2);
        robot.drive(1, 0, 0, 0).seeLeaning(START, new Translation2d(2, 0), 0.05);
      }
      alongTheTags[way] = robot.estimate().covariance().xStdDev();
    }

    // Along its tag: its shift and the tag's placement, and the doubt with them.
    double repeated =
        Math.hypot(
            EstimatorConfig.DEFAULTS.cameraShiftStdDevMeters(),
            EstimatorConfig.DEFAULTS.tagPlacementStdDevMeters());
    assertThat(alongTheTags[0]).as("the doubt ignored").isCloseTo(repeated, within(0.1 * repeated));
    double doubted = Math.hypot(repeated, 0.05);
    assertThat(alongTheTags[1]).as("the doubt added").isCloseTo(doubted, within(0.1 * doubted));
  }

  @Test
  void aMeasurementWithHeadingCorrectsTheGyroOffset() {
    Robot robot = new Robot(TEXTBOOK, new PoseReset(0, START, PLACED, "x"));
    robot.estimator.reset(
        new PoseReset(T0, START, PoseCovariance.fromStdDevs(0.05, 0.05, 0.1), "x"));
    robot.drive(1, 0, 0, 0);

    robot.see(
        new Pose2d(1, 1, new Rotation2d(0.1)),
        robot.time,
        PoseCovariance.fromStdDevs(0.05, 0.05, 0.1));

    assertThat(robot.pose().getRotation().getRadians()).isCloseTo(0.05, within(1e-4));
    double before = robot.pose().getRotation().getRadians();
    robot.drive(250, 0, 0, 1.0); // a radian of turning, by the gyro
    assertThat(robot.pose().getRotation().getRadians() - before).isCloseTo(1.0, within(1e-9));
  }

  @Test
  void modesFollowWhatTheEstimateRestsOn() {
    PoseEstimator estimator =
        new PoseEstimator(
            EstimatorConfig.DEFAULTS,
            point -> true,
            TAGS,
            new PoseReset(0, START, PoseCovariance.fromStdDevs(10, 10, 3), "unknown"));
    estimator.addOdometry(T0, new BodyMotion(0, 0, 0, 0), 0, YawSource.GYRO);
    assertThat(estimator.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.INITIALIZING);

    estimator.reset(new PoseReset(T0, START, PLACED, "placed"));
    assertThat(estimator.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.DEAD_RECKONING);

    estimator.addOdometry(T0 + STEP, new BodyMotion(0, 0, 0, 0), 0, YawSource.GYRO);
    estimator.addVision(
        new VisionMeasurement(
            T0 + STEP,
            START,
            position(0.05),
            VisionMeasurement.Lever.NONE,
            "front",
            List.of(26),
            VisionMeasurement.Solver.MULTI_TAG));
    estimator.update();
    assertThat(estimator.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.TRACKING);
    assertThat(estimator.estimate().health().visionAgeSeconds()).isZero();

    estimator.addOdometry(T0 + 300 * STEP, new BodyMotion(0, 0, 0, 0), 0, YawSource.GYRO);
    assertThat(estimator.estimate().mode())
        .as("1.2 s without vision")
        .isEqualTo(LocalizationEstimate.Mode.DEAD_RECKONING);

    estimator.addOdometry(T0 + 301 * STEP, new BodyMotion(0, 0, 0, 0), 0, YawSource.WHEELS);
    assertThat(estimator.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.IMU_FAULT);
  }

  @Test
  void withTheGyroLostTheWheelsTurnTheRobotAndHeadingGrowsUnsureFaster() {
    Robot withGyro = new Robot().drive(250, 0, 0, 1.0);
    Robot without = new Robot().drive(250, 0, 0, 1.0, 0, false);

    assertThat(without.pose().getRotation().getRadians()).isCloseTo(1.0, within(1e-9));
    assertThat(without.estimate().covariance().headingStdDev())
        .isGreaterThan(3 * withGyro.estimate().covariance().headingStdDev());
    assertThat(without.estimate().health().gyroConnected()).isFalse();
  }

  /** Back from a brownout, a gyro may read anything: the heading carries on without a jump. */
  @Test
  void aGyroThatComesBackReadingSomethingElseDoesNotJumpTheHeading() {
    Robot robot = new Robot().drive(10, 0, 0, 0, 0, false);
    double heading = robot.pose().getRotation().getRadians();

    robot.gyroYaw += 2.5; // it rebooted
    robot.drive(1, 0, 0, 0);
    robot.drive(10, 0, 0, 0);

    assertThat(robot.pose().getRotation().getRadians()).isCloseTo(heading, within(1e-9));
  }

  /**
   * The step where the gyro comes back is turned by the wheels, so its turn counts as the wheels'
   * doubt, not the gyro's: the heading grows as unsure over it as over any wheel-turned step.
   */
  @Test
  void theStepWhereTheGyroComesBackIsTheWheels() {
    Robot back = new Robot().drive(10, 0, 0, 0, 0, false);
    Robot lost = new Robot().drive(10, 0, 0, 0, 0, false);

    back.drive(1, 0, 0, 1.0); // the gyro back, turning
    lost.drive(1, 0, 0, 1.0, 0, false); // still lost, turning by the wheels

    assertThat(back.estimate().covariance().heading())
        .isCloseTo(lost.estimate().covariance().heading(), within(1e-15));
    assertThat(back.estimate().health().gyroConnected()).as("the gyro reads again").isTrue();
  }

  /** While the backup IMU turns odometry in the gyro's place, its own scale's doubt grows. */
  @Test
  void theBackupImuTurnsWithItsOwnScalesDoubt() {
    Robot gyro = new Robot().drive(1, 0, 0, 0);
    PoseEstimator backup =
        new PoseEstimator(TEXTBOOK, point -> true, TAGS, new PoseReset(0, START, PLACED, "start"));
    backup.addOdometry(T0, new BodyMotion(0, 0, 0, 0), 0.7, YawSource.BACKUP);
    backup.reset(new PoseReset(T0, START, PLACED, "placed"));
    double yaw = 0.7;
    for (int i = 1; i <= 250; i++) {
      yaw += 2 * Math.PI / 250;
      backup.addOdometry(
          T0 + i * STEP, new BodyMotion(0, 0, 2 * Math.PI / 250, 0), yaw, YawSource.BACKUP);
    }
    gyro.drive(249, 0, 0, 2 * Math.PI);

    // A full turn on each: the backup's 2° a turn against the gyro's 1°.
    double backupTurn =
        Math.toRadians(EstimatorConfig.DEFAULTS.backupHeadingStdDevDegreesPerTurn());
    double gyroTurn = Math.toRadians(EstimatorConfig.DEFAULTS.headingStdDevDegreesPerTurn());
    assertThat(backup.estimate().covariance().headingStdDev()).isGreaterThan(backupTurn);
    assertThat(gyro.estimate().covariance().headingStdDev())
        .isGreaterThan(gyroTurn)
        .isLessThan(backupTurn);
  }

  @Test
  void slippingWheelsMakeOdometryLessTrusted() {
    Robot gripping = new Robot().drive(250, 2.0, 0, 0);
    Robot slipping = new Robot().drive(250, 2.0, 0, 0, 0.004, true);

    assertThat(slipping.estimate().covariance().xStdDev())
        .isGreaterThan(gripping.estimate().covariance().xStdDev() * 1.2);
    assertThat(slipping.estimate().health().residualMeters()).isEqualTo(0.004);
  }

  @Test
  void aResetPlacesTheRobotAndAResetWithoutHeadingKeepsItsHeading() {
    Robot robot = new Robot().drive(250, 0, 0, 0.5);
    double heading = robot.pose().getRotation().getRadians();

    robot.estimator.reset(
        new PoseReset(robot.time, new Pose2d(4, 3, Rotation2d.ZERO), position(0.1), "x"));

    assertThat(robot.pose().getTranslation()).isEqualTo(new Translation2d(4, 3));
    assertThat(robot.pose().getRotation().getRadians()).isCloseTo(heading, within(1e-12));
  }

  @Test
  void itPredictsWhereTheRobotWillBe() {
    Robot robot = new Robot().drive(100, 2.0, 0, 0);

    LocalizationEstimate estimate = robot.estimate();
    Pose2d ahead = estimate.predict(0.5);

    assertThat(estimate.fieldVelocity().getX()).isCloseTo(2.0, within(1e-9));
    assertThat(ahead.getX() - estimate.pose().getX()).isCloseTo(1.0, within(1e-9));
  }

  @Test
  void itReportsItsHealth() {
    Robot robot = new Robot().drive(10, 0, 0, 0);

    robot.see(new Pose2d(1.02, 1, Rotation2d.ZERO), robot.time, position(0.05));
    robot.see(new Pose2d(5, 1, Rotation2d.ZERO), robot.time, position(0.05));
    robot.drive(50, 0, 0, 0);

    LocalizationEstimate.Health health = robot.estimate().health();
    assertThat(health.accepted()).isEqualTo(1);
    assertThat(health.rejected()).isEqualTo(1);
    assertThat(health.lastInnovationMeters()).isGreaterThan(3.9);
    assertThat(health.visionAgeSeconds()).isCloseTo(0.2, within(1e-9));
  }

  @Test
  void aTiltedRobotsWheelsCountForLessAndPastTheLimitForNothing() {
    Robot halfway = new Robot().under(tilted(15)).drive(250, 1.0, 0, 0); // a meter of wheel travel
    Robot steep = new Robot().under(tilted(30)).drive(250, 1.0, 0, 0);
    Robot level = new Robot().drive(250, 0.5, 0, 0); // half a meter, level

    assertThat(halfway.pose().getX() - START.getX()).isCloseTo(0.5, within(1e-9));
    assertThat(steep.pose().getX()).isCloseTo(START.getX(), within(1e-12));
    // The half meter left out may have been real: the doubt grows by a good share of it.
    double added =
        Math.sqrt(halfway.estimate().covariance().xx() - level.estimate().covariance().xx());
    assertThat(added).isBetween(0.3, 0.5);
    assertThat(halfway.estimate().health().tiltRadians()).isEqualTo(Math.toRadians(15));
  }

  @Test
  void slipTheImuSeesAddsUpAsOneErrorNotManySmallOnes() {
    Robot gripping = new Robot().drive(62, 2.0, 0, 0); // half a meter in a quarter second
    Robot slipping = new Robot().under(SLIDING).drive(62, 2.0, 0, 0);

    double added =
        Math.sqrt(slipping.estimate().covariance().xx() - gripping.estimate().covariance().xx());
    // The wheels may have slid 12 cm in the quarter second. Squared step by step it would add only
    // 1.6 cm; as one error building in one direction it adds nearly all of it.
    assertThat(added).isBetween(0.09, 0.125);
    assertThat(slipping.estimate().health().suspectTravelMeters()).isGreaterThan(0.09);
  }

  @Test
  void theDoubtGrowsWithHowFastTheWheelsSlide() {
    Robot gripping = new Robot().drive(62, 2.0, 0, 0);
    Robot sliding = new Robot().under(SLIDING).drive(62, 2.0, 0, 0);
    Robot slidingFaster =
        new Robot().under(new DriveConditions(0, 0, 0, 1.0, false)).drive(62, 2.0, 0, 0);

    double base = gripping.estimate().covariance().xx();
    double slow = Math.sqrt(sliding.estimate().covariance().xx() - base);
    double fast = Math.sqrt(slidingFaster.estimate().covariance().xx() - base);

    assertThat(fast / slow).as("twice as fast, twice the doubt").isCloseTo(2.0, within(0.01));
  }

  @Test
  void aCollisionAddsItsDoubtAtOnceAndOnceForItsWindow() {
    Robot robot = new Robot().drive(10, 0, 0, 0);
    double before = robot.estimate().covariance().xx();

    robot.under(HIT).drive(1, 0, 0, 0);
    double hit = robot.estimate().covariance().xx();
    robot.under(HIT).drive(5, 0, 0, 0); // the IMU still jolting, a loop later
    boolean duringWindow = robot.estimate().health().collisionActive();
    robot.under(DriveConditions.NORMAL).drive(100, 0, 0, 0);

    assertThat(hit - before).isCloseTo(0.0625, within(1e-9)); // 25 cm
    assertThat(robot.estimate().covariance().xx()).isCloseTo(hit, within(1e-12));
    assertThat(duringWindow).isTrue();
    assertThat(robot.estimate().health().collisionActive()).isFalse();
  }

  /**
   * A jolt the wheels partly explain is a share of a collision, and adds that share of its spread:
   * half a collision, half its 25 cm. It isn't a hit, and within a hit's window it adds nothing.
   */
  @Test
  void aShareOfACollisionAddsThatShareOfItsSpread() {
    DriveConditions halfAHit = new DriveConditions(0, 0, 0.5, 0, false);
    Robot robot = new Robot().drive(10, 0, 0, 0);
    double before = robot.estimate().covariance().xx();

    robot.under(halfAHit).drive(1, 0, 0, 0);
    double half = robot.estimate().covariance().xx();
    boolean hit = robot.estimate().health().collisionActive();
    robot.under(HIT).drive(1, 0, 0, 0);
    double whole = robot.estimate().covariance().xx();
    robot.under(halfAHit).drive(5, 0, 0, 0);

    assertThat(Math.sqrt(half - before)).isCloseTo(0.125, within(1e-9));
    assertThat(hit).isFalse();
    assertThat(whole - half).as("a whole one after it").isCloseTo(0.0625, within(1e-9));
    assertThat(robot.estimate().covariance().xx()).isCloseTo(whole, within(1e-12));
  }

  @Test
  void aDisabledRobotMayBeCarriedSoItsPositionGrowsUnsure() {
    Robot robot = new Robot().under(DISABLED).drive(15_000, 0, 0, 0); // a minute

    // Placed to 5 cm, plus 2 cm per square root of second.
    assertThat(robot.estimate().covariance().xStdDev())
        .isCloseTo(Math.sqrt(0.0025 + 0.0004 * 60), within(1e-3));
  }

  @Test
  void aLateMeasurementReplaysEachStepUnderItsOwnConditions() {
    Robot inTime = new Robot().drive(50, 1.0, 0, 0);
    Robot late = new Robot().drive(50, 1.0, 0, 0);
    long captured = inTime.time;
    Pose2d seen = new Pose2d(1.22, 1.03, Rotation2d.ZERO);

    inTime.see(seen, captured, position(0.05));
    for (Robot robot : List.of(inTime, late)) {
      robot.under(tilted(12)).drive(40, 1.0, 0.2, 0.3);
      robot.under(HIT).drive(10, 0.5, 0, 0);
      robot.under(SLIDING).drive(40, 2.0, -0.5, 0);
      robot.under(DriveConditions.NORMAL).drive(10, 1.0, 0, 0);
    }
    late.see(seen, captured, position(0.05));

    assertSame(late.pose(), inTime.pose(), 1e-9);
    PoseCovariance expected = inTime.estimate().covariance();
    PoseCovariance actual = late.estimate().covariance();
    assertThat(actual.xx()).isCloseTo(expected.xx(), within(1e-12));
    assertThat(actual.yy()).isCloseTo(expected.yy(), within(1e-12));
    assertThat(actual.heading()).isCloseTo(expected.heading(), within(1e-12));
  }

  @Test
  void theHealthSaysHowFarTheCamerasDisagreedLately() {
    Robot robot = new Robot().drive(10, 0, 0, 0);
    double before = robot.estimate().health().recentDisagreementMeters();

    robot.see(new Pose2d(1.05, 1, Rotation2d.ZERO), robot.time, position(0.05));
    double after = robot.estimate().health().recentDisagreementMeters();
    robot.drive(150, 0, 0, 0); // 0.6 s, no camera

    assertThat(before).isNaN();
    assertThat(after).isCloseTo(0.05, within(1e-9));
    assertThat(robot.estimate().health().recentDisagreementMeters()).isNaN();
  }

  @Test
  void aHeadingResetTurnsTheRobotWhereItStands() {
    Robot robot = new Robot().drive(100, 1.0, 0, 0);
    Translation2d here = robot.pose().getTranslation();

    robot.estimator.resetHeading(Rotation2d.fromDegrees(90), Math.toRadians(5));
    Pose2d reset = robot.pose();
    robot.drive(250, 1.0, 0, 0);

    assertThat(reset.getTranslation()).isEqualTo(here);
    assertThat(reset.getRotation().getDegrees()).isCloseTo(90, within(1e-9));
    assertThat(robot.estimate().covariance().headingStdDev())
        .isCloseTo(Math.toRadians(5), within(1e-4));
    assertThat(robot.pose().getY() - here.getY())
        .as("forward is now +y")
        .isCloseTo(1, within(1e-6));
  }

  @Test
  void aHeadingResetBeforeOdometryStartsIsKept() {
    PoseEstimator estimator =
        new PoseEstimator(
            EstimatorConfig.DEFAULTS,
            point -> true,
            TAGS,
            new PoseReset(0, START, PLACED, "start"));

    estimator.resetHeading(Rotation2d.PI, 0.1);
    estimator.addOdometry(T0, new BodyMotion(0, 0, 0, 0), 0.4, YawSource.GYRO);

    assertThat(estimator.pose().getRotation().getDegrees()).isCloseTo(180, within(1e-9));
    assertThat(estimator.pose().getTranslation()).isEqualTo(START.getTranslation());
    assertThatThrownBy(() -> estimator.resetHeading(Rotation2d.ZERO, 0))
        .isInstanceOf(IllegalArgumentException.class);
  }

  @Test
  void camerasThatAgreeOnAnotherHeadingTurnTheEstimateToIt() {
    // The robot really faces +y, but was told it faces +x: a driver's reset facing the wrong way.
    Robot robot = new Robot().drive(10, 0, 0, 0);
    Pose2d truth = new Pose2d(1, 1, Rotation2d.fromDegrees(90));

    List<Boolean> accepted = new ArrayList<>();
    for (int i = 0; i < 3; i++) {
      accepted.add(robot.seeWithHeading(truth).accepted());
      robot.drive(8, 0, 0, 0);
    }
    Pose2d recovered = robot.pose();
    boolean afterwards = robot.seeWithHeading(truth).accepted();

    assertThat(accepted).containsExactly(false, false, false);
    assertThat(recovered.getRotation().getDegrees()).isCloseTo(90, within(1e-6));
    assertThat(robot.estimate().health().recoveries()).isEqualTo(1);
    assertThat(afterwards).as("the next one, taken").isTrue();
  }

  @Test
  void headingsEitherSideOfHalfATurnAgree() {
    Robot robot = new Robot().drive(10, 0, 0, 0);

    for (double degrees : new double[] {179, -179, 179}) {
      robot.seeWithHeading(new Pose2d(1, 1, Rotation2d.fromDegrees(degrees)));
      robot.drive(8, 0, 0, 0);
    }

    assertThat(Math.abs(robot.pose().getRotation().getDegrees())).isGreaterThan(179);
  }

  @Test
  void camerasThatDisagreeWithEachOtherDontTurnIt() {
    Robot robot = new Robot().drive(10, 0, 0, 0);

    for (double degrees : new double[] {60, 90, 120}) {
      robot.seeWithHeading(new Pose2d(1, 1, Rotation2d.fromDegrees(degrees)));
      robot.drive(8, 0, 0, 0);
    }

    assertThat(robot.pose().getRotation().getDegrees()).isCloseTo(0, within(1e-6));
    assertThat(robot.estimate().health().recoveries()).isZero();
  }

  /**
   * A driver's heading reset facing the wrong way: the robot faces +x, the driver says -y. Solved
   * with that heading, its tags 3 m ahead at (4, 1) put the robot at (4, 4), turned about them, and
   * every such position agrees with the next, since they share the heading. They can't say the
   * position is wrong while it's the heading in doubt, so the estimate stays put until the cameras'
   * own headings turn it back; then their positions count again.
   */
  @Test
  void positionsSolvedWithADriversUnbackedHeadingDontMoveTheEstimate() {
    Robot robot = new Robot().drive(10, 0, 0, 0);
    robot.estimator.resetHeading(Rotation2d.fromDegrees(-90), Math.toRadians(5));
    Pose2d turnedAboutTheTags = new Pose2d(4, 4, Rotation2d.fromDegrees(-90));

    List<Boolean> leaning = new ArrayList<>();
    for (int i = 0; i < 3; i++) {
      robot.drive(8, 0, 0, 0);
      leaning.add(robot.seeLeaning(turnedAboutTheTags, new Translation2d(0, -3), 0.05).accepted());
    }
    Pose2d afterLeaning = robot.pose();
    for (int i = 0; i < 3; i++) {
      robot.drive(8, 0, 0, 0).seeWithHeading(START);
    }
    Pose2d turnedBack = robot.pose();
    VisionDecision solvedFacingRight =
        robot.drive(8, 0, 0, 0).seeLeaning(START, new Translation2d(3, 0), 0.05);

    assertThat(leaning).containsExactly(false, false, false);
    assertThat(afterLeaning.getTranslation().getDistance(START.getTranslation()))
        .as("where it stands")
        .isLessThan(1e-9);
    assertThat(turnedBack.getRotation().getDegrees()).isCloseTo(0, within(1e-6));
    assertThat(robot.estimate().health().recoveries()).isEqualTo(1);
    assertThat(solvedFacingRight.accepted()).isTrue();
    assertSame(robot.pose(), START, 0.01);
  }

  /**
   * Once a camera's own heading agrees with a driver's reset, positions solved with it say where
   * the robot is again: knocked a meter sideways, they agree it moved, and it recovers to them.
   */
  @Test
  void aResetACamerasHeadingBacksLetsItsPositionsRecoverAKnock() {
    Robot robot = new Robot().drive(10, 0, 0, 0);
    robot.estimator.resetHeading(Rotation2d.fromDegrees(2), Math.toRadians(5));
    assertThat(robot.drive(8, 0, 0, 0).seeWithHeading(START).accepted()).isTrue();
    Pose2d knocked = new Pose2d(1, 2, Rotation2d.ZERO);

    for (int i = 0; i < 3; i++) {
      robot.drive(8, 0, 0, 0).seeLeaning(knocked, new Translation2d(3, -1), 0.05);
    }

    assertThat(robot.estimate().health().recoveries()).isEqualTo(1);
    assertThat(robot.pose().getTranslation().getDistance(knocked.getTranslation()))
        .isLessThan(0.05);
  }

  @Test
  void onAKnownRampTheWheelsTravelCountsAtTheSlopesCosine() {
    double slope = Math.toRadians(15);
    Robot ramp = new Robot().under(new DriveConditions(slope, slope, 0, 0, false));
    Robot unknown = new Robot().under(tilted(15));

    ramp.drive(250, 1.0, 0, 0); // a meter along the ramp
    unknown.drive(250, 1.0, 0, 0);

    assertThat(ramp.pose().getX() - START.getX()).isCloseTo(Math.cos(slope), within(1e-9));
    assertThat(unknown.pose().getX() - START.getX()).as("halved").isCloseTo(0.5, within(1e-9));
    assertThat(ramp.estimate().health().suspectTravelMeters()).isZero();
  }

  @Test
  void theGroundExplainsSomeOrAllOfATiltNeverMore() {
    assertThatThrownBy(() -> new DriveConditions(0.1, 0.2, 0, 0, false))
        .hasMessageContaining("never more");
    assertThat(new DriveConditions(0.3, 0.2, 0, 0, false).unexplainedTiltRadians())
        .isCloseTo(0.1, within(1e-12));
  }

  /**
   * Built inside the robot's loop, the conditions don't stop it for a slip or a share of a
   * collision that can't be one: a slip that isn't a speed, zero or more, is none, and a share
   * outside zero to one is the nearest of them.
   */
  @Test
  void aSlipIsASpeedAndACollisionShareAtMostAWholeOne() {
    for (double notASpeed : new double[] {-0.1, Double.NaN, Double.POSITIVE_INFINITY}) {
      assertThat(new DriveConditions(0, 0, 0, notASpeed, false).slipMetersPerSecond()).isZero();
    }
    assertThat(new DriveConditions(0, 0, 1.5, 0, false).collisionShare()).isOne();
    assertThat(new DriveConditions(0, 0, -0.5, 0, false).collisionShare()).isZero();
    assertThat(new DriveConditions(0, 0, Double.NaN, 0, false).collision()).isFalse();
  }

  @Test
  void settingsMustMakeSense() {
    assertThatThrownBy(() -> with().translationStdDevPerMeter(-1).build())
        .hasMessage("EstimatorConfig.translationStdDevPerMeter: -1 must be zero or more");
    assertThatThrownBy(() -> with().historySeconds(0).build())
        .hasMessage("EstimatorConfig.historySeconds: 0 must be positive");
    assertThatThrownBy(() -> with().recoveryMeasurements(1).build())
        .hasMessage("EstimatorConfig.recoveryMeasurements: 1 must be at least 2");
    assertThatThrownBy(() -> with().tiltStartDegrees(30).build())
        .hasMessageStartingWith(
            "EstimatorConfig.tiltZeroDegrees: 25 must be above tiltStartDegrees (30)");
    assertThatThrownBy(() -> with().gateChiSquaredPose(0).recoveryStdDevMeters(-1).build())
        .as("each value is named")
        .hasMessage(
            "EstimatorConfig.gateChiSquaredPose: 0 must be positive\n"
                + "EstimatorConfig.recoveryStdDevMeters: -1 must be positive");
  }

  /** A gyro that sends NaN would poison the heading for good: it counts as lost instead. */
  @Test
  void aGyroYawThatIsntANumberCountsAsLost() {
    Robot robot = new Robot().drive(25, 1, 0, 0);
    robot.time += STEP;
    robot.estimator.addOdometry(
        robot.time, new BodyMotion(0.004, 0, 0, 0), Double.NaN, YawSource.GYRO);

    assertThat(robot.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.IMU_FAULT);
    robot.drive(25, 1, 0, 0);
    assertThat(robot.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.DEAD_RECKONING);
    assertThat(robot.pose().getRotation().getRadians()).isCloseTo(0, within(1e-9));
    assertThat(robot.pose().getX()).isCloseTo(START.getX() + 0.204, within(1e-6));
    assertThat(robot.estimate().covariance().largestPositionStdDev()).isFinite();
  }

  /** A measurement that isn't a number fails the gate; it doesn't pass it. */
  @Test
  void aMeasurementThatIsntANumberIsRejected() {
    Robot robot = new Robot().drive(25, 0, 0, 0);

    List<VisionDecision> decisions =
        robot.see(
            new Pose2d(Double.NaN, Double.NaN, Rotation2d.ZERO),
            robot.time,
            PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY));

    assertThat(decisions).singleElement().satisfies(d -> assertThat(d.accepted()).isFalse());
    assertThat(robot.pose().getTranslation()).isEqualTo(START.getTranslation());
  }

  private static EstimatorConfigBuilder with() {
    return EstimatorConfigBuilder.builder(EstimatorConfig.DEFAULTS);
  }

  /** What the estimate rests on, in order: each state overrides the ones after it. */
  @Test
  void theModeIsTheFirstThatApplies() {
    assertThat(PoseEstimator.mode(false, false, true, true))
        .isEqualTo(LocalizationEstimate.Mode.IMU_FAULT);
    assertThat(PoseEstimator.mode(true, false, true, true))
        .isEqualTo(LocalizationEstimate.Mode.INITIALIZING);
    assertThat(PoseEstimator.mode(true, true, true, true))
        .isEqualTo(LocalizationEstimate.Mode.LOST);
    assertThat(PoseEstimator.mode(true, true, false, true))
        .isEqualTo(LocalizationEstimate.Mode.TRACKING);
    assertThat(PoseEstimator.mode(true, true, false, false))
        .isEqualTo(LocalizationEstimate.Mode.DEAD_RECKONING);
  }

  /**
   * A field of more tags than 2026's 32: every one of them gets its place, so two cameras seeing
   * tag 40 share its placement, and can't place the robot along it better than it. A tag the field
   * doesn't have can't get one: each image's placement is taken as new, the cameras average it
   * down, and the estimator says which tag it was.
   */
  @Test
  void everyTagOfALargerFieldIsKeptApart() {
    Set<Integer> forty = tags(40);
    double[] alongTheTag = new double[2];
    List<List<Integer>> withoutPlaces = new ArrayList<>();
    int[] seen = {40, 41};
    for (int which = 0; which < 2; which++) {
      PoseEstimator estimator =
          new PoseEstimator(
              EstimatorConfig.DEFAULTS, point -> true, forty, new PoseReset(0, START, PLACED, "x"));
      long time = T0;
      estimator.addOdometry(time, new BodyMotion(0, 0, 0, 0), 0, YawSource.GYRO);
      estimator.reset(new PoseReset(time, START, PLACED, "placed"));
      for (int i = 0; i < 1000; i++) {
        time += STEP;
        estimator.addOdometry(time, new BodyMotion(0, 0, 0, 0), 0, YawSource.GYRO);
        estimator.addVision(
            new VisionMeasurement(
                time,
                START,
                position(0.05),
                new VisionMeasurement.Lever(START.getRotation(), new Translation2d(3, 0)),
                i % 2 == 0 ? "left" : "right",
                List.of(seen[which]),
                VisionMeasurement.Solver.HEADING_CONSTRAINED));
        estimator.update();
      }
      alongTheTag[which] = estimator.estimate().covariance().xStdDev();
      withoutPlaces.add(estimator.tagsWithoutPlaces());
    }

    double tag = EstimatorConfig.DEFAULTS.tagPlacementStdDevMeters();
    double shift = EstimatorConfig.DEFAULTS.cameraShiftStdDevMeters();
    double shared = Math.hypot(tag, shift / Math.sqrt(2));
    assertThat(alongTheTag[0]).as("tag 40, kept apart").isCloseTo(shared, within(0.05 * shared));
    assertThat(withoutPlaces.get(0)).isEmpty();
    assertThat(alongTheTag[1]).as("tag 41, off the field").isLessThan(0.5 * shared);
    assertThat(withoutPlaces.get(1)).containsExactly(41);
  }

  /** A field's tags, numbered from 1. */
  private static Set<Integer> tags(int count) {
    return IntStream.rangeClosed(1, count).boxed().collect(Collectors.toSet());
  }
}
