package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * When the estimate goes where the cameras say, against odometry, and when it stays: one row for
 * each way a robot comes to be somewhere its wheels did not take it, or seems to. Under the default
 * configuration, a robot driving along the field at a meter a second, its placement already backed
 * by a camera, then something happens and the cameras see it for a third of a second.
 *
 * <p>The rule: rejected images that agree move the estimate only when they agree for independent
 * reasons, which are two cameras, two tags, or a sign from the IMU or the robot's state that it may
 * have been moved unseen. A camera misreading one tag as another agrees with itself in every image,
 * the same way a shove the wheels never felt would, and on its own moves nothing.
 */
class RecoveryCasesTest {
  private static final long STEP = 4_000_000L; // 250 Hz
  private static final int STEPS_A_LOOP = 5; // a 20 ms loop
  private static final long T0 = 1_000_000_000L;
  private static final Pose2d START = new Pose2d(2, 2, Rotation2d.ZERO);
  private static final double SPEED = 1.0;

  /** As far as a misread of a tag for its neighbor put the estimate in the estimator study. */
  private static final Translation2d MISREAD = new Translation2d(0, 2.65);

  private static final Translation2d SHOVE = new Translation2d(0, 0.6);

  private static final DriveConditions HIT = new DriveConditions(0, 0, 1, 0, false);
  private static final DriveConditions SLIDING = new DriveConditions(0, 0, 0, 0.5, false);
  private static final DriveConditions DISABLED = new DriveConditions(0, 0, 0, 0, true);
  private static final PoseCovariance PLACED = PoseCovariance.fromStdDevs(0.05, 0.05, 0.02);

  /** Whether the estimate should end where the cameras say, or stay on odometry. */
  enum Outcome {
    GOES_TO_THE_CAMERAS,
    STAYS_ON_ODOMETRY
  }

  /** Whether an image taken just after the event agreed with the estimate. */
  enum RightAfter {
    NO_IMAGE,
    AN_IMAGE_AGREES
  }

  /** Which cameras see the robot after the event, and from which tags. */
  enum Seen {
    ONE_CAMERA_ONE_TAG,
    ONE_CAMERA_TWO_TAGS,
    TWO_CAMERAS
  }

  /**
   * One way the robot comes to be somewhere else, or seems to.
   *
   * @param name what happens
   * @param first what the robot's other sensors say in the loop it happens
   * @param after and in the loops after it
   * @param shove how far the robot is moved, all at once, without its wheels turning
   * @param misread how far the cameras put it from where it really is
   * @param wheelsAhead how fast its wheels run ahead of it, in meters a second
   * @param placedWrong how far from where it is the robot is told it was placed, just before
   * @param rightAfter whether an image taken just after the event agreed with the estimate
   * @param seen which cameras see it, and from which tags
   * @param outcome where the estimate should end
   */
  record Case(
      String name,
      DriveConditions first,
      DriveConditions after,
      Translation2d shove,
      Translation2d misread,
      double wheelsAhead,
      Translation2d placedWrong,
      RightAfter rightAfter,
      Seen seen,
      Outcome outcome) {
    @Override
    public String toString() {
      return name;
    }
  }

  private static final Translation2d NONE = Translation2d.ZERO;
  private static final DriveConditions NORMAL = DriveConditions.NORMAL;

  static Stream<Case> cases() {
    return Stream.of(
        new Case(
            "a lone camera repeating a misread, nothing felt",
            NORMAL,
            NORMAL,
            NONE,
            MISREAD,
            0,
            NONE,
            RightAfter.NO_IMAGE,
            Seen.ONE_CAMERA_ONE_TAG,
            Outcome.STAYS_ON_ODOMETRY),
        new Case(
            "a lone camera misreading after a hit an image after it agreed with",
            HIT,
            NORMAL,
            NONE,
            MISREAD,
            0,
            NONE,
            RightAfter.AN_IMAGE_AGREES,
            Seen.ONE_CAMERA_ONE_TAG,
            Outcome.STAYS_ON_ODOMETRY),
        new Case(
            "an unfelt shove one camera sees on one tag: no different from a misread",
            NORMAL,
            NORMAL,
            SHOVE,
            NONE,
            0,
            NONE,
            RightAfter.NO_IMAGE,
            Seen.ONE_CAMERA_ONE_TAG,
            Outcome.STAYS_ON_ODOMETRY),
        new Case(
            "an unfelt shove one camera sees on two tags",
            NORMAL,
            NORMAL,
            SHOVE,
            NONE,
            0,
            NONE,
            RightAfter.NO_IMAGE,
            Seen.ONE_CAMERA_TWO_TAGS,
            Outcome.GOES_TO_THE_CAMERAS),
        new Case(
            "an unfelt shove two cameras agree on",
            NORMAL,
            NORMAL,
            SHOVE,
            NONE,
            0,
            NONE,
            RightAfter.NO_IMAGE,
            Seen.TWO_CAMERAS,
            Outcome.GOES_TO_THE_CAMERAS),
        new Case(
            "a shove the IMU felt, a lone camera on one tag",
            HIT,
            NORMAL,
            SHOVE,
            NONE,
            0,
            NONE,
            RightAfter.NO_IMAGE,
            Seen.ONE_CAMERA_ONE_TAG,
            Outcome.GOES_TO_THE_CAMERAS),
        new Case(
            "carried a meter while disabled, a lone camera on one tag",
            DISABLED,
            DISABLED,
            new Translation2d(0, 1),
            NONE,
            0,
            NONE,
            RightAfter.NO_IMAGE,
            Seen.ONE_CAMERA_ONE_TAG,
            Outcome.GOES_TO_THE_CAMERAS),
        new Case(
            "wheels sliding, odometry running ahead, a lone camera on one tag",
            SLIDING,
            SLIDING,
            NONE,
            NONE,
            0.5,
            NONE,
            RightAfter.NO_IMAGE,
            Seen.ONE_CAMERA_ONE_TAG,
            Outcome.GOES_TO_THE_CAMERAS),
        new Case(
            "placed a meter wrong, a lone camera on one tag",
            NORMAL,
            NORMAL,
            NONE,
            NONE,
            0,
            new Translation2d(0, -1),
            RightAfter.NO_IMAGE,
            Seen.ONE_CAMERA_ONE_TAG,
            Outcome.GOES_TO_THE_CAMERAS));
  }

  @ParameterizedTest
  @MethodSource("cases")
  void theEstimateGoesWhereIndependentEvidenceSays(Case row) {
    Robot robot = new Robot();
    // A second's driving, every image agreeing: the placement is backed by a camera.
    for (int loop = 0; loop < 50; loop++) {
      robot.loop(NORMAL, 0);
      robot.see("front", List.of(26), NONE);
    }
    assertThat(robot.estimator.estimate().health().rejected()).isZero();

    robot.placeWrongBy(row.placedWrong());
    robot.truth = robot.truth.plus(row.shove());
    robot.loop(row.first(), row.wheelsAhead());
    if (row.rightAfter() == RightAfter.AN_IMAGE_AGREES) {
      robot.see("front", List.of(26), NONE);
    }
    for (int loop = 0; loop < 25; loop++) {
      robot.loop(row.after(), row.wheelsAhead());
      robot.seeAs(row.seen(), loop, row.misread());
    }

    Translation2d cameras = robot.truth.plus(row.misread());
    Translation2d expected =
        row.outcome() == Outcome.GOES_TO_THE_CAMERAS ? cameras : robot.odometry;
    assertThat(robot.estimator.pose().getTranslation().getDistance(expected))
        .as("ends where it should")
        .isLessThan(0.05);
  }

  /**
   * A robot driven by hand along the field: its wheels and gyro, where its odometry puts it, and
   * where it really is.
   */
  private static final class Robot {
    final PoseEstimator estimator;
    long time = T0;
    final double gyroYaw = 0.4;

    /** Where the robot really is. */
    Translation2d truth = START.getTranslation();

    /** Where the wheels alone put it, from where it was told it was placed. */
    Translation2d odometry = START.getTranslation();

    Robot() {
      estimator =
          new PoseEstimator(
              EstimatorConfig.DEFAULTS,
              point -> true,
              List.of(25, 26, 27, 28),
              new PoseReset(0, START, PLACED, "start"));
      estimator.addOdometry(time, new BodyMotion(0, 0, 0, 0), gyroYaw, YawSource.GYRO);
      estimator.reset(new PoseReset(time, START, PLACED, "placed"));
    }

    /** Tells it it was placed this far from where it is; nothing when it is no distance. */
    void placeWrongBy(Translation2d off) {
      if (off.getNorm() == 0) {
        return;
      }
      odometry = truth.plus(off);
      estimator.reset(
          new PoseReset(time, new Pose2d(odometry, Rotation2d.ZERO), PLACED, "placed wrong"));
    }

    /** One loop's odometry: the wheels say a meter a second, and run ahead of the robot by this. */
    void loop(DriveConditions conditions, double wheelsAhead) {
      estimator.setConditions(conditions);
      double dt = STEP / 1e9;
      for (int i = 0; i < STEPS_A_LOOP; i++) {
        time += STEP;
        estimator.addOdometry(time, new BodyMotion(SPEED * dt, 0, 0, 0), gyroYaw, YawSource.GYRO);
        odometry = odometry.plus(new Translation2d(SPEED * dt, 0));
        truth = truth.plus(new Translation2d((SPEED - wheelsAhead) * dt, 0));
      }
    }

    void seeAs(Seen seen, int loop, Translation2d misread) {
      boolean even = loop % 2 == 0;
      switch (seen) {
        case ONE_CAMERA_ONE_TAG -> see("front", List.of(26), misread);
        case ONE_CAMERA_TWO_TAGS -> see("front", even ? List.of(26) : List.of(27), misread);
        case TWO_CAMERAS -> see(even ? "front" : "back", List.of(26), misread);
      }
    }

    /**
     * A camera's position, solved with the gyro's heading, good to 2 cm, from tags 3 m ahead: where
     * the robot is, off by a misread.
     */
    void see(String camera, List<Integer> tags, Translation2d misread) {
      Pose2d pose = new Pose2d(truth.plus(misread), Rotation2d.ZERO);
      estimator.addVision(
          new VisionMeasurement(
              time,
              pose,
              PoseCovariance.fromStdDevs(0.02, 0.02, Double.POSITIVE_INFINITY),
              new VisionMeasurement.Lever(Rotation2d.ZERO, new Translation2d(3, 0)),
              camera,
              tags,
              VisionMeasurement.Solver.HEADING_CONSTRAINED));
      estimator.update();
    }
  }
}
