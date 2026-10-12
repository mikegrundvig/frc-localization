package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatCode;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.localization.LocalizerBench.Option;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The contract a localizer keeps ({@link Localizer}), as tests: every localizer passes every one of
 * these, so "the robot program can be driven by this localizer" is something a build checks.
 *
 * <p>A localizer's test class extends this and hands it the localizer's bench ({@link
 * LocalizerBench}). The scenarios here know the localizer only through {@link Localizer}: they give
 * it what a robot gives it, loop by loop in the robot's order, and check what it answers. Each is
 * named for the clauses it checks.
 *
 * <p><b>The robot in the scenarios</b> is scripted: a swerve drive whose wheels and gyro read
 * exactly what it does, so odometry alone is the truth, and where the robot really is is known.
 * That makes the scenarios about the contract, and never about how well a localizer copes with bad
 * sensors: nothing here asks a localizer to be accurate.
 *
 * <p><b>The options.</b> A scenario that needs an option the bench says the localizer lacks is
 * skipped, saying why. One scenario checks that a localizer reports nothing for each option it
 * lacks.
 */
public abstract class LocalizerConformance {
  private static final FieldModel FIELD =
      new FieldModel(
          Fields.FRC_2026_REBUILT_WELDED.loadField(),
          Symmetry.ROTATED,
          List.of(),
          List.of(),
          List.of());

  /** A forward camera at the front of the robot, tilted up 15 degrees. */
  private static final CameraConfig FRONT =
      CameraConfig.builder()
          .name("front")
          .forward(Inches.of(12))
          .left(Inches.of(0))
          .up(Inches.of(12))
          .roll(Degrees.of(0))
          .tiltUp(Degrees.of(15))
          .yaw(Degrees.of(0))
          .sensor(
              CameraConfig.Sensor.builder()
                  .widthPixels(1280)
                  .heightPixels(800)
                  .fieldOfView(Degrees.of(80))
                  .fieldOfViewAcross(CameraConfig.Across.DIAGONAL)
                  .frameRate(Hertz.of(30))
                  .latency(Milliseconds.of(30))
                  .sightRange(Meters.of(7))
                  .build())
          .noiseScale(1.0)
          .build();

  /** A 28 x 24 in frame, 60 kg. */
  private static final Body BODY =
      Body.builder()
          .frameLength(Inches.of(28))
          .frameWidth(Inches.of(24))
          .height(Inches.of(30))
          .bumperEnds(Inches.of(3.5))
          .bumperSides(Inches.of(3.5))
          .bumperBottom(Inches.of(2.5))
          .bumperTop(Inches.of(5.75))
          .weight(Body.Weight.weighed(Kilograms.of(60)))
          .inertia(
              Body.Turning.tensor(
                  Inertia.builder().ixx(6).iyy(6).izz(6).ixy(0).ixz(0).iyz(0).build()))
          .cg(new Translation3d(0, 0, 0.2))
          .build();

  /** The suite's robot: a 28 x 24 in swerve drive with the front camera. */
  private static final RobotDefinition ROBOT =
      RobotDefinition.builder()
          .name("conformance")
          .body(BODY)
          .drivetrain(
              DrivetrainGeometry.inset(
                  DrivetrainGeometry.ModuleType.builder()
                      .driveMotor(Motor.KRAKEN_X60)
                      .steerMotor(Motor.KRAKEN_X44)
                      .wheelRadius(Inches.of(2))
                      .driveGearing(Gearing.builder().ratio(6.75).efficiency(1.0).build())
                      .steerGearing(Gearing.builder().ratio(12.8).efficiency(1.0).build())
                      .couplingRatio(0)
                      .wheelCoefficientOfFriction(1.2)
                      .driveInertia(KilogramSquareMeters.of(0.001))
                      .steerInertia(KilogramSquareMeters.of(0.004))
                      .build(),
                  BODY,
                  Inches.of(2.625)))
          .imus(Imus.UNCHECKED)
          .cameras(List.of(FRONT))
          .build();

  /** Where the robot starts: on open floor, with room to drive two meters any way. */
  private static final Pose2d START = new Pose2d(4, 4, Rotation2d.fromDegrees(20));

  /** What the gyro reads at the start: not zero, as a gyro seldom is. */
  private static final double GYRO_AT_START = 0.3;

  private static final long T0 = 1_000_000_000L;
  private static final long LOOP_NANOS = 20_000_000L;

  /** A placement on the start, as sure as a drive team's. */
  private static final PoseCovariance PLACED =
      PoseCovariance.fromStdDevs(0.05, 0.05, Math.toRadians(2));

  /** How close a pose from exact odometry must be to the truth, in meters and in radians. */
  private static final double ODOMETRY_METERS = 0.02;

  private static final double ODOMETRY_RADIANS = Math.toRadians(1);

  /** How close two poses that should be the same pose must be. */
  private static final double SAME = 1e-6;

  private final LocalizerBench bench;

  protected LocalizerConformance(LocalizerBench bench) {
    this.bench = bench;
  }

  // ---- The pose ----

  /** L3. */
  @Test
  void l3_thePoseIsANumberBeforeAnythingIsKnown() {
    Localizer localizer = bench.make(ROBOT, FIELD);

    assertNumbers(localizer.pose());
    assertThat(localizer.timestampNanos()).as("L4: no wheel reading yet").isZero();
  }

  /** L2. */
  @Test
  void l2_theSameCallsGiveTheSameAnswers() {
    Run first = placed();
    Run second = placed();
    for (int loop = 0; loop < 100; loop++) {
      ChassisVelocities velocity = new ChassisVelocities(1.0, 0.3, 0.5);
      first.loop(velocity);
      second.loop(velocity);
      if (loop % 3 == 0) {
        first.measure(Transform2d.ZERO, 0.05, 0);
        second.measure(Transform2d.ZERO, 0.05, 0);
      }
      first.close();
      second.close();
    }

    assertThat(bits(first.localizer.pose())).isEqualTo(bits(second.localizer.pose()));
  }

  /** L4. */
  @Test
  void l4_theTimeIsTheNewestWheelReadingsAndNeverRunsBackwards() {
    Run run = placed();
    long before = run.localizer.timestampNanos();
    for (int loop = 0; loop < 50; loop++) {
      DrivetrainReadings readings = run.loop(new ChassisVelocities(1, 0, 0));
      run.close();

      long newest = readings.wheels().get(readings.wheels().size() - 1).timestampNanos();
      assertThat(run.localizer.timestampNanos()).isEqualTo(newest).isGreaterThan(before);
      before = newest;
    }
  }

  /** L5. */
  @Test
  void l5_thePoseAtAMomentReachesBackHalfASecondAndNoFartherThanItKeeps() {
    Run run = placed();
    List<Pose2d> truths = new ArrayList<>();
    for (int loop = 0; loop < 100; loop++) {
      run.loop(new ChassisVelocities(1, 0, 0.5));
      run.close();
      truths.add(run.robot.truth());
    }
    long newest = run.localizer.timestampNanos();

    assertThat(run.localizer.poseAt(newest))
        .as("at the newest reading it is the pose")
        .hasValueSatisfying(pose -> assertSame(pose, run.localizer.pose()));
    int loopsBack = 25;
    Optional<Pose2d> halfASecondAgo = run.localizer.poseAt(newest - loopsBack * LOOP_NANOS);
    assertThat(halfASecondAgo).as("half a second back").isPresent();
    assertNear(halfASecondAgo.get(), truths.get(truths.size() - 1 - loopsBack));
    assertThat(run.localizer.poseAt(T0 - 60_000_000_000L))
        .as("a minute before it started")
        .isEmpty();
    assertThat(run.localizer.poseAt(newest + 5_000_000_000L)).as("five seconds from now").isEmpty();
  }

  // ---- Odometry ----

  /** L6. */
  @Test
  void l6_withOdometryAloneThePoseMovesAsTheWheelsAndTheGyroSay() {
    Run run = placed();
    for (int loop = 0; loop < 100; loop++) {
      run.loop(new ChassisVelocities(1.0, 0.5, 1.0));
      run.close();
    }
    assertNear(run.localizer.pose(), run.robot.truth());

    for (int loop = 0; loop < 50; loop++) {
      run.loop(new ChassisVelocities());
      run.close();
    }
    Pose2d stopped = run.localizer.pose();
    for (int loop = 0; loop < 50; loop++) {
      run.loop(new ChassisVelocities());
      run.close();
    }
    assertThat(distance(run.localizer.pose(), stopped)).as("standing still").isLessThan(1e-3);
  }

  /** L7. */
  @Test
  void l7_theFirstReadingsMoveNothing() {
    ScriptedDrivetrain robot =
        new ScriptedDrivetrain(ROBOT.moduleTranslations(), START, T0 - LOOP_NANOS, GYRO_AT_START);
    // The wheels have rolled and the gyro has turned before the localizer is made.
    for (int loop = 0; loop < 50; loop++) {
      robot.loop(new ChassisVelocities(1, 0, 1));
    }
    for (int loop = 0; loop < 50; loop++) {
      robot.loop(new ChassisVelocities());
    }
    Localizer localizer = bench.make(ROBOT, FIELD);
    Pose2d before = localizer.pose();

    for (int loop = 0; loop < 10; loop++) {
      localizer.odometry(robot.loop(new ChassisVelocities()), new LoopContext(robot.now(), true));
      localizer.measurements(List.of());
    }

    assertThat(distance(localizer.pose(), before)).as("a robot standing still").isLessThan(1e-3);
    assertThat(Math.abs(localizer.pose().getRotation().minus(before.getRotation()).getRadians()))
        .as("its heading")
        .isLessThan(1e-3);
  }

  /** L8. */
  @Test
  void l8_aLoopWithNoReadingsAWheelThatIsNotANumberAndALostGyroAreOrdinary() {
    Run run = placed();
    for (int loop = 0; loop < 25; loop++) {
      run.loop(new ChassisVelocities(1, 0, 0));
      run.close();
    }

    // A loop the drivetrain's thread sent nothing in.
    DrivetrainReadings nothing =
        DrivetrainReadings.builder()
            .wheels(List.of())
            .gyro(ImuReadings.NONE)
            .backupImu(ImuReadings.NONE)
            .velocities(new ChassisVelocities())
            .build();
    assertThatCode(
            () -> {
              run.localizer.odometry(nothing, new LoopContext(run.robot.now(), true));
              run.localizer.measurements(List.of());
            })
        .doesNotThrowAnyException();
    assertNumbers(run.localizer.pose());

    // One wheel off the bus for a loop: the other three still say how the robot moved.
    DrivetrainReadings whole = run.robot.loop(new ChassisVelocities(1, 0, 0));
    List<OdometrySample> missingAWheel = new ArrayList<>();
    for (OdometrySample sample : whole.wheels()) {
      double[] distances = new double[sample.wheels()];
      double[] angles = new double[sample.wheels()];
      for (int wheel = 0; wheel < sample.wheels(); wheel++) {
        distances[wheel] = wheel == 0 ? Double.NaN : sample.distanceMeters(wheel);
        angles[wheel] = wheel == 0 ? Double.NaN : sample.angleRadians(wheel);
      }
      missingAWheel.add(new OdometrySample(sample.timestampNanos(), distances, angles));
    }
    run.localizer.odometry(
        DrivetrainReadingsBuilder.builder(whole).wheels(missingAWheel).build(),
        new LoopContext(run.robot.now(), true));
    run.close();
    assertNumbers(run.localizer.pose());

    // The gyro is lost: the backup IMU, whose zero is its own, carries on without a jump.
    run.robot.gyroLost = true;
    Pose2d beforeTheLoss = run.localizer.pose();
    run.loop(new ChassisVelocities(1, 0, 0));
    run.close();
    assertThat(distance(run.localizer.pose(), beforeTheLoss))
        .as("one loop at 1 m/s, the backup IMU standing in")
        .isLessThan(0.1);
    assertThat(
            Math.abs(
                run.localizer.pose().getRotation().minus(beforeTheLoss.getRotation()).getRadians()))
        .as("the backup IMU's own zero is not a turn")
        .isLessThan(Math.toRadians(5));

    // Both lost: the wheels' own turning, and still a number.
    run.robot.backupLost = true;
    for (int loop = 0; loop < 25; loop++) {
      run.loop(new ChassisVelocities(1, 0, 0.5));
      run.close();
    }
    assertNumbers(run.localizer.pose());
  }

  // ---- Placing the robot ----

  /** L9. */
  @Test
  void l9_afterAPlacementThePoseIsThePlacedPoseAndOdometryCarriesOn() {
    Run run = placed();
    for (int loop = 0; loop < 25; loop++) {
      run.loop(new ChassisVelocities(1, 0, 0.5));
      run.close();
    }
    Pose2d elsewhere = new Pose2d(7, 2, Rotation2d.fromDegrees(-100));

    run.loop(new ChassisVelocities(1, 0, 0.5));
    Pose2d truthThen = run.robot.truth();
    run.localizer.place(new PoseReset(run.robot.now(), elsewhere, PLACED, "placed elsewhere"));
    assertSame(run.localizer.pose(), elsewhere);
    run.close();
    for (int loop = 0; loop < 50; loop++) {
      run.loop(new ChassisVelocities(1, 0, 0.5));
      run.close();
    }

    // The robot moved the same way from the placed pose as it really moved from where it stood.
    Transform2d moved = new Transform2d(truthThen, run.robot.truth());
    assertNear(run.localizer.pose(), elsewhere.plus(moved));
  }

  /** L9. */
  @Test
  void l9_aPlacementThatSaysNothingOfTheHeadingKeepsTheHeading() {
    Run run = placed();
    run.loop(new ChassisVelocities());
    Rotation2d heading = run.localizer.pose().getRotation();
    Pose2d elsewhere = new Pose2d(7, 2, Rotation2d.fromDegrees(-100));

    run.localizer.place(
        new PoseReset(
            run.robot.now(),
            elsewhere,
            PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY),
            "position only"));

    assertThat(distance(run.localizer.pose(), elsewhere)).isLessThan(SAME);
    assertThat(Math.abs(run.localizer.pose().getRotation().minus(heading).getRadians()))
        .as("the heading stays")
        .isLessThan(SAME);
  }

  /** L10. */
  @Test
  void l10_afterAHeadingResetThePoseFacesTheHeadingWhereItStood() {
    Run run = placed();
    for (int loop = 0; loop < 25; loop++) {
      run.loop(new ChassisVelocities(1, 0, 0));
      run.close();
    }
    run.loop(new ChassisVelocities());
    Pose2d before = run.localizer.pose();
    Rotation2d downfield = Rotation2d.fromDegrees(180);

    run.localizer.resetHeading(downfield, Math.toRadians(5));

    assertThat(distance(run.localizer.pose(), before))
        .as("it stands where it stood")
        .isLessThan(SAME);
    assertThat(Math.abs(run.localizer.pose().getRotation().minus(downfield).getRadians()))
        .as("it faces the heading")
        .isLessThan(SAME);
  }

  // ---- Measurements ----

  /** L11, L12. */
  @Test
  void l11_aMeasurementThatAgreesIsUsedAndNeverOvershot() {
    Run run = placed();
    for (int loop = 0; loop < 10; loop++) {
      run.loop(new ChassisVelocities());
      run.close();
    }
    // Three centimeters off, from a measurement sure to five: within its own uncertainty.
    Transform2d off = new Transform2d(0.03, 0, Rotation2d.ZERO);
    Pose2d measured = run.robot.truth().plus(off);
    double before = distance(run.localizer.pose(), measured);

    double last = before;
    for (int loop = 0; loop < 100; loop++) {
      run.loop(new ChassisVelocities());
      Pose2d held = run.localizer.pose();
      run.measure(off, 0.05, 0);
      run.close();
      double now = distance(run.localizer.pose(), measured);
      // L12: never away from the measurement, and never past it.
      assertThat(now).as("loop %d: toward the measurement", loop).isLessThanOrEqualTo(last + SAME);
      assertThat(distance(run.localizer.pose(), held))
          .as("loop %d: no farther than the measurement", loop)
          .isLessThanOrEqualTo(distance(held, measured) + SAME);
      last = now;
    }

    assertThat(last).as("the estimate closes on it").isLessThan(before / 2);
  }

  /** L13. */
  @Test
  void l13_aMeasurementIsAppliedAtItsCaptureTime() {
    Run run = placed();
    List<Pose2d> truths = new ArrayList<>();
    List<Long> times = new ArrayList<>();
    for (int loop = 0; loop < 50; loop++) {
      run.loop(new ChassisVelocities(2, 0, 0));
      run.close();
      truths.add(run.robot.truth());
      times.add(run.robot.now());
    }

    // An image taken 0.2 s ago, of exactly where the robot was then: 0.4 m back along its path.
    int back = 10;
    run.loop(new ChassisVelocities(2, 0, 0));
    run.localizer.measurements(
        List.of(
            measurement(
                times.get(times.size() - back), truths.get(truths.size() - back), 0.02, 0.02)));

    assertThat(distance(run.localizer.pose(), run.robot.truth()))
        .as("applied where the robot was then, the estimate stays where the robot is now")
        .isLessThan(0.05);
  }

  /** L14. */
  @Test
  void l14_aMeasurementWithNoHeadingDoesNotTurnTheEstimate() {
    Run run = placed();
    for (int loop = 0; loop < 10; loop++) {
      run.loop(new ChassisVelocities());
      run.close();
    }
    Rotation2d heading = run.localizer.pose().getRotation();

    for (int loop = 0; loop < 25; loop++) {
      run.loop(new ChassisVelocities());
      Pose2d turned =
          new Pose2d(
              run.robot.truth().getTranslation(),
              run.robot.truth().getRotation().plus(Rotation2d.fromDegrees(30)));
      run.localizer.measurements(
          List.of(measurement(run.robot.now(), turned, 0.05, Double.POSITIVE_INFINITY)));
    }

    assertThat(Math.abs(run.localizer.pose().getRotation().minus(heading).getRadians()))
        .as("thirty degrees off, with nothing to say of the heading")
        .isLessThan(1e-3);
  }

  /** L15, L16. */
  @Test
  void l16_measurementsOutOfOrderTooOldOrFromTheFutureCostNothingButThemselves() {
    Localizer fresh = bench.make(ROBOT, FIELD);
    assertThatCode(() -> fresh.measurements(List.of(measurement(T0, START, 0.05, 0.05))))
        .as("before any wheel reading")
        .doesNotThrowAnyException();
    assertNumbers(fresh.pose());

    Run run = placed();
    for (int loop = 0; loop < 50; loop++) {
      run.loop(new ChassisVelocities(1, 0, 0));
      run.close();
    }
    run.loop(new ChassisVelocities(1, 0, 0));
    Pose2d before = run.localizer.pose();
    long now = run.robot.now();
    Pose2d truth = run.robot.truth();
    Pose2d farOff = truth.plus(new Transform2d(3, 3, Rotation2d.ZERO));

    assertThatCode(
            () ->
                run.localizer.measurements(
                    List.of(
                        // Newest first, which is not the order they were taken in.
                        measurement(now, truth, 0.05, 0.05),
                        measurement(now - 2 * LOOP_NANOS, truth, 0.05, 0.05),
                        // Ten minutes old: older than any localizer keeps.
                        measurement(1, farOff, 0.05, 0.05),
                        // Stamped an hour from now.
                        measurement(now + 3_600_000_000_000L, truth, 0.05, 0.05))))
        .doesNotThrowAnyException();

    assertNumbers(run.localizer.pose());
    assertThat(distance(run.localizer.pose(), before))
        .as("the old one, three meters off, moved nothing")
        .isLessThan(0.1);
  }

  // ---- Cost ----

  /** L17. */
  @Test
  void l17_whatALoopCostsDoesNotGrowWithHowLongTheRobotHasRun() {
    Run run = placed();
    com.sun.management.ThreadMXBean threads =
        (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
    int loopsABatch = 1500;
    // The first batch warms the code up. A minute of loops then stands beside the minute after.
    long[] bytes = new long[3];
    for (int batch = 0; batch < bytes.length; batch++) {
      for (int loop = 0; loop < loopsABatch; loop++) {
        DrivetrainReadings readings = run.robot.loop(new ChassisVelocities(0.5, 0, 1.0));
        LoopContext context = new LoopContext(run.robot.now(), true);
        List<VisionMeasurement> taken =
            List.of(measurement(run.robot.now(), run.robot.truth(), 0.05, 0.05));
        long before = threads.getCurrentThreadAllocatedBytes();
        run.localizer.odometry(readings, context);
        run.localizer.measurements(taken);
        run.localizer.pose();
        bytes[batch] += threads.getCurrentThreadAllocatedBytes() - before;
      }
    }

    double earlier = (double) bytes[1] / loopsABatch;
    double later = (double) bytes[2] / loopsABatch;
    assertThat(later)
        .as("bytes a loop, a minute later (%.0f then, %.0f now)", earlier, later)
        .isLessThan(1.5 * earlier + 1024);
  }

  // ---- The options ----

  /** L18. */
  @Test
  void l18_anOptionIsReportedInEveryLoopOrNever() {
    Run run = placed();
    for (int loop = 0; loop < 100; loop++) {
      run.loop(new ChassisVelocities(1, 0, 0.5));
      for (Option option : Option.values()) {
        assertThat(reported(run.localizer, option).isPresent())
            .as("%s, loop %d, after odometry: %s", option, loop, bench.name())
            .isEqualTo(has(option));
      }
      if (loop % 4 == 0) {
        run.measure(Transform2d.ZERO, 0.05, 0);
      }
      run.close();
      for (Option option : Option.values()) {
        assertThat(reported(run.localizer, option).isPresent())
            .as("%s, loop %d, after its measurements: %s", option, loop, bench.name())
            .isEqualTo(has(option));
      }
    }
  }

  /** L19. */
  @Test
  void l19_theCovarianceGrowsWhileOnlyOdometryComesIn() {
    needs(Option.COVARIANCE);
    Run run = placed();
    run.loop(new ChassisVelocities());
    run.close();
    double before = run.localizer.covariance().orElseThrow().largestPositionStdDev();

    for (int loop = 0; loop < 250; loop++) {
      run.loop(new ChassisVelocities(1.5, 0, 1.0));
      run.close();
    }

    double after = run.localizer.covariance().orElseThrow().largestPositionStdDev();
    assertThat(after).as("five seconds of driving on the wheels alone").isGreaterThan(before);
    assertThat(distance(run.localizer.pose(), run.robot.truth()))
        .as("and the error stays within what it claims, four times over")
        .isLessThan(4 * after + ODOMETRY_METERS);
  }

  /** L20. */
  @Test
  void l20_thePoseTracksOnlyWhileAMeasurementWasUsedLately() {
    needs(Option.MODE);
    Run run = placed();
    for (int loop = 0; loop < 25; loop++) {
      run.loop(new ChassisVelocities());
      run.measure(Transform2d.ZERO, 0.05, 0);
      run.close();
    }
    assertThat(run.localizer.mode()).contains(LocalizationEstimate.Mode.TRACKING);

    // Half a minute with no measurement at all.
    for (int loop = 0; loop < 1500; loop++) {
      run.loop(new ChassisVelocities());
      run.close();
    }
    assertThat(run.localizer.mode().orElseThrow()).isNotEqualTo(LocalizationEstimate.Mode.TRACKING);
  }

  /** L21. */
  @Test
  void l21_thereIsADecisionForEachMeasurementGiven() {
    needs(Option.DECISIONS);
    Run run = placed();
    run.loop(new ChassisVelocities());
    run.close();
    assertThat(run.localizer.decisions().orElseThrow()).as("given none").isEmpty();

    run.loop(new ChassisVelocities());
    List<VisionMeasurement> taken =
        List.of(
            measurement(run.robot.now(), run.robot.truth(), 0.05, 0.05),
            measurement(run.robot.now() - LOOP_NANOS, run.robot.truth(), 0.05, 0.05),
            measurement(1, run.robot.truth(), 0.05, 0.05));
    run.localizer.measurements(taken);

    assertThat(run.localizer.decisions().orElseThrow())
        .extracting(VisionDecision::measurement)
        .containsExactlyInAnyOrderElementsOf(taken);
  }

  /** L22. */
  @Test
  void l22_theFieldVelocitiesAreTheRobotsOverTheField() {
    needs(Option.FIELD_VELOCITIES);
    Run run = placed();
    for (int loop = 0; loop < 25; loop++) {
      run.loop(new ChassisVelocities());
      run.close();
    }
    ChassisVelocities still = run.localizer.fieldVelocities().orElseThrow();
    assertThat(Math.hypot(still.vx, still.vy)).as("standing still").isLessThan(1e-3);
    assertThat(still.omega).isCloseTo(0, within(1e-3));

    // One meter a second forward, the robot facing 20 degrees: along that heading on the field.
    for (int loop = 0; loop < 50; loop++) {
      run.loop(new ChassisVelocities(1, 0, 0));
      run.close();
    }
    ChassisVelocities moving = run.localizer.fieldVelocities().orElseThrow();
    Rotation2d heading = run.robot.truth().getRotation();
    assertThat(moving.vx).isCloseTo(heading.getCos(), within(0.1));
    assertThat(moving.vy).isCloseTo(heading.getSin(), within(0.1));
  }

  /** L23. */
  @Test
  void l23_whatTurnsOdometryIsTheGyroWhileItReads() {
    needs(Option.YAW_SOURCE);
    Run run = placed();
    run.loop(new ChassisVelocities(1, 0, 0));
    run.close();
    assertThat(run.localizer.yawSource()).contains(YawSource.GYRO);

    run.robot.gyroLost = true;
    run.loop(new ChassisVelocities(1, 0, 0));
    run.close();
    assertThat(run.localizer.yawSource().orElseThrow()).isNotEqualTo(YawSource.GYRO);
  }

  // ---- A scenario's robot and its localizer ----

  /** A scripted robot and the bench's localizer, given the robot's loops in the robot's order. */
  private final class Run {
    final ScriptedDrivetrain robot =
        new ScriptedDrivetrain(ROBOT.moduleTranslations(), START, T0 - LOOP_NANOS, GYRO_AT_START);
    final Localizer localizer = bench.make(ROBOT, FIELD);
    private final List<VisionMeasurement> taken = new ArrayList<>();

    /** One loop of the robot gaining toward a velocity: its readings go to the localizer. */
    DrivetrainReadings loop(ChassisVelocities toward) {
      DrivetrainReadings readings = robot.loop(toward);
      localizer.odometry(readings, new LoopContext(robot.now(), true));
      return readings;
    }

    /** A camera measures the robot this far from where it really is, now. */
    void measure(Transform2d off, double stdDevMeters, double ageNanos) {
      taken.add(
          measurement(
              robot.now() - (long) ageNanos, robot.truth().plus(off), stdDevMeters, stdDevMeters));
    }

    /** The loop's measurements go to the localizer, which closes the loop. */
    void close() {
      localizer.measurements(List.copyOf(taken));
      taken.clear();
    }
  }

  /** A robot that stood for a loop and was then placed where it really is. */
  private Run placed() {
    Run run = new Run();
    run.loop(new ChassisVelocities());
    run.localizer.place(new PoseReset(run.robot.now(), run.robot.truth(), PLACED, "placed"));
    run.close();
    return run;
  }

  /** A measurement from the suite's camera, of two of the field's tags, solved with a heading. */
  private static VisionMeasurement measurement(
      long takenNanos, Pose2d pose, double stdDevMeters, double headingStdDevRadians) {
    return new VisionMeasurement(
        takenNanos,
        pose,
        PoseCovariance.fromStdDevs(stdDevMeters, stdDevMeters, headingStdDevRadians),
        VisionMeasurement.Lever.NONE,
        FRONT.name(),
        List.of(25, 26),
        VisionMeasurement.Solver.MULTI_TAG);
  }

  /** What a localizer reports for an option, whatever its type. */
  private static Optional<?> reported(Localizer localizer, Option option) {
    return switch (option) {
      case COVARIANCE -> localizer.covariance();
      case MODE -> localizer.mode();
      case HEALTH -> localizer.health();
      case DECISIONS -> localizer.decisions();
      case FIELD_VELOCITIES -> localizer.fieldVelocities();
      case WHEELS -> localizer.wheels();
      case TRUST -> localizer.trust();
      case YAW_SOURCE -> localizer.yawSource();
    };
  }

  private boolean has(Option option) {
    return bench.lacks(option).isEmpty();
  }

  /** Skips the rest of a scenario for a localizer that lacks the option it needs, saying why. */
  private void needs(Option option) {
    Optional<String> why = bench.lacks(option);
    Assumptions.assumeTrue(
        why.isEmpty(), () -> bench.name() + " lacks " + option + ": " + why.orElse(""));
  }

  // ---- What is close enough ----

  private static double distance(Pose2d one, Pose2d other) {
    return one.getTranslation().getDistance(other.getTranslation());
  }

  private static void assertNumbers(Pose2d pose) {
    assertThat(pose.getX()).as("x").isFinite();
    assertThat(pose.getY()).as("y").isFinite();
    assertThat(pose.getRotation().getRadians()).as("heading").isFinite();
  }

  /** The same pose, to within rounding. */
  private static void assertSame(Pose2d pose, Pose2d expected) {
    assertThat(distance(pose, expected)).as("%s is %s", pose, expected).isLessThan(SAME);
    assertThat(Math.abs(pose.getRotation().minus(expected.getRotation()).getRadians()))
        .as("%s faces as %s", pose, expected)
        .isLessThan(SAME);
  }

  /** Where exact odometry puts it. */
  private static void assertNear(Pose2d pose, Pose2d expected) {
    assertThat(distance(pose, expected))
        .as("%s near %s", pose, expected)
        .isLessThan(ODOMETRY_METERS);
    assertThat(Math.abs(pose.getRotation().minus(expected.getRotation()).getRadians()))
        .as("%s faces as %s", pose, expected)
        .isLessThan(ODOMETRY_RADIANS);
  }

  private static String bits(Pose2d pose) {
    Translation2d at = pose.getTranslation();
    return Long.toHexString(Double.doubleToRawLongBits(at.getX()))
        + ","
        + Long.toHexString(Double.doubleToRawLongBits(at.getY()))
        + ","
        + Long.toHexString(Double.doubleToRawLongBits(pose.getRotation().getRadians()));
  }
}
