package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * WPILib's estimator as a localizer, on the conformance suite's robot: what it does about each
 * thing WPILib leaves to the code that feeds it (its class comment lists them), and the counts it
 * keeps of what WPILib did with the measurements. The suite checks the contract; this checks the
 * choices.
 */
class WpilibLocalizerTest {
  private static final Pose2d START = new Pose2d(4, 4, Rotation2d.fromDegrees(20));
  private static final long T0 = 1_000_000_000L;
  private static final long LOOP_NANOS = ScriptedDrivetrain.LOOP_NANOS;

  /** A scripted robot placed where it stands, and WPILib's localizer fed its loops. */
  private static final class Drive {
    final ScriptedDrivetrain robot =
        new ScriptedDrivetrain(
            LocalizerConformance.ROBOT.moduleTranslations(), START, T0 - LOOP_NANOS, 0.3);
    final WpilibLocalizer localizer;

    Drive(LocalizerMaker maker) {
      localizer =
          (WpilibLocalizer) maker.make(LocalizerConformance.ROBOT, LocalizerConformance.FIELD);
      loop(new ChassisVelocities());
      localizer.place(
          new PoseReset(
              robot.now(), robot.truth(), PoseCovariance.fromStdDevs(0.05, 0.05, 0.03), "placed"));
    }

    Drive() {
      this(WpilibLocalizer::new);
    }

    /** One loop of the robot's readings, as the robot reads them. */
    DrivetrainReadings loop(ChassisVelocities toward) {
      DrivetrainReadings readings = robot.loop(toward);
      localizer.odometry(readings, new LoopContext(robot.now(), true));
      return readings;
    }

    /** Loops of the robot driving. */
    Drive drive(int loops, ChassisVelocities toward) {
      for (int k = 0; k < loops; k++) {
        loop(toward);
        localizer.measurements(List.of());
      }
      return this;
    }

    /** A measurement of the robot this far from where it is now, taken this long ago. */
    VisionMeasurement seen(Transform2d off, long ageNanos) {
      return measurement(robot.now() - ageNanos, robot.truth().plus(off));
    }

    double headingErrorRadians() {
      return Math.abs(
          localizer.pose().getRotation().minus(robot.truth().getRotation()).getRadians());
    }

    double positionErrorMeters() {
      return localizer.pose().getTranslation().getDistance(robot.truth().getTranslation());
    }
  }

  private static VisionMeasurement measurement(long takenNanos, Pose2d pose) {
    return new VisionMeasurement(
        takenNanos,
        pose,
        PoseCovariance.fromStdDevs(0.05, 0.05, 0.05),
        VisionMeasurement.Lever.NONE,
        "front",
        List.of(25, 26),
        VisionMeasurement.Solver.MULTI_TAG);
  }

  /** What WPILib did with the measurements, as the localizer counts it. */
  private record Counts(int offered, int corrected, int discardedLater, int tooOld) {
    static Counts of(WpilibLocalizer localizer) {
      return new Counts(
          localizer.offered(),
          localizer.corrected(),
          localizer.discardedLater(),
          localizer.tooOld());
    }
  }

  /**
   * One way measurements arrive, and what the counts say of it.
   *
   * @param name what the row is
   * @param arrive what the cameras send, loop by loop, to a robot driving a meter a second
   * @param counts what the localizer counts
   */
  private record CountRow(String name, Consumer<Drive> arrive, Counts counts) {}

  @Test
  void itCountsWhatWpilibDidWithEachMeasurement() {
    Transform2d off = new Transform2d(0.1, 0, Rotation2d.ZERO);
    ChassisVelocities ahead = new ChassisVelocities(1, 0, 0);
    List<CountRow> rows =
        List.of(
            new CountRow(
                "two in a loop, newest first: sorted, both correct",
                drive -> {
                  drive.loop(ahead);
                  drive.localizer.measurements(
                      List.of(drive.seen(off, 0), drive.seen(off, 2 * LOOP_NANOS)));
                },
                new Counts(2, 2, 0, 0)),
            new CountRow(
                "an older one a loop after a newer one: the newer correction is discarded",
                drive -> {
                  drive.loop(ahead);
                  drive.localizer.measurements(List.of(drive.seen(off, 0)));
                  drive.loop(ahead);
                  drive.localizer.measurements(List.of(drive.seen(off, 3 * LOOP_NANOS)));
                },
                new Counts(2, 2, 1, 0)),
            new CountRow(
                "two seconds old: older than WPILib keeps",
                drive -> {
                  drive.loop(ahead);
                  drive.localizer.measurements(List.of(drive.seen(off, 2_000_000_000L)));
                },
                new Counts(1, 0, 0, 1)),
            new CountRow(
                "a pose that isn't a number: never given to WPILib",
                drive -> {
                  drive.loop(ahead);
                  drive.localizer.measurements(
                      List.of(
                          measurement(
                              drive.robot.now(), new Pose2d(Double.NaN, 4, Rotation2d.ZERO)),
                          measurement(
                              drive.robot.now(),
                              new Pose2d(
                                  drive.robot.truth().getTranslation(),
                                  new Rotation2d(Double.NaN)))));
                },
                new Counts(2, 0, 0, 0)));
    for (CountRow row : rows) {
      Drive drive = new Drive().drive(50, ahead);

      row.arrive().accept(drive);

      assertThat(drive.positionErrorMeters()).as("%s: meters off", row.name()).isLessThan(0.2);
      assertThat(Counts.of(drive.localizer)).as(row.name()).isEqualTo(row.counts());
    }
  }

  /** Before any wheel reading there is nothing to correct, and nothing to look back on. */
  @Test
  void beforeOdometryItCorrectsNothingAndKeepsNothing() {
    WpilibLocalizer fresh =
        new WpilibLocalizer(LocalizerConformance.ROBOT, LocalizerConformance.FIELD);
    Pose2d before = fresh.pose();

    fresh.measurements(List.of(measurement(T0, START)));

    assertThat(Counts.of(fresh)).isEqualTo(new Counts(1, 0, 0, 0));
    assertThat(fresh.pose()).isEqualTo(before);
    assertThat(fresh.poseAt(T0 - LOOP_NANOS)).isEmpty();
  }

  /**
   * One way the yaw and the wheels reach it, each spoiling a loop's readings as a robot's can, and
   * how far its pose may then be from where the robot is.
   *
   * @param name what the row is
   * @param spoil what the robot's readings go through, loop by loop, while it drives and turns
   * @param headingRadians how far its heading may be off at the end
   * @param positionMeters and its position
   */
  private record ReadingRow(
      String name, Spoil spoil, double headingRadians, double positionMeters) {}

  /** A change to one loop's readings, given which loop it is. */
  @FunctionalInterface
  private interface Spoil {
    DrivetrainReadings apply(ScriptedDrivetrain robot, int loop, DrivetrainReadings readings);
  }

  /**
   * The gyro, then the backup IMU, then the wheels turn it, with no jump as each takes over; a
   * wheel that doesn't report stays where it was; yaw readings stamped alike, and a wheel reading
   * after the last yaw reading, are read as the nearest; a wheel reading taken twice or out of
   * order is ignored.
   */
  @Test
  void eachWayTheReadingsArriveKeepsTheRobotWhereItIs() {
    List<ReadingRow> rows =
        List.of(
            new ReadingRow(
                "every reading as read", (robot, loop, readings) -> readings, 1e-6, 1e-3),
            new ReadingRow(
                "the gyro lost, then the backup IMU",
                (robot, loop, readings) -> {
                  robot.gyroLost = loop >= 20;
                  robot.backupLost = loop >= 40;
                  return readings;
                },
                0.02,
                0.02),
            new ReadingRow(
                "a wheel not reporting for ten loops",
                (robot, loop, readings) ->
                    loop >= 20 && loop < 30 ? withoutWheel(readings, 0) : readings,
                0.01,
                0.05),
            new ReadingRow(
                "each yaw reading stamped as the one before",
                (robot, loop, readings) -> withYawTimesPaired(readings),
                0.01,
                0.02),
            new ReadingRow(
                "the newest wheel reading after the newest yaw reading",
                (robot, loop, readings) -> withoutLastYaw(readings),
                0.01,
                0.02),
            new ReadingRow(
                "every loop's readings taken in twice",
                (robot, loop, readings) -> withEachWheelReadingTwice(readings),
                1e-6,
                1e-3),
            new ReadingRow(
                "a reading from an earlier loop among each loop's",
                (robot, loop, readings) -> withAnOlderReading(readings),
                1e-6,
                1e-3));
    for (ReadingRow row : rows) {
      Drive drive = new Drive();
      long newest = drive.localizer.timestampNanos();
      for (int loop = 0; loop < 60; loop++) {
        DrivetrainReadings readings =
            row.spoil().apply(drive.robot, loop, drive.robot.loop(new ChassisVelocities(1, 0, 1)));
        drive.localizer.odometry(readings, new LoopContext(drive.robot.now(), true));
        drive.localizer.measurements(List.of());

        assertThat(drive.localizer.timestampNanos())
            .as("%s, loop %d: the time never runs backwards", row.name(), loop)
            .isGreaterThanOrEqualTo(newest);
        newest = drive.localizer.timestampNanos();
      }

      assertThat(drive.headingErrorRadians()).as(row.name()).isLessThan(row.headingRadians());
      assertThat(drive.positionErrorMeters()).as(row.name()).isLessThan(row.positionMeters());
    }
  }

  private static DrivetrainReadings withoutWheel(DrivetrainReadings readings, int wheel) {
    List<OdometrySample> samples = new ArrayList<>();
    for (OdometrySample sample : readings.wheels()) {
      double[] distances = new double[sample.wheels()];
      double[] angles = new double[sample.wheels()];
      for (int i = 0; i < sample.wheels(); i++) {
        distances[i] = i == wheel ? Double.NaN : sample.distanceMeters(i);
        angles[i] = i == wheel ? Double.NaN : sample.angleRadians(i);
      }
      samples.add(new OdometrySample(sample.timestampNanos(), distances, angles));
    }
    return DrivetrainReadingsBuilder.builder(readings).wheels(samples).build();
  }

  /** The gyro's yaw readings, each odd one stamped with the time of the one before it. */
  private static DrivetrainReadings withYawTimesPaired(DrivetrainReadings readings) {
    YawSamples yaws = readings.gyro().yaws();
    long[] times = new long[yaws.count()];
    double[] values = new double[yaws.count()];
    for (int i = 0; i < yaws.count(); i++) {
      times[i] = yaws.timeNanos(i - i % 2);
      values[i] = yaws.yawRadians(i);
    }
    return withGyroYaws(readings, YawSamples.of(times, values));
  }

  /** The gyro's yaw readings without the newest, which the newest wheel reading comes after. */
  private static DrivetrainReadings withoutLastYaw(DrivetrainReadings readings) {
    YawSamples yaws = readings.gyro().yaws();
    int kept = yaws.count() - 1;
    long[] times = new long[kept];
    double[] values = new double[kept];
    for (int i = 0; i < kept; i++) {
      times[i] = yaws.timeNanos(i);
      values[i] = yaws.yawRadians(i);
    }
    return withGyroYaws(readings, YawSamples.of(times, values));
  }

  private static DrivetrainReadings withGyroYaws(DrivetrainReadings readings, YawSamples yaws) {
    return DrivetrainReadingsBuilder.builder(readings)
        .gyro(ImuReadingsBuilder.builder(readings.gyro()).yaws(yaws).build())
        .build();
  }

  /** Each wheel reading, then the same again: a reading taken twice. */
  private static DrivetrainReadings withEachWheelReadingTwice(DrivetrainReadings readings) {
    List<OdometrySample> twice = new ArrayList<>();
    for (OdometrySample sample : readings.wheels()) {
      twice.add(sample);
      twice.add(sample);
    }
    return DrivetrainReadingsBuilder.builder(readings).wheels(twice).build();
  }

  /**
   * The loop's wheel readings with, after them, one from a loop ago at its old place: a reading out
   * of order, which would run the robot back a loop.
   */
  private static DrivetrainReadings withAnOlderReading(DrivetrainReadings readings) {
    List<OdometrySample> samples = new ArrayList<>(readings.wheels());
    OdometrySample first = samples.get(0);
    double[] distances = new double[first.wheels()];
    double[] angles = new double[first.wheels()];
    for (int i = 0; i < first.wheels(); i++) {
      distances[i] = first.distanceMeters(i) - 0.02;
      angles[i] = first.angleRadians(i);
    }
    samples.add(new OdometrySample(first.timestampNanos() - LOOP_NANOS, distances, angles));
    return DrivetrainReadingsBuilder.builder(readings).wheels(samples).build();
  }

  /** A placement that says nothing of the heading keeps the heading WPILib had. */
  @Test
  void aPlacementWithNoHeadingKeepsTheHeading() {
    Drive drive = new Drive().drive(25, new ChassisVelocities(1, 0, 1));
    Rotation2d heading = drive.localizer.pose().getRotation();
    Pose2d elsewhere = new Pose2d(7, 2, Rotation2d.fromDegrees(-100));

    drive.localizer.place(
        new PoseReset(
            drive.robot.now(),
            elsewhere,
            PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY),
            "position only"));

    assertThat(drive.localizer.pose().getTranslation()).isEqualTo(elsewhere.getTranslation());
    assertThat(drive.localizer.pose().getRotation().minus(heading).getRadians())
        .isCloseTo(0, within(1e-9));
  }

  /**
   * The state's standard deviations set the gain: odometry trusted ten times more, one measurement
   * ten centimeters off moves the estimate far less than with WPILib's defaults.
   */
  @Test
  void trustingOdometryMoreMovesTheEstimateLessForAMeasurement() {
    Transform2d off = new Transform2d(0.1, 0, Rotation2d.ZERO);
    double[] moved = new double[2];
    LocalizerMaker[] makers = {WpilibLocalizer::new, WpilibLocalizer.with(0.01, 0.01)};
    for (int k = 0; k < makers.length; k++) {
      Drive drive = new Drive(makers[k]).drive(10, new ChassisVelocities());
      drive.loop(new ChassisVelocities());
      drive.localizer.measurements(List.of(drive.seen(off, 0)));
      moved[k] = drive.positionErrorMeters();
    }

    // WPILib's defaults: 0.1 m of state against 0.05 m measured.
    assertThat(moved[0]).as("WPILib's defaults").isBetween(0.03, 0.1);
    assertThat(moved[1]).as("odometry trusted ten times more").isLessThan(moved[0] / 3);
  }
}
