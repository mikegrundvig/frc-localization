package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.localization.EstimatorConfig.GyroErrors;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.HexFormat;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;

/**
 * The estimator's behavior, recorded bit for bit: a scripted session, the same for every way it can
 * correct the heading and treat the gyro's errors, whose every answer is fingerprinted ({@link
 * Fingerprint}) and must hash to the digest in its row. It is the safety net for restructuring the
 * estimator without changing what it does: the replay, the gate, the watch on the cameras, the
 * mounts, recovery and the books all have their say in it.
 *
 * <p>When a change is meant to change the estimator's behavior, the failure prints the new digest:
 * put it in the row, and say in the commit why it moved.
 */
class EstimatorSessionGoldenTest {
  private static final long STEP = 4_000_000L; // 250 Hz
  private static final int STEPS_PER_LOOP = 5;
  private static final long T0 = 1_000_000_000L;
  private static final int LOOPS = 320;
  private static final List<String> CAMERAS = List.of("front", "left", "right");

  /** The gyro reads 2% more turn than the robot makes, and drifts a little every second. */
  private static final double GYRO_SCALE = 1.02;

  private static final double GYRO_DRIFT_RADIANS_PER_SECOND = 0.002;

  /** One way of running the estimator, and the digest of everything its session said. */
  record Row(HeadingCorrection heading, GyroErrors gyro, String digest) {}

  static Stream<Row> rows() {
    return Stream.of(
        new Row(
            HeadingCorrection.EVERY_FIX,
            GyroErrors.ESTIMATED,
            "b970457070dcd5814c104a907c55969568aab514c9dbe73bfa763a392392d3d3"),
        new Row(
            HeadingCorrection.EVERY_FIX,
            GyroErrors.CONSIDERED,
            "da37393ad621cec6efcba08b9ec172183c9e1f34d886f57e9de63e3065fcd9ff"),
        new Row(
            HeadingCorrection.HEADING_MEASUREMENTS,
            GyroErrors.ESTIMATED,
            "5887009cc83ba74c8d76b6cbc20ad14edcfc4c46ad009d7636fbdf0d174bb102"),
        new Row(
            HeadingCorrection.HEADING_MEASUREMENTS,
            GyroErrors.CONSIDERED,
            "147085a6de8c5293f4fb5ab4bbe7cdd7ad700441ebeb37a3f544711a9554bacf"),
        new Row(
            HeadingCorrection.HEADING_MEASUREMENTS_AND_MOUNT_DOUBT,
            GyroErrors.ESTIMATED,
            "39e93aaf2cff7a08a63ff57fd56837d006047f961af95c47817ecc72ac14584d"),
        new Row(
            HeadingCorrection.HEADING_MEASUREMENTS_AND_MOUNT_DOUBT,
            GyroErrors.CONSIDERED,
            "6509233ed296a5a4c14a85451d89dc73e2170d1248494c385d6eebfe9f02bc9b"));
  }

  @ParameterizedTest
  @MethodSource("rows")
  void theSessionSaysWhatItSaidWhenRecorded(Row row) {
    Session session = new Session(estimator(row.heading(), row.gyro()));
    session.play();

    assertThat(digest(session.said.toString()))
        .as(row.heading() + ", " + row.gyro())
        .isEqualTo(row.digest());
  }

  /** The session reaches every part of the estimator the recording is meant to guard. */
  @Test
  void theSessionReachesEveryPartOfTheEstimator() {
    Session session = new Session(estimator(HeadingCorrection.EVERY_FIX, GyroErrors.ESTIMATED));
    session.play();

    assertThat(session.verdicts)
        .contains(
            Verdict.ACCEPTED,
            Verdict.GATED,
            Verdict.HEADING_UNBACKED,
            Verdict.BEFORE_ODOMETRY,
            Verdict.BEFORE_RESET,
            Verdict.STALE,
            Verdict.FROM_THE_FUTURE,
            Verdict.OFF_FIELD);
    assertThat(session.corrections)
        .as("a recovery to the cameras' position, and one to their heading")
        .containsExactlyInAnyOrder(Recovered.Correction.values());
    assertThat(session.modes)
        .contains(
            LocalizationEstimate.Mode.INITIALIZING,
            LocalizationEstimate.Mode.TRACKING,
            LocalizationEstimate.Mode.DEAD_RECKONING,
            LocalizationEstimate.Mode.IMU_FAULT);
    assertThat(session.mostLateSeconds).as("the late camera is found late").isGreaterThan(0.005);
    assertThat(session.mostDoubtMeters).as("the camera off to one side is doubted").isPositive();
  }

  private static PoseEstimator estimator(HeadingCorrection heading, GyroErrors gyro) {
    EstimatorConfig config =
        EstimatorConfigBuilder.builder(EstimatorConfig.defaults()).gyroErrors(gyro).build();
    CameraConfig.MountTolerance tolerance =
        CameraConfig.MountTolerance.of(Meters.of(0.002), Degrees.of(0.1));
    return new PoseEstimator(
        config,
        heading,
        EstimatorSessionGoldenTest::onField,
        tags(32),
        TestCameras.mounted(tolerance, CAMERAS.toArray(String[]::new)),
        new PoseReset(
            0,
            new Pose2d(8, 4, Rotation2d.ZERO),
            PoseCovariance.fromStdDevs(16, 8, Math.PI),
            "not placed yet"));
  }

  private static boolean onField(Translation2d point) {
    return point.getX() >= 0 && point.getX() <= 16 && point.getY() >= 0 && point.getY() <= 8;
  }

  private static Set<Integer> tags(int count) {
    Set<Integer> tags = new HashSet<>();
    for (int tag = 1; tag <= count; tag++) {
      tags.add(tag);
    }
    return tags;
  }

  private static String digest(String said) {
    try {
      MessageDigest sha = MessageDigest.getInstance("SHA-256");
      return HexFormat.of().formatHex(sha.digest(said.getBytes(StandardCharsets.UTF_8)));
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException(e);
    }
  }

  /** A camera's image waiting to arrive: the measurement, and the loop it arrives at. */
  private record InFlight(int arrives, VisionMeasurement measurement) {}

  /**
   * The scripted session: a robot driving arcs on a gyro that misreads its turns and drifts, three
   * cameras (one sound, one whose measurements land to one side, one that stamps its images late),
   * and the events the estimator is there for: a placement, a misread tag, an unfelt shove, a
   * collision, a driver's wrong heading reset, a knocked camera, a mount's doubt, a lost gyro, and
   * measurements it can't use.
   */
  private static final class Session {
    final PoseEstimator estimator;
    final StringBuilder said = new StringBuilder();
    final Set<Verdict> verdicts = EnumSet.noneOf(Verdict.class);
    final Set<Recovered.Correction> corrections = EnumSet.noneOf(Recovered.Correction.class);
    final Set<LocalizationEstimate.Mode> modes = EnumSet.noneOf(LocalizationEstimate.Mode.class);
    double mostLateSeconds;
    double mostDoubtMeters;

    private final Random noise = new Random(20261109L);
    private final List<InFlight> inFlight = new ArrayList<>();
    private final List<Long> times = new ArrayList<>();
    private final List<Pose2d> truths = new ArrayList<>();
    private long time = T0;
    private Pose2d truth = new Pose2d(3, 3, Rotation2d.fromDegrees(10));
    private double gyroYaw = 0.4;
    private double knockRadians;

    Session(PoseEstimator estimator) {
      this.estimator = estimator;
    }

    void play() {
      // Before odometry starts, a camera's measurement can't be placed in time.
      estimator.addVision(position("front", time, truth, 0.02));
      record(estimator.update());
      odometry(1, 0, 0, 0, YawSource.GYRO);
      for (int loop = 1; loop <= LOOPS; loop++) {
        loop(loop);
      }
    }

    private void loop(int loop) {
      double vx = 1.2 * Math.sin(loop / 25.0);
      double vy = 0.4 * Math.cos(loop / 40.0);
      double omega = 0.9 * Math.sin(loop / 17.0);
      if (loop >= 255 && loop < 315) {
        // Standing still, where the gyro's drift shows by itself.
        vx = 0;
        vy = 0;
        omega = 0;
      }
      YawSource source = YawSource.GYRO;
      if (loop >= 230 && loop < 240) {
        source = YawSource.WHEELS; // both IMUs lost
      }
      if (loop >= 240 && loop < 250) {
        source = YawSource.BACKUP;
      }
      if (loop == 100) {
        estimator.setConditions(new DriveConditions(0, 0, 1, 0, false)); // a collision, felt
      }
      if (loop == 104) {
        estimator.setConditions(DriveConditions.NORMAL);
      }
      if (loop == 150) {
        estimator.setConditions(new DriveConditions(Math.toRadians(8), 0, 0, 0.3, false));
      }
      if (loop == 160) {
        estimator.setConditions(DriveConditions.NORMAL);
      }
      odometry(STEPS_PER_LOOP, vx, vy, omega, source);
      if (loop == 70) {
        // Pushed, unfelt: the wheels never saw it.
        truth = truth.transformBy(new Transform2d(0.6, 0.2, Rotation2d.ZERO));
      }
      if (loop == 20) {
        estimator.reset(
            new PoseReset(time, truth, PoseCovariance.fromStdDevs(0.05, 0.05, 0.03), "placed"));
        // Stamped a little after the newest step, with a history of that step alone.
        estimator.addVision(position("front", time + 8_000_000L, truth, 0.03));
      }
      if (loop == 130) {
        // A driver's heading reset, 25 degrees wrong.
        estimator.resetHeading(
            truth.getRotation().plus(Rotation2d.fromDegrees(25)), Math.toRadians(2));
      }
      if (loop == 190) {
        knockRadians = Math.toRadians(5); // the left camera knocked on its mount
      }
      if (loop == 200) {
        estimator.mountDoubt("left", 0.06, 3.0);
      }
      see(loop);
      deliver(loop);
      if (loop == 60) {
        // A misread tag: far off, and sure of itself.
        estimator.addVision(
            position("front", time - 30_000_000L, offset(truthAt(time - 30_000_000L), 1.5), 0.02));
      }
      if (loop == 120) {
        estimator.addVision(position("front", time + 200_000_000L, truth, 0.02)); // the future
        estimator.addVision(
            position("front", time - 20_000_000L, new Pose2d(-3, 2, Rotation2d.ZERO), 0.02));
      }
      if (loop == 300) {
        estimator.addVision(position("front", time - 1_700_000_000L, truth, 0.02)); // stale
      }
      if (loop == 21) {
        estimator.addVision(position("left", time - 100_000_000L, truth, 0.02)); // before reset
      }
      record(estimator.update());
      record();
    }

    /** Some odometry steps: the wheels read the motion, the gyro its own idea of the turn. */
    private void odometry(int steps, double vx, double vy, double omega, YawSource source) {
      double dt = STEP / 1e9;
      for (int i = 0; i < steps; i++) {
        time += STEP;
        truth = truth.plus(new Twist2d(vx * dt, vy * dt, omega * dt).exp());
        times.add(time);
        truths.add(truth);
        gyroYaw += GYRO_SCALE * omega * dt + GYRO_DRIFT_RADIANS_PER_SECOND * dt;
        double reading = gyroYaw;
        if (times.size() == 400) {
          reading = Double.NaN; // one reading that isn't a number
        }
        double slip = 1 + 0.02 * noise.nextGaussian();
        estimator.addOdometry(
            time,
            new BodyMotion(slip * vx * dt, slip * vy * dt, omega * dt, 0.001),
            reading,
            source);
      }
    }

    /** What each camera takes this loop, sent on with its latency. */
    private void see(int loop) {
      long taken = time - 2 * STEP;
      Pose2d then = truthAt(taken);
      boolean dark = loop >= 255 && loop < 315;
      if (dark) {
        return; // no tags in sight for over a second
      }
      boolean headingsOnly = loop >= 130 && loop < 150;
      if (headingsOnly) {
        // After the driver's wrong reset, the cameras see only tags that give their own heading.
        inFlight.add(new InFlight(loop + 1, withHeading("front", taken, then, 0)));
        inFlight.add(new InFlight(loop + 2, withHeading("left", taken, then, 0)));
        inFlight.add(new InFlight(loop + 3, singleTag("right", taken + 40_000_000L, then)));
        return;
      }
      // The front camera: every loop, a position solved with the gyro's heading, and every third a
      // multi-tag solve with its own heading.
      inFlight.add(new InFlight(loop + 1, leaning("front", taken, then, 0)));
      if (loop % 3 == 0) {
        inFlight.add(new InFlight(loop + 1, withHeading("front", taken, then, 0)));
      }
      // The left camera, every second loop: its positions land 8 cm to one side, and its headings
      // turn with its mount once it is knocked.
      if (loop % 2 == 0) {
        Pose2d aside = then.transformBy(new Transform2d(0, 0.08, Rotation2d.ZERO));
        inFlight.add(new InFlight(loop + 2, leaning("left", taken, aside, 0)));
        inFlight.add(new InFlight(loop + 2, withHeading("left", taken, then, knockRadians)));
      }
      // The right camera stamps its images 40 ms after they were taken, and they come in pairs,
      // out of order with the others'.
      if (loop % 3 == 1) {
        inFlight.add(new InFlight(loop + 3, leaning("right", taken + 40_000_000L, then, 0)));
        Pose2d earlier = truthAt(taken - 4 * STEP);
        inFlight.add(
            new InFlight(loop + 3, leaning("right", taken - 4 * STEP + 40_000_000L, earlier, 0)));
        // And a single tag's heading, as unsure as one is.
        inFlight.add(new InFlight(loop + 3, singleTag("right", taken + 40_000_000L, then)));
      }
    }

    private void deliver(int loop) {
      List<InFlight> arrived = new ArrayList<>();
      for (InFlight each : inFlight) {
        if (each.arrives() <= loop) {
          arrived.add(each);
        }
      }
      inFlight.removeAll(arrived);
      // The latest first: the estimator must give the same result in any order of arrival.
      for (int i = arrived.size() - 1; i >= 0; i--) {
        estimator.addVision(arrived.get(i).measurement());
      }
    }

    /**
     * A position solved with the gyro's heading, or, while the estimator has its heading in doubt,
     * the same position solved from its tags alone, as the default localizer has it solved.
     */
    private VisionMeasurement leaning(String camera, long taken, Pose2d pose, double turn) {
      if (estimator.headingInDoubt()) {
        return position(
            camera,
            taken,
            new Pose2d(jitter(pose.getTranslation(), 0.05), pose.getRotation()),
            0.08);
      }
      Pose2d seen = new Pose2d(jitter(pose.getTranslation(), 0.01), pose.getRotation());
      return new VisionMeasurement(
          taken,
          seen,
          PoseCovariance.fromStdDevs(0.03, 0.03, Double.POSITIVE_INFINITY),
          new VisionMeasurement.Lever(
              estimator.pose().getRotation().plus(new Rotation2d(turn)),
              new Translation2d(3, 0.4),
              new Translation2d(0.3, 0)),
          camera,
          List.of(25, 26),
          VisionMeasurement.Solver.HEADING_CONSTRAINED);
    }

    private VisionMeasurement withHeading(String camera, long taken, Pose2d pose, double turn) {
      Rotation2d heading =
          pose.getRotation()
              .plus(new Rotation2d(turn + Math.toRadians(0.5) * noise.nextGaussian()));
      return new VisionMeasurement(
          taken,
          new Pose2d(jitter(pose.getTranslation(), 0.04), heading),
          PoseCovariance.fromStdDevs(0.06, 0.06, Math.toRadians(1.5)),
          VisionMeasurement.Lever.NONE,
          camera,
          List.of(9, 10),
          VisionMeasurement.Solver.MULTI_TAG);
    }

    private VisionMeasurement singleTag(String camera, long taken, Pose2d pose) {
      Rotation2d heading =
          pose.getRotation().plus(new Rotation2d(Math.toRadians(3) * noise.nextGaussian()));
      return new VisionMeasurement(
          taken,
          new Pose2d(jitter(pose.getTranslation(), 0.05), heading),
          PoseCovariance.fromStdDevs(0.08, 0.08, Math.toRadians(8)),
          VisionMeasurement.Lever.NONE,
          camera,
          List.of(26),
          VisionMeasurement.Solver.SINGLE_TAG);
    }

    private VisionMeasurement position(String camera, long taken, Pose2d pose, double stdDev) {
      return new VisionMeasurement(
          taken,
          pose,
          PoseCovariance.fromStdDevs(stdDev, stdDev, Double.POSITIVE_INFINITY),
          VisionMeasurement.Lever.NONE,
          camera,
          List.of(26),
          VisionMeasurement.Solver.HEADING_CONSTRAINED);
    }

    private Translation2d jitter(Translation2d point, double stdDev) {
      return point.plus(
          new Translation2d(stdDev * noise.nextGaussian(), stdDev * noise.nextGaussian()));
    }

    private static Pose2d offset(Pose2d pose, double meters) {
      return pose.transformBy(new Transform2d(meters, 0, Rotation2d.ZERO));
    }

    /** Where the robot really was at a step's time: the step at or before it. */
    private Pose2d truthAt(long when) {
      for (int i = times.size() - 1; i >= 0; i--) {
        if (times.get(i) <= when) {
          return truths.get(i);
        }
      }
      return truths.get(0);
    }

    private void record(List<MeasurementVerdict> update) {
      for (MeasurementVerdict verdict : update) {
        verdicts.add(verdict.verdict());
      }
      said.append(Fingerprint.of(update)).append('\n');
    }

    /** Everything the estimator says once the loop's update is done. */
    private void record() {
      LocalizationEstimate estimate = estimator.estimate();
      modes.add(estimate.mode());
      estimator.lastRecovery().ifPresent(recovered -> corrections.add(recovered.correction()));
      List<Object> answers = new ArrayList<>();
      answers.add(estimate);
      answers.add(estimator.pose());
      answers.add(estimator.poseAt(time - 50_000_000L));
      answers.add(estimator.headingDoubt());
      answers.add(estimator.headingResets());
      answers.add(estimator.lastRecovery());
      answers.add(estimator.gyroScalePercent());
      answers.add(estimator.gyroScaleStdDevPercent());
      answers.add(estimator.gyroDriftDegreesPerMinute());
      answers.add(estimator.gyroDriftStdDevDegreesPerMinute());
      answers.add(estimator.odometryOnly().xMeters());
      answers.add(estimator.odometryOnly().yMeters());
      answers.add(estimator.odometryOnly().headingRadians());
      answers.add(estimator.cameras());
      for (String camera : CAMERAS) {
        answers.add(estimator.cameraDoubtMeters(camera));
        answers.add(estimator.cameraLateSeconds(camera));
        answers.add(estimator.cameraTurnDegrees(camera));
        answers.add(estimator.cameraTurnStdDevDegrees(camera));
        mostLateSeconds = Math.max(mostLateSeconds, estimator.cameraLateSeconds(camera));
        mostDoubtMeters = Math.max(mostDoubtMeters, estimator.cameraDoubtMeters(camera));
      }
      said.append(Fingerprint.of(answers)).append('\n');
    }
  }
}
