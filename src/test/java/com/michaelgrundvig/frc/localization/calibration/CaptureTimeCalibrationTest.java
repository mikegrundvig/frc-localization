package com.michaelgrundvig.frc.localization.calibration;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import java.util.Optional;
import java.util.Random;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The spin test's arithmetic, on a robot spinning in place 2.5 m from a tag: each image's view of
 * the tag made from where the robot truly was when the image was taken, its stamp late by what the
 * case says, the view's heading off by noise and by a bias that changes with where the robot
 * points, as a real solve's does.
 */
class CaptureTimeCalibrationTest {
  /** A tag 2.5 m ahead of the field's origin, facing back along x. */
  private static final Pose3d TAG = new Pose3d(2.5, 0, 0.5, new Rotation3d(0, 0, Math.PI));

  /** A camera 0.3 m ahead of the robot's center and 0.1 m left, looking straight ahead. */
  private static final Transform3d MOUNT =
      new Transform3d(new Translation3d(0.3, 0.1, 0.4), Rotation3d.ZERO);

  /** How a spin went: the stamps' lateness, the rates each way, how noisy and wild the views. */
  private record Spin(
      String what,
      double lateSeconds,
      double leftRate,
      double rightRate,
      double noiseDegrees,
      double wildShare) {}

  /**
   * Sightings from spins through ±50° of facing the tag, each way at its rate, 30 images a second.
   */
  private static CaptureTimeCalibration spun(Spin spin, long seed) {
    Random random = new Random(seed);
    CaptureTimeCalibration calibration = new CaptureTimeCalibration();
    for (double rate : new double[] {spin.leftRate(), -spin.rightRate()}) {
      if (rate == 0) {
        continue;
      }
      double reach = Math.toRadians(50);
      int images = (int) (30 * 2 * reach / Math.abs(rate));
      for (int i = 0; i < images; i++) {
        // Where the robot truly pointed when the image was taken, sweeping the way it turns.
        double fraction = (double) i / images;
        double taken = rate > 0 ? -reach + 2 * reach * fraction : reach - 2 * reach * fraction;
        // Its heading at the stamp, which is later: it had turned further by then.
        Rotation2d atStamp = new Rotation2d(taken + rate * spin.lateSeconds());
        Pose3d robot = new Pose3d(0, 0, 0, new Rotation3d(0, 0, taken));
        Transform3d view = new Transform3d(robot.transformBy(MOUNT), TAG);
        // The solve's error: a bias that changes with the view, noise, and now and then a wild one.
        double error =
            Math.toRadians(0.8) * Math.sin(3 * taken)
                + Math.toRadians(spin.noiseDegrees()) * random.nextGaussian();
        if (random.nextDouble() < spin.wildShare()) {
          error += Math.toRadians(random.nextBoolean() ? 6 : 40);
        }
        Transform3d seen = turned(view, error);
        // The other view: the tag turned over, far from the truth.
        Transform3d other = turned(view, Math.toRadians(70));
        calibration.add(TAG, MOUNT, seen, other, atStamp, rate);
      }
    }
    return calibration;
  }

  /**
   * A view of the tag that puts the robot's heading off by an angle: the camera turned about the
   * tag's place, as a heading error in the solve does.
   */
  private static Transform3d turned(Transform3d view, double headingErrorRadians) {
    Rotation3d turn = new Rotation3d(0, 0, -headingErrorRadians);
    return new Transform3d(view.getTranslation().rotateBy(turn), view.getRotation().rotateBy(turn));
  }

  @Test
  void howLateTheStampsAreIsFoundFromSpinningBothWays() {
    List<Spin> spins =
        List.of(
            new Spin("a loop late", 0.020, 1.5, 1.5, 0.3, 0),
            new Spin("on time", 0, 1.5, 1.5, 0.3, 0),
            new Spin("early", -0.015, 1.5, 1.5, 0.3, 0),
            new Spin("faster one way", 0.025, 1.0, 2.0, 0.3, 0),
            new Spin("noisy", 0.020, 1.5, 1.5, 1.0, 0),
            new Spin("misread tags among them", 0.020, 1.5, 1.5, 0.3, 0.08));
    for (Spin spin : spins) {
      for (long seed = 1; seed <= 5; seed++) {
        CaptureTimeCalibration.Result result = spun(spin, seed).solve().orElseThrow();
        assertThat(result.lateSeconds())
            .as("%s, seed %d", spin.what(), seed)
            .isCloseTo(spin.lateSeconds(), within(0.002));
        // Its own uncertainty is honest: the truth within four of its standard deviations.
        assertThat(Math.abs(result.lateSeconds() - spin.lateSeconds()))
            .as(
                "%s, seed %d, against its %.2f ms",
                spin.what(), seed, result.stdDevSeconds() * 1000)
            .isLessThan(4 * result.stdDevSeconds() + 0.0002);
      }
    }
  }

  /**
   * Images 2.9° apart (30 a second at 1.5 rad/s), the right spin's falling half a frame from the
   * left's, against a solve error that changes steeply with where the robot points: each direction
   * then meets the error at its own headings, and a fit that took the error as flat within a group
   * would read the difference as time.
   */
  @Test
  void aSteepBiasMetAtOtherHeadingsEachWayDropsOut() {
    for (double late : new double[] {0, 0.02}) {
      CaptureTimeCalibration calibration = new CaptureTimeCalibration();
      double rate = 1.5;
      double step = rate / 30;
      double reach = Math.toRadians(50);
      for (double way : new double[] {1, -1}) {
        double phase = way > 0 ? 0 : step / 2;
        for (double taken = -reach + phase; taken < reach; taken += step) {
          Rotation2d atStamp = new Rotation2d(taken + way * rate * late);
          Pose3d robot = new Pose3d(0, 0, 0, new Rotation3d(0, 0, taken));
          Transform3d view = new Transform3d(robot.transformBy(MOUNT), TAG);
          // 1.5° of error swinging over a 30° stretch of heading: steep within any one group.
          double error = Math.toRadians(1.5) * Math.sin(12 * taken);
          calibration.add(
              TAG,
              MOUNT,
              turned(view, error),
              turned(view, Math.toRadians(70)),
              atStamp,
              way * rate);
        }
      }
      CaptureTimeCalibration.Result result = calibration.solve().orElseThrow();
      assertThat(result.lateSeconds())
          .as("%.0f ms late", late * 1000)
          .isCloseTo(late, within(0.0005));
    }
  }

  /**
   * A real spin's rate isn't even: it wobbles with where the robot points (a swerve's modules, a
   * carpet's nap). With a solve error that also changes with heading, a fit that took each image's
   * own rate would read the two lining up as time: only the difference between the directions says
   * time.
   */
  @Test
  void aRateThatWobblesWithHeadingIsNotTakenForTime() {
    for (double late : new double[] {0, 0.02}) {
      CaptureTimeCalibration calibration = new CaptureTimeCalibration();
      double reach = Math.toRadians(50);
      for (double way : new double[] {1, -1}) {
        for (int i = 0; i < 2000; i++) {
          double taken = -reach + 2 * reach * i / 2000.0;
          // 1.5 rad/s, 20% faster and slower as the heading goes round, the error rising with it.
          double rate = way * 1.5 * (1 + 0.2 * Math.sin(12 * taken));
          Rotation2d atStamp = new Rotation2d(taken + rate * late);
          Pose3d robot = new Pose3d(0, 0, 0, new Rotation3d(0, 0, taken));
          Transform3d view = new Transform3d(robot.transformBy(MOUNT), TAG);
          double error = Math.toRadians(1.5) * Math.sin(12 * taken);
          calibration.add(
              TAG, MOUNT, turned(view, error), turned(view, Math.toRadians(70)), atStamp, rate);
        }
      }
      CaptureTimeCalibration.Result result = calibration.solve().orElseThrow();
      assertThat(result.lateSeconds())
          .as("%.0f ms late", late * 1000)
          .isCloseTo(late, within(0.0005));
    }
  }

  @Test
  void aBiasThatDependsOnWhereTheRobotPointsDropsOut() {
    // Without the grouping by heading, the bias's sine, met at different times each way, would
    // read as time: spinning both ways over the same headings, it doesn't.
    CaptureTimeCalibration.Result result =
        spun(new Spin("biased", 0.010, 1.5, 1.5, 0, 0), 1).solve().orElseThrow();
    assertThat(result.lateSeconds()).isCloseTo(0.010, within(0.0005));
  }

  @Test
  void aSpinOneWayOrTooShortSaysNothing() {
    record Case(String what, Spin spin) {}
    for (Case c :
        List.of(
            new Case("left only", new Spin("left only", 0.02, 1.5, 0, 0.3, 0)),
            new Case("right only", new Spin("right only", 0.02, 0, 1.5, 0.3, 0)))) {
      Optional<CaptureTimeCalibration.Result> result = spun(c.spin(), 1).solve();
      assertThat(result).as(c.what()).isEmpty();
    }
    CaptureTimeCalibration few = new CaptureTimeCalibration();
    Transform3d view = new Transform3d(new Pose3d().transformBy(MOUNT), TAG);
    for (int i = 0; i < CaptureTimeCalibration.MIN_EACH_WAY - 1; i++) {
      few.add(TAG, MOUNT, view, view, Rotation2d.ZERO, 1.5);
      few.add(TAG, MOUNT, view, view, Rotation2d.ZERO, -1.5);
    }
    assertThat(few.solve()).as("one short each way").isEmpty();
    assertThat(few.left()).isEqualTo(CaptureTimeCalibration.MIN_EACH_WAY - 1);
  }

  @Test
  void aViewFarOffOrTakenStandingStillIsNotCounted() {
    CaptureTimeCalibration calibration = new CaptureTimeCalibration();
    Transform3d view = new Transform3d(new Pose3d().transformBy(MOUNT), TAG);
    calibration.add(
        TAG,
        MOUNT,
        turned(view, Math.toRadians(30)),
        turned(view, Math.toRadians(-40)),
        Rotation2d.ZERO,
        1.5);
    calibration.add(TAG, MOUNT, view, view, Rotation2d.ZERO, 0.05);
    assertThat(calibration.left() + calibration.right()).isZero();
    // The other view counts when it's the one that fits.
    calibration.add(TAG, MOUNT, turned(view, Math.toRadians(60)), view, Rotation2d.ZERO, 1.5);
    assertThat(calibration.left()).isEqualTo(1);
  }
}
