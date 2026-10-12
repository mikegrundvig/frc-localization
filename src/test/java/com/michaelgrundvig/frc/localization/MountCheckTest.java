package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.Random;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Judging a camera's mount by where its tags land: the robot turns as it drives, so a mount that
 * moved puts tags off a different way on the field each time, and the same way on the robot.
 */
class MountCheckTest {
  private static final int SIGHTINGS = 500;
  private static final double TOLERANCE = 0.05;
  private static final Pose3d TAG = new Pose3d(4.0, 3.0, 1.1, Rotation3d.ZERO);

  /** The robot at its k-th sighting: driving about, turned a different way each time. */
  private static Pose3d robot(int k) {
    return new Pose3d(1 + 0.01 * (k % 100), 2 + 0.02 * (k % 50), 0, new Rotation3d(0, 0, 0.37 * k));
  }

  /** A sighting from a camera whose tags land this far off, in the robot's frame. */
  private static void see(MountCheck check, int k, Translation3d offRobot) {
    Pose3d robot = robot(k);
    Pose3d seen =
        new Pose3d(
            TAG.getTranslation().plus(offRobot.rotateBy(robot.getRotation())), TAG.getRotation());
    check.add(robot, seen, TAG);
  }

  @Test
  void aCameraWhereItsConfigurationSaysIsTrustedFully() {
    MountCheck check = new MountCheck(SIGHTINGS);
    for (int k = 0; k < SIGHTINGS; k++) {
      see(check, k, Translation3d.ZERO);
    }

    assertThat(check.judged()).isTrue();
    assertThat(check.offsetMeters()).isCloseTo(0, within(1e-12));
    assertThat(check.outOfCalibration(TOLERANCE)).isFalse();
    assertThat(check.doubtMeters(TOLERANCE)).isZero();
  }

  /** How far its tags were, on average: what turns a doubt in meters into a turn on its mount. */
  @Test
  void itKnowsHowFarItsTagsWere() {
    MountCheck check = new MountCheck(SIGHTINGS);
    assertThat(check.distanceMeters()).as("not judged yet").isNaN();
    double sum = 0;
    for (int k = 0; k < SIGHTINGS; k++) {
      see(check, k, Translation3d.ZERO);
      sum += TAG.getTranslation().getDistance(robot(k).getTranslation());
    }

    assertThat(check.distanceMeters()).isCloseTo(sum / SIGHTINGS, within(1e-9));
  }

  /**
   * Partway around its ring a second time, the check is judged by its latest sightings alone: each
   * new one took the place of the oldest, both in what it averages and in how far its tags were.
   */
  @Test
  void partwayAroundAgainOnlyTheLatestSightingsCount() {
    int ring = 4;
    MountCheck check = new MountCheck(ring);
    double[] offX = {0.30, 0.02, -0.11, 0.07, 0.05, -0.04};
    double[] offY = {-0.20, 0.09, 0.03, -0.06, 0.01, 0.08};
    double[] offZ = {0.10, -0.05, 0.02, 0.04, -0.03, 0.06};
    for (int k = 0; k < offX.length; k++) {
      see(check, k, new Translation3d(offX[k], offY[k], offZ[k]));
    }

    double sumX = 0;
    double sumY = 0;
    double sumZ = 0;
    double sumDistance = 0;
    for (int k = offX.length - ring; k < offX.length; k++) {
      sumX += offX[k];
      sumY += offY[k];
      sumZ += offZ[k];
      Pose3d robot = robot(k);
      Translation3d seen =
          TAG.getTranslation()
              .plus(new Translation3d(offX[k], offY[k], offZ[k]).rotateBy(robot.getRotation()));
      sumDistance += seen.getDistance(robot.getTranslation());
    }
    assertThat(check.offsetMeters())
        .isCloseTo(Math.sqrt(sumX * sumX + sumY * sumY + sumZ * sumZ) / ring, within(1e-12));
    assertThat(check.distanceMeters()).isCloseTo(sumDistance / ring, within(1e-12));
    assertThat(check.doubtMeters(0.01))
        .isCloseTo(
            Math.sqrt((sumX * sumX + sumY * sumY) / (ring * ring) - 0.01 * 0.01), within(1e-12));
  }

  /** Each tag 10 cm off in some direction: noise, which averages out. */
  @Test
  void noiseAveragesOut() {
    MountCheck check = new MountCheck(SIGHTINGS);
    Random random = new Random(7);
    for (int k = 0; k < 2 * SIGHTINGS; k++) {
      see(
          check,
          k,
          new Translation3d(
              0.1 * random.nextGaussian(),
              0.1 * random.nextGaussian(),
              0.1 * random.nextGaussian()));
    }

    assertThat(check.offsetMeters()).isLessThan(0.02);
    assertThat(check.outOfCalibration(TOLERANCE)).isFalse();
    assertThat(check.doubtMeters(TOLERANCE)).isZero();
  }

  /**
   * A camera 10 cm left of where it's configured puts every tag 10 cm right, on the robot, however
   * the robot turns: out of calibration, and its measurements count for less by the part beyond the
   * tolerance, taken off in squares.
   */
  @Test
  void aCameraKnockedAsideIsOutOfCalibrationAndDoubted() {
    MountCheck check = new MountCheck(SIGHTINGS);
    for (int k = 0; k < SIGHTINGS; k++) {
      see(check, k, new Translation3d(0, -0.1, 0));
    }

    assertThat(check.offsetMeters()).isCloseTo(0.1, within(1e-9));
    assertThat(check.outOfCalibration(TOLERANCE)).isTrue();
    assertThat(check.doubtMeters(TOLERANCE))
        .isCloseTo(Math.sqrt(0.1 * 0.1 - TOLERANCE * TOLERANCE), within(1e-9));
  }

  /**
   * A camera 10 cm lower than configured puts its tags 10 cm high: out of calibration, but on level
   * ground it still places the robot right on the floor, so it isn't doubted.
   */
  @Test
  void aCameraKnockedStraightDownIsOutOfCalibrationButNotDoubted() {
    MountCheck check = new MountCheck(SIGHTINGS);
    for (int k = 0; k < SIGHTINGS; k++) {
      see(check, k, new Translation3d(0, 0, 0.1));
    }

    assertThat(check.offsetMeters()).isCloseTo(0.1, within(1e-9));
    assertThat(check.outOfCalibration(TOLERANCE)).isTrue();
    assertThat(check.doubtMeters(TOLERANCE)).isZero();
  }

  @Test
  void aCameraIsJudgedOnlyOnceItHasSeenEnoughTags() {
    MountCheck check = new MountCheck(SIGHTINGS);
    for (int k = 0; k < SIGHTINGS - 1; k++) {
      see(check, k, new Translation3d(0, -0.2, 0));
    }

    assertThat(check.judged()).isFalse();
    assertThat(check.offsetMeters()).isNaN();
    assertThat(check.outOfCalibration(TOLERANCE)).isFalse();
    assertThat(check.doubtMeters(TOLERANCE)).isZero();
    see(check, SIGHTINGS, new Translation3d(0, -0.2, 0));
    assertThat(check.offsetMeters()).as("the average of all it saw").isCloseTo(0.2, within(1e-9));
    assertThat(check.outOfCalibration(TOLERANCE)).isTrue();
  }

  /**
   * A camera judged good, then knocked 12 cm aside: each tag since counts as one of its latest 500,
   * so it's out of calibration once 5/12 of them are new (the 209th), not later.
   */
  @Test
  void aCameraKnockedMidwayIsOutOfCalibrationOnceEnoughOfItsLatestTagsAre() {
    MountCheck check = new MountCheck(SIGHTINGS);
    for (int k = 0; k < SIGHTINGS; k++) {
      see(check, k, Translation3d.ZERO);
    }
    Translation3d knocked = new Translation3d(0, -0.12, 0);
    for (int k = 0; k < 208; k++) {
      see(check, k, knocked);
    }

    assertThat(check.outOfCalibration(TOLERANCE)).as("208 tags since").isFalse();
    see(check, 208, knocked);
    assertThat(check.outOfCalibration(TOLERANCE)).as("209").isTrue();
    for (int k = 209; k < SIGHTINGS; k++) {
      see(check, k, knocked);
    }
    assertThat(check.offsetMeters()).as("all of them").isCloseTo(0.12, within(1e-9));
  }

  /** A camera put back where it belongs is trusted again once its latest tags land right. */
  @Test
  void theOldestSightingsGiveWayToTheNewest() {
    MountCheck check = new MountCheck(SIGHTINGS);
    for (int k = 0; k < SIGHTINGS; k++) {
      see(check, k, new Translation3d(0, -0.1, 0));
    }
    for (int k = 0; k < SIGHTINGS; k++) {
      see(check, k, Translation3d.ZERO);
    }

    assertThat(check.offsetMeters()).isCloseTo(0, within(1e-12));
    assertThat(check.outOfCalibration(TOLERANCE)).isFalse();
  }

  /** A match's worth of tags, a hundred times around the ring, leaves no rounding behind. */
  @Test
  void manyTimesAroundTheRingTheAverageStaysExact() {
    MountCheck check = new MountCheck(SIGHTINGS);
    Random random = new Random(3);
    for (int k = 0; k < 100 * SIGHTINGS; k++) {
      see(check, k, new Translation3d(random.nextGaussian(), random.nextGaussian(), 0));
    }
    for (int k = 0; k < SIGHTINGS; k++) {
      see(check, k, new Translation3d(0, 0.03, 0));
    }

    assertThat(check.offsetMeters()).isCloseTo(0.03, within(1e-12));
  }

  @Test
  void itJudgesByAtLeastOneSighting() {
    assertThatThrownBy(() -> new MountCheck(0)).isInstanceOf(IllegalArgumentException.class);
  }
}
