package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The filter's math, fed by hand: what it does with the errors it considers but never estimates.
 * Every number here can be worked out on paper.
 */
class PoseFilterTest {
  private static final double TURN = Math.toRadians(0.25);
  private static final double SHIFT = 0.003;
  private static final double TAG = 0.01;

  /** How many tags the field has: each kept apart, in its place. */
  private static final int TAGS = 32;

  private static final double HEADING_SHIFT = Math.toRadians(1);

  /**
   * A gyro whose scale may be off by 0.3%, cameras turned 0.25° and shifted 3 mm on their mounts,
   * their headings 1°, and tags 1 cm off the layout.
   */
  private static final PoseFilter.Considered CONSIDERED =
      new PoseFilter.Considered(
          0.003, 0.005, TURN, SHIFT, HEADING_SHIFT, TAG, 1.0, Math.toRadians(45));

  /** Nothing considered: every error is new each time, the textbook filter. */
  private static final PoseFilter.Considered TEXTBOOK =
      new PoseFilter.Considered(0, 0, 0, 0, 0, 0, 1.0, Math.toRadians(45));

  private static PoseFilter placed(
      PoseFilter.Considered considered, double meters, double radians) {
    PoseFilter filter = new PoseFilter(considered, TAGS);
    filter.set(
        new PoseReset(0, Pose2d.ZERO, PoseCovariance.fromStdDevs(meters, meters, radians), "x"), 0);
    return filter;
  }

  /** A position solved with the gyro's heading as the filter had it, from tags this far off. */
  private static VisionMeasurement fix(Translation2d position, Translation2d toTags, double noise) {
    return fix(position, toTags, noise, Rotation2d.ZERO);
  }

  /** A position solved with this heading, from tags this far off. */
  private static VisionMeasurement fix(
      Translation2d position, Translation2d toTags, double noise, Rotation2d solvedWith) {
    return fix(position, toTags, noise, solvedWith, 26);
  }

  /** A position solved with this heading from one tag, this far off. */
  private static VisionMeasurement fix(
      Translation2d position, Translation2d toTag, double noise, Rotation2d solvedWith, int tag) {
    return new VisionMeasurement(
        1,
        new Pose2d(position, solvedWith),
        PoseCovariance.fromStdDevs(noise, noise, Double.POSITIVE_INFINITY),
        new VisionMeasurement.Lever(solvedWith, toTag),
        "front",
        List.of(tag),
        VisionMeasurement.Solver.HEADING_CONSTRAINED);
  }

  /** A measurement of the heading too, from a multi-tag solve. */
  private static VisionMeasurement withHeading(Rotation2d heading, double noise, double radians) {
    return new VisionMeasurement(
        1,
        new Pose2d(Translation2d.ZERO, heading),
        PoseCovariance.fromStdDevs(noise, noise, radians),
        VisionMeasurement.Lever.NONE,
        "front",
        List.of(25, 26),
        VisionMeasurement.Solver.MULTI_TAG);
  }

  /** Each tag's place among the filter's: its number, within the places there are. */
  private static int[] places(VisionMeasurement measurement) {
    return measurement.tagIds().stream().mapToInt(id -> id % TAGS).toArray();
  }

  private static void apply(PoseFilter filter, VisionMeasurement measurement, int camera) {
    apply(filter, measurement, 0, camera);
  }

  private static void apply(
      PoseFilter filter, VisionMeasurement measurement, double yaw, int camera) {
    filter.correct(
        Objects.requireNonNull(filter.innovation(measurement, yaw, camera, places(measurement))));
  }

  @Test
  void withNothingConsideredItIsTheTextbookKalmanUpdate() {
    PoseFilter filter = placed(TEXTBOOK, 0.1, 0.01);

    apply(filter, fix(new Translation2d(0.2, 0), Translation2d.ZERO, 0.1), 0);

    // Equal trust: halfway, and the variance halved.
    assertThat(filter.x()).isCloseTo(0.1, within(1e-12));
    assertThat(filter.covariance().xx()).isCloseTo(0.005, within(1e-12));
    assertThat(filter.covariance().heading()).isCloseTo(1e-4, within(1e-15));
  }

  /**
   * However many images one camera takes of the same tag, its shift on its mount and the tag's
   * placement are in every one: the position can't be surer than they along the tag, nor than they
   * and its turn together square to it.
   */
  @Test
  void oneCameraCantPlaceTheRobotBetterThanItsRepeatedErrorsAllow() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0); // the heading exactly known
    for (int i = 0; i < 2000; i++) {
      apply(filter, fix(Translation2d.ZERO, new Translation2d(3, 0), 0.05), 0);
    }

    PoseCovariance covariance = filter.covariance();
    double along = Math.hypot(SHIFT, TAG);
    assertThat(covariance.xStdDev()).isCloseTo(along, within(0.02 * along));
    double square = Math.hypot(along, 3 * TURN); // 1.73 cm
    assertThat(covariance.yStdDev()).isCloseTo(square, within(0.02 * square));
  }

  /**
   * Two cameras seeing the same tag, each with its own shift, together do better than either: but
   * only at their shifts, by √2. The tag's placement they share, and its doubt stays whole.
   */
  @Test
  void twoCamerasSeeingOneTagShareItsPlacement() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0);
    for (int i = 0; i < 2000; i++) {
      apply(filter, fix(Translation2d.ZERO, new Translation2d(3, 0), 0.05), i % 2);
    }

    double shared = Math.hypot(TAG, SHIFT / Math.sqrt(2));
    assertThat(filter.covariance().xStdDev()).isCloseTo(shared, within(0.02 * shared));
  }

  /** Two cameras seeing a tag each do better by √2: neither the shifts nor the tags are shared. */
  @Test
  void twoCamerasSeeingATagEachDoBetter() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0);
    for (int i = 0; i < 2000; i++) {
      int camera = i % 2;
      apply(
          filter,
          fix(Translation2d.ZERO, new Translation2d(3, 0), 0.05, Rotation2d.ZERO, 25 + camera),
          camera);
    }

    double each = Math.hypot(TAG, SHIFT) / Math.sqrt(2);
    assertThat(filter.covariance().xStdDev()).isCloseTo(each, within(0.02 * each));
  }

  /**
   * A camera without a place among the considered errors (beyond {@link
   * ConsideredErrors#MAX_CAMERAS}) still has its turn and shift, in every image: they're taken as
   * new each time, so its images average them down with their noise, as the textbook would. A tag
   * without a place, likewise.
   */
  @Test
  void aCameraAndATagWithoutAPlaceCountEachImagesErrorsAsNew() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0);
    VisionMeasurement fix = fix(Translation2d.ZERO, new Translation2d(3, 0), 0.005);
    for (int i = 0; i < 2500; i++) {
      filter.correct(Objects.requireNonNull(filter.innovation(fix, 0, -1, new int[] {-1})));
    }

    // 5 mm of noise, the shift and the tag's placement in each image, averaged over 2,500 of them.
    double each = Math.hypot(0.005, Math.hypot(SHIFT, TAG)) / Math.sqrt(2500);
    assertThat(filter.covariance().xStdDev()).isCloseTo(each, within(0.02 * each));
  }

  /**
   * Turned around, a camera's shift on its mount points the other way on the field: its fixes from
   * both ways cancel it, and only the tags' placement is left.
   */
  @Test
  void aCamerasShiftTurnsWithTheRobot() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0);
    for (int i = 0; i < 2000; i++) {
      apply(filter, fix(Translation2d.ZERO, new Translation2d(3, 0), 0.05), 0);
    }
    double facingOneWay = filter.covariance().xStdDev();

    // Half a turn, the heading as sure as before; the camera now sees a tag behind, as sure.
    filter.predict(0, Math.PI, 0, 0, 0, 0, YawSource.WHEELS);
    Rotation2d around = new Rotation2d(Math.PI);
    for (int i = 0; i < 2000; i++) {
      apply(
          filter, fix(Translation2d.ZERO, new Translation2d(-3, 0), 0.05, around, 27), Math.PI, 0);
    }

    // Nothing could do better than the two tags alone, TAG / √2; the filter, which never estimates
    // the shift, gets most of the way there.
    assertThat(facingOneWay).isCloseTo(Math.hypot(SHIFT, TAG), within(0.02 * TAG));
    assertThat(filter.covariance().xStdDev()).isBetween(TAG / Math.sqrt(2), 0.8 * facingOneWay);
  }

  /**
   * Turning on, a camera sees its tags from elsewhere and what its headings share is a new error:
   * its headings count again, and the heading gets surer than standing still allowed.
   */
  @Test
  void asTheRobotTurnsACamerasHeadingsShiftBecomesANewOne() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0.1);
    for (int i = 0; i < 2000; i++) {
      apply(filter, withHeading(Rotation2d.ZERO, 0.5, 0.05), 0);
    }
    double standing = filter.covariance().headingStdDev();

    // A quarter turn on the wheels (no scale error), the turn itself sure: the shift mostly
    // forgotten, e^-2 of it left.
    filter.predict(0, Math.PI / 2, 0, 0, 0, 0, YawSource.WHEELS);
    for (int i = 0; i < 2000; i++) {
      apply(filter, withHeading(new Rotation2d(Math.PI / 2), 0.5, 0.05), Math.PI / 2, 0);
    }

    assertThat(standing).isCloseTo(Math.hypot(TURN, HEADING_SHIFT), within(0.03 * HEADING_SHIFT));
    assertThat(filter.covariance().headingStdDev()).isLessThan(0.85 * standing);
  }

  /**
   * A gyro's scale error grows the heading's in proportion to the turn, the same each turn: a turn
   * one way and back again leaves no doubt from it at all, where a random walk would leave √2.
   */
  @Test
  void theGyrosScaleErrorIsTheSameEveryTurn() {
    PoseFilter filter = placed(CONSIDERED, 0.01, 0);

    filter.predict(0, 2 * Math.PI, 0, 0, 0, 0, YawSource.GYRO);
    double afterATurn = filter.covariance().headingStdDev();
    filter.predict(2 * Math.PI, 2 * Math.PI, 0, 0, 0, 0, YawSource.GYRO);
    double afterTwo = filter.covariance().headingStdDev();
    filter.predict(4 * Math.PI, -4 * Math.PI, 0, 0, 0, 0, YawSource.GYRO);

    assertThat(afterATurn).isCloseTo(0.003 * 2 * Math.PI, within(1e-12));
    assertThat(afterTwo).as("twice as far off").isCloseTo(2 * afterATurn, within(1e-12));
    assertThat(filter.covariance().headingStdDev()).as("back where it started").isLessThan(1e-9);
  }

  /**
   * Each IMU's scale is its own: three turns one way on the gyro, then three back on the backup IMU
   * while the gyro is lost, leave the heading as unsure as both scales over three turns each, not
   * back where it started as one scale's turns there and back would.
   */
  @Test
  void eachImusScaleIsItsOwn() {
    PoseFilter filter = placed(CONSIDERED, 0.01, 0);

    filter.predict(0, 6 * Math.PI, 0, 0, 0, 0, YawSource.GYRO);
    filter.predict(6 * Math.PI, -6 * Math.PI, 0, 0, 0, 0, YawSource.BACKUP);

    double gyro = 0.003 * 6 * Math.PI;
    double backup = 0.005 * 6 * Math.PI;
    assertThat(filter.covariance().headingStdDev())
        .isCloseTo(Math.hypot(gyro, backup), within(1e-12));
  }

  /** Turned by the wheels while the gyro is lost, the gyro's scale has nothing to do with it. */
  @Test
  void theWheelsTurningLeaveTheGyrosScaleOut() {
    PoseFilter filter = placed(CONSIDERED, 0.01, 0);

    filter.predict(0, 2 * Math.PI, 0, 0, 0, 0, YawSource.WHEELS);

    assertThat(filter.covariance().headingStdDev()).isZero();
  }

  /**
   * A camera's headings share its turn on its mount and its shift: however many there are, they
   * can't make the heading surer than those two together.
   */
  @Test
  void oneCamerasHeadingsCantBeatItsTurnAndShift() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0.1);
    for (int i = 0; i < 2000; i++) {
      apply(filter, withHeading(Rotation2d.ZERO, 0.5, 0.05), 0);
    }

    // A little above it: the images' own noise, 0.05 rad over 2,000 of them, adds in squares.
    double shared = Math.hypot(TURN, HEADING_SHIFT);
    assertThat(filter.covariance().headingStdDev()).isCloseTo(shared, within(0.03 * shared));
  }

  /**
   * A position solved with a heading a degree off the estimate's lands 3 m times a degree aside, as
   * that heading puts it: the filter reads it through its lever and finds the robot where it was,
   * as it would a late measurement whose heading the estimate has since corrected.
   */
  @Test
  void aPositionSolvedWithAnotherHeadingIsReadThroughItsLever() {
    PoseFilter filter = placed(TEXTBOOK, 0.1, 1e-6);
    double off = Math.toRadians(1);
    // Tags 3 m ahead along x, seen with the heading a degree to the left: the solve put the robot
    // 3 m times a degree to the right.
    Translation2d solved = new Translation2d(0, -3 * off);

    apply(filter, fix(solved, new Translation2d(3, 0), 0.05, new Rotation2d(off)), 0);

    assertThat(filter.x()).isCloseTo(0, within(1e-6));
    assertThat(filter.y())
        .as("where the robot is, not where the solve put it")
        .isCloseTo(0, within(1e-6));
  }

  /** The gain the filter holds is for its latest innovation: an older one can't be applied. */
  @Test
  void onlyTheLatestInnovationCanBeApplied() {
    PoseFilter filter = placed(TEXTBOOK, 0.1, 0.01);
    VisionMeasurement here = fix(Translation2d.ZERO, Translation2d.ZERO, 0.1);
    VisionMeasurement there = fix(new Translation2d(0.2, 0), Translation2d.ZERO, 0.1);
    PoseFilter.Innovation first =
        Objects.requireNonNull(filter.innovation(here, 0, 0, places(here)));
    PoseFilter.Innovation second =
        Objects.requireNonNull(filter.innovation(there, 0, 0, places(there)));

    assertThatThrownBy(() -> filter.correct(first))
        .isInstanceOf(IllegalStateException.class)
        .hasMessageContaining("1 more came between");
    filter.correct(second);
    assertThat(filter.x()).isCloseTo(0.1, within(1e-12));
  }

  @Test
  void aMeasurementWithHeadingTurnsTheHeading() {
    PoseFilter filter = placed(TEXTBOOK, 1.0, 0.1);

    apply(filter, withHeading(new Rotation2d(0.1), 1.0, 0.1), 0);

    assertThat(filter.headingOffset()).isCloseTo(0.05, within(1e-9));
  }

  /** What the history keeps, it gives back exactly: the covariance and every relation. */
  @Test
  void theUncertaintyComesBackAsKept() {
    PoseFilter filter = placed(CONSIDERED, 0.1, 0.02);
    filter.predict(0, 1, 1, 0.5, 0.001, 0.0001, YawSource.GYRO);
    apply(filter, fix(new Translation2d(1, 0.6), new Translation2d(2, 1), 0.05), 3);
    double[] kept = new double[filter.uncertaintyLength()];
    filter.copyUncertaintyTo(kept, 0);
    PoseFilter other = new PoseFilter(CONSIDERED, TAGS);

    other.restore(filter.x(), filter.y(), filter.headingOffset(), kept, 0);
    apply(filter, fix(new Translation2d(1.1, 0.5), new Translation2d(2, 1), 0.05), 3);
    apply(other, fix(new Translation2d(1.1, 0.5), new Translation2d(2, 1), 0.05), 3);

    assertThat(other.covariance()).isEqualTo(filter.covariance());
    assertThat(other.x()).isEqualTo(filter.x());
  }

  /**
   * The covariance the estimate reports is always one. A variance a hair under zero, or x and y
   * varying together a hair past what their variances allow, as rounding can leave them, come back
   * at the bound; one that isn't a number, or is infinite, which only a bug could leave, comes back
   * as knowing nothing (100 m either way, a half turn), never as NaN or a throw in the robot loop.
   */
  @Test
  void theCovarianceReportedIsAlwaysOne() {
    PoseFilter filter = placed(TEXTBOOK, 0.1, 0.1);
    double[] kept = new double[filter.uncertaintyLength()];
    filter.copyUncertaintyTo(kept, 0);
    // Row by row: x, y, and the heading's offset; xx at 0, xy at 1, yy at 4, the heading's at 8.
    double[] underZero = kept.clone();
    underZero[0] = -1e-18;
    underZero[1] = 1e-12;
    underZero[3] = 1e-12;
    underZero[8] = -1e-20;
    double[] pastTheirVariances = kept.clone();
    pastTheirVariances[1] = 0.0100001;
    pastTheirVariances[3] = 0.0100001;

    PoseCovariance rounded = restored(underZero);
    assertThat(rounded.xx()).isZero();
    assertThat(rounded.xy()).as("no more than the variances allow").isZero();
    assertThat(rounded.heading()).isZero();
    PoseCovariance together = restored(pastTheirVariances);
    assertThat(together.xy()).isCloseTo(Math.sqrt(together.xx() * together.yy()), within(1e-15));
    for (double broken : new double[] {Double.NaN, Double.POSITIVE_INFINITY}) {
      for (int at : new int[] {0, 1, 4, 8}) {
        double[] bug = kept.clone();
        bug[at] = broken;
        PoseCovariance noIdea = restored(bug);
        assertThat(noIdea.xStdDev()).as("%s at %d", broken, at).isEqualTo(100);
        assertThat(noIdea.yStdDev()).isEqualTo(100);
        assertThat(noIdea.xy()).isZero();
        assertThat(noIdea.headingStdDev()).isCloseTo(Math.PI, within(1e-12));
      }
    }
  }

  /** The covariance a filter given this uncertainty reports. */
  private static PoseCovariance restored(double[] uncertainty) {
    PoseFilter filter = new PoseFilter(TEXTBOOK, TAGS);
    filter.restore(0, 0, 0, uncertainty, 0);
    return filter.covariance();
  }

  /**
   * Set afresh, a position or a heading has nothing to do with what came before: the next camera
   * fix weighs only its own errors against it.
   */
  @Test
  void aResetForgetsHowTheEstimateRelatedToTheConsideredErrors() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0);
    for (int i = 0; i < 2000; i++) {
      apply(filter, fix(Translation2d.ZERO, new Translation2d(3, 0), 0.05), 0);
    }
    filter.shift(0, 0, 0.0001);
    PoseFilter fresh = placed(CONSIDERED, 0.01, 0);

    apply(filter, fix(new Translation2d(0.01, 0), new Translation2d(3, 0), 0.05), 0);
    apply(fresh, fix(new Translation2d(0.01, 0), new Translation2d(3, 0), 0.05), 0);

    assertThat(filter.x()).isCloseTo(fresh.x(), within(1e-12));
    assertThat(filter.covariance().xx()).isCloseTo(fresh.covariance().xx(), within(1e-15));
  }
}
