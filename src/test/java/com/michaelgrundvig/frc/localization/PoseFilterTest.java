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
    filter.predict(0, Math.PI, 0, 0, 0, 0, 0, YawSource.WHEELS);
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
    filter.predict(0, Math.PI / 2, 0, 0, 0, 0, 0, YawSource.WHEELS);
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

    filter.predict(0, 2 * Math.PI, 0, 0, 0, 0, 0, YawSource.GYRO);
    double afterATurn = filter.covariance().headingStdDev();
    filter.predict(2 * Math.PI, 2 * Math.PI, 0, 0, 0, 0, 0, YawSource.GYRO);
    double afterTwo = filter.covariance().headingStdDev();
    filter.predict(4 * Math.PI, -4 * Math.PI, 0, 0, 0, 0, 0, YawSource.GYRO);

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

    filter.predict(0, 6 * Math.PI, 0, 0, 0, 0, 0, YawSource.GYRO);
    filter.predict(6 * Math.PI, -6 * Math.PI, 0, 0, 0, 0, 0, YawSource.BACKUP);

    double gyro = 0.003 * 6 * Math.PI;
    double backup = 0.005 * 6 * Math.PI;
    assertThat(filter.covariance().headingStdDev())
        .isCloseTo(Math.hypot(gyro, backup), within(1e-12));
  }

  /** Turned by the wheels while the gyro is lost, the gyro's scale has nothing to do with it. */
  @Test
  void theWheelsTurningLeaveTheGyrosScaleOut() {
    PoseFilter filter = placed(CONSIDERED, 0.01, 0);

    filter.predict(0, 2 * Math.PI, 0, 0, 0, 0, 0, YawSource.WHEELS);

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
    filter.predict(0, 1, 1, 0.5, 0, 0.001, 0.0001, YawSource.GYRO);
    apply(filter, fix(new Translation2d(1, 0.6), new Translation2d(2, 1), 0.05), 3);
    double[] kept = new double[filter.uncertaintyLength()];
    filter.copyUncertaintyTo(kept, 0);
    PoseFilter other = new PoseFilter(CONSIDERED, TAGS);

    other.restore(
        filter.x(),
        filter.y(),
        filter.headingOffset(),
        filter.gyroScale(),
        filter.gyroDrift(),
        kept,
        0);
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
    // Row by row, five to a row: x, y, the heading's offset, and the gyro's scale and drift; xx at
    // 0, xy at 1 and 5, yy at 6, the heading's at 12.
    double[] underZero = kept.clone();
    underZero[0] = -1e-18;
    underZero[1] = 1e-12;
    underZero[5] = 1e-12;
    underZero[12] = -1e-20;
    double[] pastTheirVariances = kept.clone();
    pastTheirVariances[1] = 0.0100001;
    pastTheirVariances[5] = 0.0100001;

    PoseCovariance rounded = restored(underZero);
    assertThat(rounded.xx()).isZero();
    assertThat(rounded.xy()).as("no more than the variances allow").isZero();
    assertThat(rounded.heading()).isZero();
    PoseCovariance together = restored(pastTheirVariances);
    assertThat(together.xy()).isCloseTo(Math.sqrt(together.xx() * together.yy()), within(1e-15));
    for (double broken : new double[] {Double.NaN, Double.POSITIVE_INFINITY}) {
      for (int at : new int[] {0, 1, 6, 12}) {
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
    filter.restore(0, 0, 0, 0, 0, uncertainty, 0);
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

  // ---- The gyro's scale, where the filter estimates it ----

  /** Only the gyro's scale is in doubt, by 1%: every other error is new each time. */
  private static final PoseFilter.Considered SCALE_ALONE =
      new PoseFilter.Considered(0.01, 0, 0, 0, 0, 0, 1.0, Math.toRadians(45));

  /**
   * A gyro whose drift is 0.001 rad/s in doubt at first, its scale wandering 0.0003 in a second and
   * its drift 0.0002 rad/s.
   */
  private static final PoseFilter.GyroDoubt WANDERING =
      new PoseFilter.GyroDoubt(0.001, 0.0003, 0.0002);

  /** What a gyro reading 2% high shows for a quarter turn, and the turn the robot really made. */
  private static final double SHOWN = 1.02 * Math.PI / 2;

  private static final double REAL = Math.PI / 2;

  /** A filter sure of where it starts and which way it faces, in doubt of the gyro's scale. */
  private static PoseFilter sureOfAllButTheScale(
      boolean positionFixesCorrectHeading, PoseFilter.GyroErrors gyroErrors) {
    PoseFilter filter =
        new PoseFilter(SCALE_ALONE, TAGS, positionFixesCorrectHeading, gyroErrors, WANDERING);
    filter.set(new PoseReset(0, Pose2d.ZERO, PoseCovariance.fromStdDevs(0.01, 0.01, 0), "x"), 0);
    return filter;
  }

  /**
   * A heading found off after a turn is laid to the gyro's scale, since nothing else was in doubt:
   * the scale is the miss over the turn. The next turn is then right with no camera at all, where a
   * filter that only considers the scale is off by as much again.
   */
  @Test
  void aHeadingFoundOffAfterATurnIsLaidToTheGyrosScale() {
    PoseFilter estimating = sureOfAllButTheScale(false, PoseFilter.GyroErrors.ESTIMATED);
    PoseFilter considering = sureOfAllButTheScale(false, PoseFilter.GyroErrors.CONSIDERED);
    for (PoseFilter filter : List.of(estimating, considering)) {
      filter.predict(0, SHOWN, 0, 0, 0, 0, 0, YawSource.GYRO);
      apply(filter, withHeading(new Rotation2d(REAL), 1.0, 1e-4), SHOWN, 0);
      filter.predict(SHOWN, SHOWN, 0, 0, 0, 0, 0, YawSource.GYRO);
    }

    // Each radian the gyro shows adds this to the heading: the real turn less the shown, over it.
    double scale = (REAL - SHOWN) / SHOWN;
    assertThat(estimating.gyroScale()).isCloseTo(scale, within(2e-4));
    assertThat(estimating.gyroScaleStdDev()).as("surer than it began").isLessThan(0.001);
    assertThat(2 * SHOWN + estimating.headingOffset())
        .as("the second turn, with no camera")
        .isCloseTo(2 * REAL, within(1e-3));
    assertThat(estimating.covariance().headingStdDev()).isLessThan(Math.toRadians(0.1));

    assertThat(considering.gyroScale()).as("never corrected").isZero();
    assertThat(considering.gyroScaleStdDev()).isCloseTo(0.01, within(1e-15));
    assertThat(2 * SHOWN + considering.headingOffset())
        .as("the second turn's miss, uncorrected")
        .isCloseTo(2 * REAL + (SHOWN - REAL), within(1e-3));
    assertThat(considering.covariance().headingStdDev())
        .as("and its doubt, a quarter turn's worth")
        .isCloseTo(0.01 * SHOWN, within(1e-4));
  }

  /**
   * A position solved with the gyro's heading corrects the scale only where it may correct the
   * heading: the heading is all the scale shows itself in. The fix here lands where a heading 0.03
   * rad off puts it, 3 m from its tags.
   */
  @Test
  void aPositionFixCorrectsTheScaleOnlyWhereItCorrectsTheHeading() {
    PoseFilter leavingHeading = sureOfAllButTheScale(false, PoseFilter.GyroErrors.ESTIMATED);
    PoseFilter correctingHeading = sureOfAllButTheScale(true, PoseFilter.GyroErrors.ESTIMATED);
    for (PoseFilter filter : List.of(leavingHeading, correctingHeading)) {
      filter.predict(0, SHOWN, 0, 0, 0, 0, 0, YawSource.GYRO);
      Rotation2d solvedWith = new Rotation2d(SHOWN);
      apply(
          filter,
          fix(new Translation2d(0, -0.09), new Translation2d(3, 0), 0.001, solvedWith),
          SHOWN,
          0);
    }

    assertThat(leavingHeading.gyroScale()).isZero();
    assertThat(leavingHeading.headingOffset()).isZero();
    assertThat(correctingHeading.headingOffset()).as("turned toward the fix").isLessThan(-0.01);
    assertThat(correctingHeading.gyroScale()).as("and the scale with it").isLessThan(-0.005);
  }

  /**
   * The gyro's errors may wander: over a second the gyro turns the robot, each one's doubt grows by
   * what it may have wandered in a second, and over four seconds by twice that. Where they are only
   * considered, the scale's doubt stays as told and there is no drift to doubt.
   */
  @Test
  void theGyrosErrorsDoubtGrowsAsTheyWander() {
    PoseFilter estimating = sureOfAllButTheScale(false, PoseFilter.GyroErrors.ESTIMATED);
    PoseFilter considering = sureOfAllButTheScale(false, PoseFilter.GyroErrors.CONSIDERED);
    assertThat(estimating.gyroDriftStdDev()).isCloseTo(0.001, within(1e-15));

    estimating.predict(0, 0, 0, 0, 1, 0, 0, YawSource.GYRO);
    considering.predict(0, 0, 0, 0, 1, 0, 0, YawSource.GYRO);

    assertThat(estimating.gyroScaleStdDev()).isCloseTo(Math.hypot(0.01, 0.0003), within(1e-12));
    assertThat(estimating.gyroDriftStdDev()).isCloseTo(Math.hypot(0.001, 0.0002), within(1e-12));
    // A second of an unsure drift is that much unsure heading, beside what was there.
    assertThat(estimating.covariance().headingStdDev()).isCloseTo(0.001, within(1e-12));
    estimating.predict(0, 0, 0, 0, 3, 0, 0, YawSource.WHEELS);
    assertThat(estimating.gyroScaleStdDev())
        .as("the wheels turning the robot leave the gyro's errors as they were")
        .isCloseTo(Math.hypot(0.01, 0.0003), within(1e-12));
    assertThat(considering.gyroScaleStdDev()).isCloseTo(0.01, within(1e-15));
    assertThat(considering.gyroDriftStdDev()).isZero();
    assertThat(considering.covariance().headingStdDev()).isZero();
  }

  /**
   * A heading found off after standing still is laid to the gyro's drift, since the gyro turned
   * nothing for its scale to misread: the drift is the miss over the time. The next stretch is then
   * right with no camera.
   */
  @Test
  void aHeadingFoundOffAfterStandingIsLaidToTheGyrosDrift() {
    PoseFilter filter = sureOfAllButTheScale(false, PoseFilter.GyroErrors.ESTIMATED);
    // Ten seconds in which the gyro's yaw crept 0.005 rad while the robot stood still.
    double crept = 0.005;
    filter.predict(0, crept, 0, 0, 10, 0, 0, YawSource.GYRO);
    apply(filter, withHeading(Rotation2d.ZERO, 1.0, 1e-4), crept, 0);

    assertThat(filter.gyroDrift()).as("radians a second").isCloseTo(-crept / 10, within(2e-5));
    assertThat(crept + filter.headingOffset()).isCloseTo(0, within(1e-4));
    filter.predict(crept, crept, 0, 0, 10, 0, 0, YawSource.GYRO);
    assertThat(2 * crept + filter.headingOffset())
        .as("ten seconds more, with no camera")
        .isCloseTo(0, within(5e-4));
  }

  /**
   * The scale is the gyro's, not the pose's. A heading told afresh keeps the scale already found,
   * and the turns after it are corrected by it; what the heading is told, it is, as sure as told.
   */
  @Test
  void aHeadingToldAfreshKeepsTheScaleFound() {
    PoseFilter filter = sureOfAllButTheScale(false, PoseFilter.GyroErrors.ESTIMATED);
    filter.predict(0, SHOWN, 0, 0, 0, 0, 0, YawSource.GYRO);
    apply(filter, withHeading(new Rotation2d(REAL), 1.0, 1e-4), SHOWN, 0);
    double found = filter.gyroScale();

    filter.resetHeading(0.5, 1e-6);

    assertThat(filter.gyroScale()).isEqualTo(found);
    assertThat(filter.covariance().headingStdDev()).isCloseTo(1e-3, within(1e-12));
    filter.predict(SHOWN, SHOWN, 0, 0, 0, 0, 0, YawSource.GYRO);
    assertThat(filter.headingOffset()).isCloseTo(0.5 + found * SHOWN, within(1e-12));
    // The turn before the reset no longer bears on the heading: only this one's share of the
    // scale's remaining doubt is added.
    assertThat(filter.covariance().headingStdDev())
        .isCloseTo(Math.hypot(1e-3, filter.gyroScaleStdDev() * SHOWN), within(1e-9));
  }

  /**
   * A camera known to put its measurements off has that taken off the measurement before it is
   * compared, and what is unsure of it counts as more noise, after the gate's test. With the
   * estimate's position good to 10 cm each way, and a measurement whose noise is [4 2; 2 9] cm²,
   * the innovation's covariance is S = [104 2; 2 109] cm², and the gate tests d^T S^-1 d. The gain
   * is then worked out with 100 cm² of doubt added to each axis of S.
   */
  @Test
  void aKnownMissIsTakenOffAndItsDoubtCountsAfterTheGate() {
    PoseFilter filter = placed(TEXTBOOK, 0.1, 0);
    VisionMeasurement measurement =
        new VisionMeasurement(
            1,
            new Pose2d(0.3, -0.2, Rotation2d.ZERO),
            new PoseCovariance(0.0004, 0.0002, 0.0009, Double.POSITIVE_INFINITY),
            new VisionMeasurement.Lever(Rotation2d.ZERO, new Translation2d(2, 1)),
            "front",
            List.of(26),
            VisionMeasurement.Solver.HEADING_CONSTRAINED);

    PoseFilter.Innovation innovation =
        Objects.requireNonNull(
            filter.innovation(measurement, 0, 3, places(measurement), 0.05, -0.08, 0.01));

    double dx = 0.25;
    double dy = -0.12;
    assertThat(innovation.xMeters()).isCloseTo(dx, within(1e-12));
    assertThat(innovation.yMeters()).isCloseTo(dy, within(1e-12));
    double chiSquared = quadratic(0.0104, 0.0002, 0.0109, dx, dy);
    assertThat(chiSquared).isCloseTo(7.439, within(0.001));
    assertThat(innovation.positionChiSquared()).isCloseTo(chiSquared, within(1e-9));
    assertThat(innovation.chiSquared()).isCloseTo(chiSquared, within(1e-9));

    // The estimate moves by P S^-1 d, with the doubt in S: P is 100 cm² each way.
    filter.correct(innovation);
    double determinant = 0.0204 * 0.0209 - 0.0002 * 0.0002;
    assertThat(filter.x())
        .isCloseTo(0.01 * (0.0209 * dx - 0.0002 * dy) / determinant, within(1e-12));
    assertThat(filter.y())
        .isCloseTo(0.01 * (0.0204 * dy - 0.0002 * dx) / determinant, within(1e-12));
  }

  /** d^T S^-1 d for S = [sxx sxy; sxy syy]. */
  private static double quadratic(double sxx, double sxy, double syy, double dx, double dy) {
    return (syy * dx * dx - 2 * sxy * dx * dy + sxx * dy * dy) / (sxx * syy - sxy * sxy);
  }

  /**
   * A heading claimed exactly, against an estimate sure of its own, leaves no spread to measure the
   * heading's distance in: the measurement cannot be compared, though its position could be.
   */
  @Test
  void aMeasurementWithNoSpreadInItsHeadingIsNotCompared() {
    PoseFilter filter = placed(TEXTBOOK, 0.1, 0);
    VisionMeasurement exact = withHeading(new Rotation2d(0.1), 0.05, 0);

    assertThat(filter.innovation(exact, 0, 3, places(exact))).isNull();
    VisionMeasurement unsure = withHeading(new Rotation2d(0.1), 0.05, 0.01);
    assertThat(filter.innovation(unsure, 0, 3, places(unsure))).isNotNull();
  }
}
