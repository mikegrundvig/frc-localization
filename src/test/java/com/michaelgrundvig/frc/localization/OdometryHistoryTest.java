package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The estimator's history of odometry steps, filled by hand. Step k, of seven, ends 10 ms after the
 * one before, with:
 *
 * <ul>
 *   <li>the yaw 0.02 rad on from the step before, by the gyro, but for step 3, which the wheels
 *       turned;
 *   <li>the robot carried 0.01 k m along x and 0.02 m back along y by odometry alone, which adds a
 *       position variance of 0.0001 k;
 *   <li>the filter at (1 + 0.1 k, 2 - 0.05 k) as it ended.
 * </ul>
 */
class OdometryHistoryTest {
  private static final long STEP_NANOS = 10_000_000L;
  private static final double YAW_AT_START = 0.5;
  private static final double YAW_EACH_STEP = 0.02;
  private static final int WHEELS_TURNED_STEP = 3;
  private static final int STEPS = 7;

  private final PoseFilter filter =
      new PoseFilter(new ConsideredSizes(0, 0, 0, 0, 0, 0, 1.0, Math.toRadians(45)), 4);

  /** Room for the start and seven steps, and no more. */
  private final OdometryHistory history =
      new OdometryHistory(STEPS + 1, filter.uncertaintyLength());

  private void filterAt(double x, double y) {
    filter.set(
        new PoseReset(
            0, new Pose2d(x, y, Rotation2d.ZERO), PoseCovariance.fromStdDevs(0.1, 0.1, 0.1), "x"),
        0);
  }

  private int add(int k) {
    YawSource turnedBy = k == WHEELS_TURNED_STEP ? YawSource.WHEELS : YawSource.GYRO;
    int step =
        history.add(k * STEP_NANOS, 0.03, 0.01, 0, 0, turnedBy, YAW_AT_START + YAW_EACH_STEP * k);
    history.movedOverNewestStep(0.01 * k, -0.02, 0.0001 * k);
    filterAt(1 + 0.1 * k, 2 - 0.05 * k);
    history.store(step, filter);
    return step;
  }

  @BeforeEach
  void fill() {
    history.start(0, YawSource.GYRO, YAW_AT_START);
    filterAt(1, 2);
    history.store(OdometryHistory.OLDEST_STEP, filter);
    for (int k = 1; k <= STEPS; k++) {
      assertThat(add(k)).isEqualTo(k);
    }
  }

  @Test
  void odometryAloneCarriesTheOldestStepsPlaceOn() {
    // Through step 3 it carried 0.01 (1 + 2 + 3) along x, and half of step 4 is 0.02 more.
    assertThat(history.xByOdometryAlone(4, 0.5)).isCloseTo(1 + 0.06 + 0.02, within(1e-12));
    assertThat(history.yByOdometryAlone(4, 0.5)).isCloseTo(2 - 0.06 - 0.01, within(1e-12));
    assertThat(history.xByOdometryAlone(4, 1)).isCloseTo(1.10, within(1e-12));
    assertThat(history.varianceByOdometryAlone(4)).isCloseTo(0.0010, within(1e-12));

    assertThat(history.xByOdometryAlone(OdometryHistory.OLDEST_STEP, 0.7)).isEqualTo(1);
    assertThat(history.yByOdometryAlone(OdometryHistory.OLDEST_STEP, 0.7)).isEqualTo(2);
    assertThat(history.varianceByOdometryAlone(OdometryHistory.OLDEST_STEP)).isZero();
  }

  @Test
  void theRobotsRatesAreItsChangeOverTheLastFiveSteps() {
    // Steps 3 to 7 carried 0.01 (3 + 4 + 5 + 6 + 7) along x in 50 ms.
    assertThat(history.velocityX(7)).isCloseTo(0.25 / 0.05, within(1e-9));
    assertThat(history.velocityY(7)).isCloseTo(-0.10 / 0.05, within(1e-9));
    assertThat(history.yawRate(7)).isCloseTo(5 * YAW_EACH_STEP / 0.05, within(1e-9));

    // Step 4 has only four steps behind it.
    assertThat(history.velocityX(4)).isCloseTo(0.10 / 0.04, within(1e-9));
    assertThat(history.velocityX(OdometryHistory.OLDEST_STEP)).as("nothing before it").isZero();
    assertThat(history.yawRate(OdometryHistory.OLDEST_STEP)).isZero();
  }

  /**
   * The pose half way through step 5, understood with today's heading offset (0.3 rad), a gyro
   * scale of 0.1 and a drift of 2 rad/s. Since then the gyro has turned the robot half of step 5
   * and steps 6 and 7, 0.05 rad over 25 ms, so the offset then was 0.3 - 0.1 × 0.05 - 2 × 0.025.
   */
  @Test
  void aPastPoseIsUnderstoodWithWhatTheGyroHasGotWrongSince() {
    Pose2d then = history.poseAt(45_000_000L, 0.3, 0.1, 2.0);

    assertThat(then.getX()).isCloseTo(1.45, within(1e-12));
    assertThat(then.getY()).isCloseTo(1.775, within(1e-12));
    double yawThen = YAW_AT_START + 4.5 * YAW_EACH_STEP;
    assertThat(then.getRotation().getRadians())
        .isCloseTo(yawThen + 0.3 - 0.005 - 0.05, within(1e-12));
  }

  /**
   * At the oldest step, the gyro has turned the robot six steps since, the wheels the seventh: 0.12
   * rad over 60 ms.
   */
  @Test
  void theOldestPoseIsUnderstoodTheSameWay() {
    Pose2d oldest = history.poseAt(-5, 0.3, 0.1, 2.0);

    assertThat(oldest.getX()).isEqualTo(1);
    assertThat(oldest.getY()).isEqualTo(2);
    assertThat(oldest.getRotation().getRadians())
        .isCloseTo(YAW_AT_START + 0.3 - 0.1 * 0.12 - 2.0 * 0.06, within(1e-12));
  }

  @Test
  void aDriftSeenAtRestStaysWithItsStepUntilTheStepIsDropped() {
    assertThat(history.driftSeenAtRest(STEPS)).as("none seen").isNaN();

    history.sawDriftOverNewestStep(0.001, 4e-6);
    assertThat(history.driftSeenAtRest(STEPS)).isEqualTo(0.001);
    assertThat(history.driftSeenVariance(STEPS)).isEqualTo(4e-6);
    assertThat(history.driftSeenAtRest(STEPS - 1)).isNaN();

    // The history is full: each step added drops the oldest, and the step numbers go down by one.
    add(STEPS + 1);
    assertThat(history.stepCount()).isEqualTo(STEPS + 1);
    assertThat(history.oldestTime()).isEqualTo(STEP_NANOS);
    assertThat(history.driftSeenAtRest(STEPS - 1)).isEqualTo(0.001);
    for (int k = STEPS + 2; k <= 2 * STEPS + 2; k++) {
      add(k);
    }
    for (int step = 0; step <= STEPS; step++) {
      assertThat(history.driftSeenAtRest(step)).as("step %d, its place used again", step).isNaN();
    }
    // Odometry alone is counted from the oldest step still kept.
    assertThat(history.varianceByOdometryAlone(1)).isCloseTo(0.0001 * (STEPS + 3), within(1e-12));
  }

  /**
   * One step of its own, every value it keeps telling it apart from the others: the same steps go
   * to a history that grows and to one with room for them all from the start.
   */
  private void addDistinct(OdometryHistory into, int k) {
    YawSource turnedBy = k % 3 == 0 ? YawSource.WHEELS : YawSource.GYRO;
    int step =
        into.add(k * STEP_NANOS, 0.001 * k, 0.002 * k, 1e-4 * k, 1e-5 * k, turnedBy, 0.01 * k * k);
    into.movedOverNewestStep(0.003 * k, -0.001 * k, 2e-5 * k);
    if (k % 4 == 0) {
      into.sawDriftOverNewestStep(1e-3 * k, 1e-6 * k);
    }
    into.reopenedTurnAtNewestStep(k % 2, 1e-7 * k);
    filterAt(k, -0.5 * k);
    into.store(step, filter);
  }

  /**
   * A history that starts with room for four steps and grows to sixteen keeps every step's values
   * where they were, in order, though its oldest step sat part way along its ring when it grew:
   * read through every accessor, it is the same as one that had room from the start.
   */
  @Test
  void aHistoryThatGrowsAcrossItsRingKeepsEveryStepInOrder() {
    OdometryHistory growing = new OdometryHistory(4, 16, filter.uncertaintyLength());
    OdometryHistory roomy = new OdometryHistory(32, filter.uncertaintyLength());
    for (OdometryHistory each : List.of(growing, roomy)) {
      each.start(0, YawSource.GYRO, 0);
      filterAt(0, 0);
      each.store(OdometryHistory.OLDEST_STEP, filter);
      for (int k = 1; k <= 3; k++) {
        addDistinct(each, k);
      }
      // Full at four: the two oldest go, so the oldest sits half way along the ring, and the next
      // two wrap around it before the history must grow.
      each.dropStepsNotNeededSince(2 * STEP_NANOS);
      for (int k = 4; k <= 14; k++) {
        addDistinct(each, k);
      }
    }

    assertThat(growing.stepCount()).isEqualTo(roomy.stepCount()).isEqualTo(13);
    for (int step = OdometryHistory.OLDEST_STEP; step < roomy.stepCount(); step++) {
      assertSameStep(growing, roomy, step);
    }
    long middle = (growing.oldestTime() + growing.newestTime()) / 2 + STEP_NANOS / 3;
    assertThat(growing.poseAt(middle, 0.3, 0.1, 2.0))
        .isEqualTo(roomy.poseAt(middle, 0.3, 0.1, 2.0));
    assertThat(growing.motionOver(4 * STEP_NANOS, 0.3))
        .isEqualTo(roomy.motionOver(4 * STEP_NANOS, 0.3));
    assertThat(growing.lastStepBefore(middle)).isEqualTo(roomy.lastStepBefore(middle));
  }

  /** One step read through every accessor, from two histories that should hold the same. */
  private void assertSameStep(OdometryHistory growing, OdometryHistory roomy, int step) {
    String at = "step " + step;
    assertThat(growing.endTime(step)).as(at).isEqualTo(roomy.endTime(step));
    assertThat(growing.travelForward(step)).as(at).isEqualTo(roomy.travelForward(step));
    assertThat(growing.travelLeft(step)).as(at).isEqualTo(roomy.travelLeft(step));
    assertThat(growing.wheelResidual(step)).as(at).isEqualTo(roomy.wheelResidual(step));
    assertThat(growing.addedPositionVariance(step))
        .as(at)
        .isEqualTo(roomy.addedPositionVariance(step));
    assertThat(growing.turnedBy(step)).as(at).isEqualTo(roomy.turnedBy(step));
    assertThat(growing.yawAfter(step)).as(at).isEqualTo(roomy.yawAfter(step));
    // Boxed, so a step with no drift seen, not a number in both, compares as the same.
    assertThat(Double.valueOf(growing.driftSeenAtRest(step)))
        .as(at)
        .isEqualTo(Double.valueOf(roomy.driftSeenAtRest(step)));
    assertThat(growing.driftSeenVariance(step)).as(at).isEqualTo(roomy.driftSeenVariance(step));
    for (int camera = 0; camera < 2; camera++) {
      assertThat(growing.turnReopenedBy(step, camera))
          .as(at)
          .isEqualTo(roomy.turnReopenedBy(step, camera));
    }
    assertThat(growing.varianceByOdometryAlone(step))
        .as(at)
        .isEqualTo(roomy.varianceByOdometryAlone(step));
    assertThat(growing.xByOdometryAlone(step, 0.5))
        .as(at)
        .isEqualTo(roomy.xByOdometryAlone(step, 0.5));
    assertThat(growing.yByOdometryAlone(step, 0.5))
        .as(at)
        .isEqualTo(roomy.yByOdometryAlone(step, 0.5));
    assertThat(growing.velocityX(step)).as(at).isEqualTo(roomy.velocityX(step));
    assertThat(growing.velocityY(step)).as(at).isEqualTo(roomy.velocityY(step));
    assertThat(growing.yawRate(step)).as(at).isEqualTo(roomy.yawRate(step));
    double[] grownUncertainty = new double[filter.uncertaintyLength()];
    double[] roomyUncertainty = new double[filter.uncertaintyLength()];
    growing.restore(step, filter);
    double grownX = filter.x();
    double grownY = filter.y();
    filter.copyUncertaintyTo(grownUncertainty, 0);
    roomy.restore(step, filter);
    filter.copyUncertaintyTo(roomyUncertainty, 0);
    assertThat(grownX).as(at).isEqualTo(filter.x());
    assertThat(grownY).as(at).isEqualTo(filter.y());
    assertThat(grownUncertainty).as(at).containsExactly(roomyUncertainty);
  }

  /** The motion over the last 30 ms: three steps of 0.03 m forward and 0.01 m left, turning. */
  @Test
  void theMotionOverAWindowTurnsEachStepsTravelByItsHeading() {
    OdometryHistory.Motion motion = history.motionOver(3 * STEP_NANOS, 0.3);

    double x = 0;
    double y = 0;
    for (int k = 5; k <= STEPS; k++) {
      double heading = YAW_AT_START + YAW_EACH_STEP * (k - 1) + 0.3;
      x += Math.cos(heading) * 0.03 - Math.sin(heading) * 0.01;
      y += Math.sin(heading) * 0.03 + Math.cos(heading) * 0.01;
    }
    assertThat(motion.fieldVelocity().getX()).isCloseTo(x / 0.03, within(1e-9));
    assertThat(motion.fieldVelocity().getY()).isCloseTo(y / 0.03, within(1e-9));
    assertThat(motion.yawRateRadiansPerSecond()).isCloseTo(YAW_EACH_STEP / 0.01, within(1e-9));
  }
}
