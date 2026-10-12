package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;

/** What the rejected measurements agree on, fed by hand, under the default configuration. */
class RecoveryTest {
  private static final EstimatorConfig CONFIG = EstimatorConfig.DEFAULTS;
  private static final double AGREE_METERS = CONFIG.recoveryAgreementMeters();
  private static final long WINDOW_NANOS = Timestamps.fromSeconds(CONFIG.recoveryWindowSeconds());

  private final Recovery recovery = new Recovery(CONFIG);

  /** A rejection whose position says the estimate's is wrong, with no heading of its own. */
  private static Recovery.Rejected positionAt(long time, double x, double y) {
    return new Recovery.Rejected(time, x, y, Double.NaN, false, true);
  }

  /** A rejection whose heading alone disagreed. */
  private static Recovery.Rejected headingOf(long time, double radians) {
    return new Recovery.Rejected(time, 0, 0, radians, true, true);
  }

  @Test
  void theLatestRejectionsThatAgreeMoveTheEstimateToTheirMean() {
    assertThat(CONFIG.recoveryMeasurements()).isEqualTo(3);
    assertThat(recovery.hasEnough()).isFalse();
    // An older one, far from the rest, was taken while the robot was still being moved.
    recovery.rejected(positionAt(5, -3, 7));
    recovery.rejected(positionAt(10, 1 - 0.9 * AGREE_METERS, 2));
    recovery.rejected(positionAt(30, 1 + 0.9 * AGREE_METERS, 2.1));
    recovery.rejected(positionAt(20, 1, 1.9));

    assertThat(recovery.hasEnough()).isTrue();
    Recovery.Move move = recovery.positionCorrection().orElseThrow();
    assertThat(move.by().getX()).isCloseTo(1, within(1e-12));
    assertThat(move.by().getY()).isCloseTo(2, within(1e-12));
    assertThat(move.newestTaken()).isEqualTo(30);
  }

  @Test
  void rejectionsTooFarApartAgreeOnNothing() {
    recovery.rejected(positionAt(10, 1 - 1.6 * AGREE_METERS, 2));
    recovery.rejected(positionAt(20, 1 + 1.6 * AGREE_METERS, 2));
    recovery.rejected(positionAt(30, 1, 2));
    assertThat(recovery.positionCorrection()).isEmpty();

    Recovery alongY = new Recovery(CONFIG);
    alongY.rejected(positionAt(10, 1, 2 - 1.6 * AGREE_METERS));
    alongY.rejected(positionAt(20, 1, 2 + 1.6 * AGREE_METERS));
    alongY.rejected(positionAt(30, 1, 2));
    assertThat(alongY.positionCorrection()).isEmpty();
  }

  /**
   * The headings are judged apart from the positions: two rejections with no heading, one with a
   * heading and a wrong position, and two whose headings alone disagreed. The latest three that say
   * the position is wrong are not the latest three with headings.
   */
  @Test
  void theHeadingsThatAgreeAreJudgedOnTheirOwn() {
    recovery.rejected(positionAt(10, 1, 2));
    recovery.rejected(positionAt(20, 1, 2));
    recovery.rejected(new Recovery.Rejected(30, 1, 2, 0.3, false, true));
    recovery.rejected(headingOf(40, 0.3));
    recovery.rejected(headingOf(50, 0.3));

    assertThat(recovery.positionCorrection()).isPresent();
    assertThat(recovery.headingCorrection().orElseThrow()).isCloseTo(0.3, within(1e-12));
  }

  @Test
  void rejectionsOlderThanTheWindowAreForgotten() {
    recovery.rejected(positionAt(0, 1, 2));
    recovery.rejected(positionAt(0, 1, 2));
    recovery.rejected(positionAt(0, 1, 2));

    recovery.forgetOlderThanTheWindow(WINDOW_NANOS);
    assertThat(recovery.hasEnough()).as("at the window's edge").isTrue();
    recovery.forgetOlderThanTheWindow(WINDOW_NANOS + 1);
    assertThat(recovery.hasEnough()).as("past it").isFalse();
  }
}
