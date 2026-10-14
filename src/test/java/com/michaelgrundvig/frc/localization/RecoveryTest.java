package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * What the rejected measurements agree on, fed by hand, under the default configuration. Each comes
 * from one camera and one tag, with a sign that the robot may have been moved unseen, so that only
 * their agreement is judged here; {@code RecoveryCasesTest} has which sources count.
 */
class RecoveryTest {
  private static final EstimatorConfig CONFIG = EstimatorConfig.DEFAULTS;
  private static final double AGREE_METERS = CONFIG.recoveryAgreementMeters();
  private static final long WINDOW_NANOS = Timestamps.fromSeconds(CONFIG.recoveryWindowSeconds());

  private static final MotionEvidence.Sign HIT = MotionEvidence.Sign.HIT;

  private final Recovery recovery = new Recovery(CONFIG);

  /** A rejection whose position says the estimate's is wrong, with no heading of its own. */
  private static Recovery.Rejected positionAt(long time, double x, double y) {
    return new Recovery.Rejected(time, "front", List.of(26), x, y, Double.NaN, false, true);
  }

  /** A rejection whose heading alone disagreed. */
  private static Recovery.Rejected headingOf(long time, double radians) {
    return new Recovery.Rejected(time, "front", List.of(26), 0, 0, radians, true, true);
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
    Recovery.Move move = recovery.positionCorrection(HIT).orElseThrow();
    assertThat(move.by().getX()).isCloseTo(1, within(1e-12));
    assertThat(move.by().getY()).isCloseTo(2, within(1e-12));
    assertThat(move.newestTaken()).isEqualTo(30);
  }

  @Test
  void rejectionsTooFarApartAgreeOnNothing() {
    recovery.rejected(positionAt(10, 1 - 1.6 * AGREE_METERS, 2));
    recovery.rejected(positionAt(20, 1 + 1.6 * AGREE_METERS, 2));
    recovery.rejected(positionAt(30, 1, 2));
    assertThat(recovery.positionCorrection(HIT)).isEmpty();

    Recovery alongY = new Recovery(CONFIG);
    alongY.rejected(positionAt(10, 1, 2 - 1.6 * AGREE_METERS));
    alongY.rejected(positionAt(20, 1, 2 + 1.6 * AGREE_METERS));
    alongY.rejected(positionAt(30, 1, 2));
    assertThat(alongY.positionCorrection(HIT)).isEmpty();
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
    recovery.rejected(new Recovery.Rejected(30, "front", List.of(26), 1, 2, 0.3, false, true));
    recovery.rejected(headingOf(40, 0.3));
    recovery.rejected(headingOf(50, 0.3));

    assertThat(recovery.positionCorrection(HIT)).isPresent();
    assertThat(recovery.headingCorrection(HIT).orElseThrow()).isCloseTo(0.3, within(1e-12));
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

  /**
   * Which sources count as independent: three rejections that agree on a move and on a turn, from
   * these cameras and tags, with this sign from the other sensors.
   *
   * @param cameras each rejection's camera, oldest first
   * @param tags each rejection's tag
   * @param sign what besides the cameras says the robot may have moved
   * @param recovers what recovers
   */
  record Sources(
      List<String> cameras, List<Integer> tags, MotionEvidence.Sign sign, Recovers recovers) {}

  /** What three rejections that agree on a move and a turn recover. */
  enum Recovers {
    NOTHING,
    THE_POSITION_ALONE,
    THE_HEADING_AND_THE_POSITION
  }

  static Stream<Sources> sources() {
    List<String> oneCamera = List.of("front", "front", "front");
    List<String> twoCameras = List.of("front", "back", "front");
    List<Integer> oneTag = List.of(26, 26, 26);
    List<Integer> twoTags = List.of(26, 27, 26);
    MotionEvidence.Sign none = MotionEvidence.Sign.NONE;
    return Stream.of(
        new Sources(oneCamera, oneTag, none, Recovers.NOTHING),
        new Sources(oneCamera, twoTags, none, Recovers.THE_POSITION_ALONE),
        new Sources(twoCameras, oneTag, none, Recovers.THE_HEADING_AND_THE_POSITION),
        new Sources(
            oneCamera, oneTag, MotionEvidence.Sign.HIT, Recovers.THE_HEADING_AND_THE_POSITION),
        new Sources(
            oneCamera, oneTag, MotionEvidence.Sign.SLID, Recovers.THE_HEADING_AND_THE_POSITION),
        new Sources(
            oneCamera, oneTag, MotionEvidence.Sign.LIFTED, Recovers.THE_HEADING_AND_THE_POSITION),
        new Sources(
            oneCamera, oneTag, MotionEvidence.Sign.DISABLED, Recovers.THE_HEADING_AND_THE_POSITION),
        new Sources(
            oneCamera, oneTag, MotionEvidence.Sign.PLACED, Recovers.THE_HEADING_AND_THE_POSITION));
  }

  /**
   * A position needs two cameras, two tags, or a sign; a heading needs two cameras or a sign, since
   * every tag a camera sees is turned by its mount alike.
   */
  @ParameterizedTest
  @MethodSource("sources")
  void agreementCountsOnlyFromIndependentSources(Sources row) {
    for (int i = 0; i < 3; i++) {
      recovery.rejected(
          new Recovery.Rejected(
              10L * (i + 1),
              row.cameras().get(i),
              List.of(row.tags().get(i)),
              1,
              2,
              0.3,
              false,
              true));
    }

    boolean moves = recovery.positionCorrection(row.sign()).isPresent();
    boolean turns = recovery.headingCorrection(row.sign()).isPresent();

    assertThat(moves).isEqualTo(row.recovers() != Recovers.NOTHING);
    assertThat(turns).isEqualTo(row.recovers() == Recovers.THE_HEADING_AND_THE_POSITION);
  }
}
