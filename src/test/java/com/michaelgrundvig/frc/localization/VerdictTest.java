package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;

/**
 * Every reason the estimator and the vision pipeline gave in words before {@link Verdict}, word for
 * word: the cameras' logs (their decisions, and the reasons beside refused tags) keep the same
 * strings, so old logs and new read alike.
 */
class VerdictTest {
  /** A verdict's numbers, and the words the code wrote by hand for them before. */
  private record Before(double value, double threshold, String words) {}

  /**
   * Each reason as the code wrote it, with the code's own format where it had numbers: the
   * estimator's first, then the pipeline's.
   */
  private static final Map<Verdict, Before> BEFORE = new EnumMap<>(Verdict.class);

  static {
    BEFORE.put(Verdict.ACCEPTED, new Before(0.0189, 13.8, "accepted"));
    BEFORE.put(
        Verdict.GATED,
        new Before(
            45.24, 13.8155, String.format(Locale.ROOT, "gate: %.1f > %.1f", 45.24, 13.8155)));
    BEFORE.put(Verdict.OFF_FIELD, new Before(Double.NaN, Double.NaN, "off the field"));
    // Added on purpose, 2026-10-11: before it, such a measurement left the estimator with no reason
    // at all.
    BEFORE.put(
        Verdict.NOT_COMPARABLE,
        new Before(
            Double.NaN,
            Double.NaN,
            "can't be compared with the estimate: its uncertainty leaves no spread to measure in"));
    BEFORE.put(
        Verdict.BEFORE_ODOMETRY, new Before(Double.NaN, Double.NaN, "before odometry started"));
    BEFORE.put(
        Verdict.BEFORE_RESET,
        new Before(Double.NaN, Double.NaN, "before the robot was last placed or recovered"));
    BEFORE.put(
        Verdict.STALE, new Before(Double.NaN, Double.NaN, "too late: older than the history kept"));
    BEFORE.put(
        Verdict.FROM_THE_FUTURE,
        new Before(Double.NaN, Double.NaN, "from the future: after the newest odometry"));
    // The pipeline adds which tags the task trusts: "no tags this task trusts (Hub: 10)".
    BEFORE.put(
        Verdict.UNTRUSTED_TAGS, new Before(Double.NaN, Double.NaN, "no tags this task trusts"));
    BEFORE.put(
        Verdict.UNKNOWN_TAGS,
        new Before(Double.NaN, Double.NaN, "no tags from this field's layout"));
    BEFORE.put(
        Verdict.TAG_NOT_A_NUMBER,
        new Before(Double.NaN, Double.NaN, "a tag's pose from the camera isn't a number"));
    BEFORE.put(
        Verdict.TURNING,
        new Before(6.04, 4.0, String.format(Locale.ROOT, "turning too fast: %.1f rad/s", 6.04)));
    BEFORE.put(
        Verdict.TOO_FAR,
        new Before(5.06, 4.5, String.format(Locale.ROOT, "too far: %.1f m", 5.06)));
    BEFORE.put(
        Verdict.AMBIGUOUS,
        new Before(
            0.349,
            0.2,
            String.format(Locale.ROOT, "ambiguous single tag: %.2f > %.2f", 0.349, 0.2)));
    BEFORE.put(
        Verdict.TAG_HEIGHT,
        new Before(
            -0.314, 0.25, String.format(Locale.ROOT, "a tag's height is off by %.2f m", -0.314)));
    BEFORE.put(
        Verdict.SOLVE_NOT_A_NUMBER, new Before(Double.NaN, Double.NaN, "the solve isn't a number"));
    // The robot's own, which the cameras' log puts before the pipeline's outcome: "held, autonomous
    // just started: heading-constrained: 2 tags at 3.1 m".
    BEFORE.put(
        Verdict.CALIBRATING,
        new Before(Double.NaN, Double.NaN, "withheld while calibrating this camera"));
    BEFORE.put(Verdict.HELD, new Before(Double.NaN, Double.NaN, "held, autonomous just started"));
    // Added with the localizer seam, on purpose: a localizer that doesn't report its decisions.
    BEFORE.put(
        Verdict.GIVEN,
        new Before(
            Double.NaN,
            Double.NaN,
            "given to the localizer, which doesn't report what became of it"));
    BEFORE.put(
        Verdict.OFF_THE_FLOOR,
        new Before(
            0.404,
            0.25,
            String.format(Locale.ROOT, "the solve puts the robot %.2f m off the floor", 0.404)));
  }

  @Test
  void everyReasonTheCodeGaveIsAVerdictInTheSameWords() {
    for (Map.Entry<Verdict, Before> reason : BEFORE.entrySet()) {
      Before before = reason.getValue();

      assertThat(reason.getKey().describe(before.value(), before.threshold()))
          .as(reason.getKey().name())
          .isEqualTo(before.words());
    }
  }

  /** A verdict nobody gave before has no log to match: it'd be a new reason, added on purpose. */
  @Test
  void everyVerdictIsOneOfThoseReasons() {
    assertThat(BEFORE.keySet()).containsExactlyInAnyOrder(Verdict.values());
  }

  @Test
  void eachVerdictHasWordsOfItsOwn() {
    assertThat(Arrays.stream(Verdict.values()).map(Verdict::words))
        .doesNotHaveDuplicates()
        .allMatch(words -> !words.isBlank());
  }

  /** A gate's χ² that isn't a number fails it, and says so as it always did. */
  @Test
  void aChiSquaredThatIsntANumberReadsAsOne() {
    assertThat(Verdict.GATED.describe(Double.NaN, 13.8)).isEqualTo("gate: NaN > 13.8");
  }

  /** The records that hold a verdict say it the same way, and accept only an acceptance. */
  @Test
  void decisionsSayTheirVerdictWithTheirNumbers() {
    VisionMeasurement measurement =
        new VisionMeasurement(
            1,
            Pose2d.ZERO,
            PoseCovariance.fromStdDevs(0.1, 0.1, 0.1),
            VisionMeasurement.Lever.NONE,
            "front",
            List.of(7),
            VisionMeasurement.Solver.SINGLE_TAG);
    VisionDecision gated = new VisionDecision(measurement, Verdict.GATED, 0.8, 45.2, 13.8);
    VisionDecision accepted = new VisionDecision(measurement, Verdict.ACCEPTED, 0.01, 0.2, 13.8);

    assertThat(gated.accepted()).isFalse();
    assertThat(gated.reason()).isEqualTo("gate: 45.2 > 13.8");
    assertThat(accepted.accepted()).isTrue();
    assertThat(accepted.reason()).isEqualTo("accepted");
  }
}
