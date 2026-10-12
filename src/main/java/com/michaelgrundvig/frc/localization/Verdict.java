package com.michaelgrundvig.frc.localization;

import java.util.Locale;

/**
 * What became of a camera's image: accepted, or why not. Three places decide. The robot may hold an
 * image back ({@link LocalizationService#hold}), the {@link VisionPipeline} may throw one out
 * before it becomes a measurement, and the {@link PoseEstimator} weighs each measurement against
 * the estimate. Each verdict carries its words, so the page and the log say it without anything
 * reading a reason back out of a sentence.
 *
 * <p>A verdict's numbers (how far past the gate, how far away, how fast it turned) travel beside it
 * in the record that holds it; {@link #describe} puts them into its words, as the log has always
 * had them.
 */
public enum Verdict {
  /** It moved the estimate. */
  ACCEPTED("accepted", "accepted"),

  /** Its χ² against the estimate was past the gate: the value is its χ², the threshold the gate. */
  GATED("outside the gate", "gate: %1$.1f > %2$.1f"),

  /**
   * It could not be compared with the estimate at all: its uncertainty and the estimate's together
   * left no spread to measure a distance in, so it has no χ² and was never held to the gate.
   */
  NOT_COMPARABLE(
      "can't be compared with the estimate",
      "can't be compared with the estimate: its uncertainty leaves no spread to measure in"),

  /** It put the robot off the field. */
  OFF_FIELD("off the field", "off the field"),

  /** It came before the first odometry step: there was nothing to weigh it against. */
  BEFORE_ODOMETRY("before odometry started", "before odometry started"),

  /** It was taken before the robot was last placed, or the estimate last recovered. */
  BEFORE_RESET(
      "before the robot was placed or recovered", "before the robot was last placed or recovered"),

  /** It was older than the history the estimator keeps to apply it in. */
  STALE("too late", "too late: older than the history kept"),

  /** It was stamped after the newest odometry, by more than the estimator allows. */
  FROM_THE_FUTURE("from the future", "from the future: after the newest odometry"),

  /**
   * The image saw no tag the current task trusts ({@link TagAllowlist}): the pipeline adds which
   * tags those are.
   */
  UNTRUSTED_TAGS("no tags this task trusts", "no tags this task trusts"),

  /** The image saw no tag the field's layout has. */
  UNKNOWN_TAGS("no tags from this field's layout", "no tags from this field's layout"),

  /** A tag's pose from the camera wasn't a number. */
  TAG_NOT_A_NUMBER("a tag's pose isn't a number", "a tag's pose from the camera isn't a number"),

  /** The robot was turning too fast for a sharp image: the value is how fast, in rad/s. */
  TURNING("turning too fast", "turning too fast: %1$.1f rad/s"),

  /** The tags were too far: the value is their average distance, in meters. */
  TOO_FAR("too far", "too far: %1$.1f m"),

  /**
   * One tag alone, and too ambiguous to tell its two poses apart: the value is its ambiguity, the
   * threshold the most allowed.
   */
  AMBIGUOUS("ambiguous single tag", "ambiguous single tag: %1$.2f > %2$.2f"),

  /** A tag's known height disagreed with the solve: the value is by how much, in meters. */
  TAG_HEIGHT("a tag's height is off", "a tag's height is off by %1$.2f m"),

  /** The solve came out not a number. */
  SOLVE_NOT_A_NUMBER("the solve isn't a number", "the solve isn't a number"),

  /** The solve put the robot off the floor: the value is how far, in meters. */
  OFF_THE_FLOOR("off the floor", "the solve puts the robot %1$.2f m off the floor"),

  /** Held back by the robot: its camera is being calibrated, from the others' pose. */
  CALIBRATING("withheld while calibrating this camera", "withheld while calibrating this camera"),

  /** Held back by the robot: autonomous just started, and the placed start is trusted. */
  HELD("held, autonomous just started", "held, autonomous just started"),

  /**
   * It was given to a localizer that does not report what it does with its measurements ({@link
   * Localizer#decisions}): whether it moved the estimate is not known here.
   */
  GIVEN("given to the localizer", "given to the localizer, which doesn't report what became of it");

  /**
   * A number a verdict doesn't have, or a measurement that wasn't compared: what an output record's
   * builder leaves such a number as.
   */
  public static final double NONE = Double.NaN;

  private final String words;
  private final String pattern;

  Verdict(String words, String pattern) {
    this.words = words;
    this.pattern = pattern;
  }

  /** In a few words, without its numbers: "too far". */
  public String words() {
    return words;
  }

  /**
   * In words, with its numbers, as the log has them: "gate: 45.2 > 13.8", "too far: 5.1 m". A
   * verdict without numbers ignores them.
   *
   * @param value its number: a χ², a distance, a turn rate, an ambiguity, a height
   * @param threshold the limit the value was held to, where the words say it
   */
  public String describe(double value, double threshold) {
    return String.format(Locale.ROOT, pattern, value, threshold);
  }
}
