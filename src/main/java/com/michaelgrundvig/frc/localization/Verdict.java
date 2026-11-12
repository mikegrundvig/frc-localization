package com.michaelgrundvig.frc.localization;

import java.util.Locale;

/**
 * What became of a camera's image, or of one tag in it: accepted, or why not. Three places decide.
 * The robot may leave an image out or hold it back ({@link LocalizationService#leftOut}, {@link
 * LocalizationService#hold}), the {@link VisionPipeline} may throw one out before it becomes a
 * measurement, or leave some of its tags out, and the {@link PoseEstimator} weighs each measurement
 * against the estimate. Each verdict carries its words, so the page and the log say it without
 * anything reading a reason back out of a sentence.
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
   * Its own heading disagreed with one a person gave (a placement, or a driver's heading reset)
   * that no camera has backed yet: until one does, a camera's heading either backs it or counts
   * against it, and is never averaged with it. Its χ² passed the gate: the value is its χ², the
   * threshold the gate.
   */
  HEADING_UNBACKED(
      "its heading disagrees with an unbacked placement",
      "its heading disagrees with one a person gave, which no camera has backed yet"),

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

  /** The image saw no tag at all: nothing in view. */
  NO_TAGS("saw no tags", "saw no tags"),

  /**
   * The image's camera saw tags it couldn't solve because it has no lens calibration: Visor sends a
   * tag's corners and no pose without one. The pipeline adds which tags those were.
   */
  NO_LENS(
      "no lens calibration",
      "no lens calibration: Visor sends corners and no poses without one (docs/visor.md)"),

  /**
   * The image's camera, calibrated, saw tags whose poses couldn't be solved: seen edge-on, or too
   * small. The pipeline adds which tags those were.
   */
  UNSOLVED_TAGS("tags too skewed to solve", "tags seen too edge-on or too small to solve"),

  /**
   * A tag the field's layout doesn't have: as an image's verdict, every tag it saw was one, and the
   * pipeline adds which. A venue or a season's layout that is missing tags, or a misread number.
   */
  UNKNOWN_TAGS("tags not in this field's layout", "tags not in this field's layout"),

  /** A tag's pose from the camera wasn't a number. */
  TAG_NOT_A_NUMBER("a tag's pose isn't a number", "a tag's pose from the camera isn't a number"),

  /**
   * The robot was turning too fast for a sharp image: the value is how fast, in rad/s, the
   * threshold the fastest allowed.
   */
  TURNING("turning too fast", "turning too fast: %1$.1f rad/s > %2$.1f rad/s"),

  /**
   * A tag was too far to place the robot well, and was left out of its image: the value is its
   * distance, in meters, the threshold the farthest allowed. As an image's verdict, every tag was,
   * and the value is the nearest one's.
   */
  TOO_FAR("too far", "too far: %1$.1f m > %2$.1f m"),

  /**
   * A tag was nearer than any real view of one, and was left out of its image: the value is its
   * distance, in meters, the threshold the least allowed. As an image's verdict, every tag was, and
   * the value is the farthest one's.
   */
  TOO_CLOSE("too close", "too close: %1$.2f m < %2$.2f m"),

  /** The robot's speed or turn rate as the image was taken wasn't a number. */
  MOTION_NOT_A_NUMBER(
      "the robot's motion isn't a number",
      "the robot's speed or turn rate as the image was taken isn't a number"),

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

  /** Left out by the robot: the camera is turned off (fogged over, knocked loose, blinded). */
  TURNED_OFF("camera turned off", "camera turned off"),

  /** Left out by the robot: the camera handed over an image no newer than one it already had. */
  SEEN_BEFORE("the same image again", "the same image again"),

  /**
   * Left out by the robot: the image arrived before it was taken, or much later than its camera's
   * usual latency, so its capture time is wrong (the camera's clock and the robot's out of step).
   */
  OUT_OF_STEP("arrived out of step", "arrived out of step with its capture time"),

  /** Left out by the robot: the image arrived without a capture time to solve it at. */
  NO_CAPTURE_TIME("no capture time", "arrived without a capture time"),

  /** Left out by the robot: the camera's data for it couldn't be read. */
  UNREADABLE("unreadable data", "the camera's data for it couldn't be read"),

  /** Held back by the robot: its camera is being calibrated, from the others' pose. */
  CALIBRATING("withheld while calibrating this camera", "withheld while calibrating this camera"),

  /** Held back by the robot: autonomous just started, and the placed start is trusted. */
  HELD("held, autonomous just started", "held, autonomous just started"),

  /**
   * It was given to a localizer that does not report what it does with its measurements ({@link
   * Localizer#verdicts}): whether it moved the estimate is not known here.
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
