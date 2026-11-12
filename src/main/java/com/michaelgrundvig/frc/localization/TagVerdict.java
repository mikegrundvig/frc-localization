package com.michaelgrundvig.frc.localization;

import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.Objects;

/**
 * What became of one tag in a camera's image. A tag the measurement rested on has the verdict on
 * that measurement, with its numbers (the χ² and the gate, for the estimator's verdicts): the
 * estimator weighs an image's tags together. A tag the pipeline left out of its image has its own
 * verdict, with its own numbers: too far, too close, not in the field's layout, or without a pose
 * (no lens calibration, or seen too edge-on). An image refused whole, or held back, gives every tag
 * it saw its verdict. How far the measurement landed from the estimate is the image's, in {@link
 * FrameVerdict.Measured}.
 *
 * @param tag the tag's number
 * @param verdict what became of it: accepted, or why not
 * @param threshold the limit the verdict's value was held to: the gate a χ² was, the farthest a tag
 *     may be, the fastest the robot may turn; NaN where the verdict has none
 * @param value the verdict's number: the χ² for the estimator's verdicts; the tag's distance, the
 *     robot's turning, a tag's ambiguity or height for the pipeline's; NaN where it has none
 */
@OutputRecord
public record TagVerdict(
    int tag,
    Verdict verdict,
    @RecordBuilder.Initializer(value = "NONE", source = Verdict.class) double threshold,
    @RecordBuilder.Initializer(value = "NONE", source = Verdict.class) double value) {
  public TagVerdict {
    Objects.requireNonNull(verdict, "a tag's verdict");
  }

  /** Builds one by name; a number left out is NaN. */
  public static TagVerdictBuilder builder() {
    return TagVerdictBuilder.builder();
  }

  /** Whether it moved the estimate. */
  public boolean accepted() {
    return verdict == Verdict.ACCEPTED;
  }

  /** Why, in words with its numbers, as the log has them: "gate: 45.2 > 13.8", "too far: 6.3 m". */
  public String described() {
    return verdict.describe(value, threshold);
  }
}
