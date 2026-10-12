package com.michaelgrundvig.frc.localization;

import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.Objects;

/**
 * What became of one tag in a camera's image: the verdict on the measurement it went into, or on
 * the image, if it was refused before it became one. An image's tags are weighed together today, so
 * every tag of a frame carries the frame's verdict; each is its own record so a check of each tag
 * on its own has a place to say what it found.
 *
 * @param tag the tag's number
 * @param verdict whether the measurement moved the estimate, or why not
 * @param innovationMeters how far the measurement was from the estimate at its capture time; NaN if
 *     it was refused before being compared
 * @param chiSquared its squared Mahalanobis distance from the estimate; NaN if not compared
 * @param threshold the limit the verdict's value was held to: the gate a χ² was, the farthest the
 *     tags may be, the fastest the robot may turn; NaN where the verdict has none
 * @param value the verdict's number: the χ² for the estimator's verdicts; the tags' distance, the
 *     robot's turning, a tag's ambiguity or height for the pipeline's; NaN where it has none
 */
@OutputRecord
public record TagVerdict(
    int tag,
    Verdict verdict,
    @RecordBuilder.Initializer(value = "NONE", source = Verdict.class) double innovationMeters,
    @RecordBuilder.Initializer(value = "NONE", source = Verdict.class) double chiSquared,
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
  public String reason() {
    return verdict.describe(value, threshold);
  }
}
