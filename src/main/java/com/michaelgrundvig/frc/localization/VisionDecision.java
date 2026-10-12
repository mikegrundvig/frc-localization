package com.michaelgrundvig.frc.localization;

/**
 * What the estimator did with a camera measurement, and why: every one is logged, accepted or not,
 * and the innovations of the accepted ones are the data that tunes each camera's trust.
 *
 * @param measurement the measurement
 * @param verdict whether it moved the estimate, or why not
 * @param innovationMeters how far it was from the estimate at its capture time; NaN if it was
 *     rejected before being compared
 * @param chiSquared its squared Mahalanobis distance from the estimate; NaN if not compared
 * @param threshold the gate its χ² was held to; NaN if not compared
 */
public record VisionDecision(
    VisionMeasurement measurement,
    Verdict verdict,
    double innovationMeters,
    double chiSquared,
    double threshold) {
  /** Whether it moved the estimate. */
  public boolean accepted() {
    return verdict == Verdict.ACCEPTED;
  }

  /** Why, in words, as the log has them: "accepted", "gate: 45.2 > 13.8", "off the field". */
  public String reason() {
    return verdict.describe(chiSquared, threshold);
  }
}
