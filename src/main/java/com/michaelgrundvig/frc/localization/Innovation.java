package com.michaelgrundvig.frc.localization;

/**
 * How far a camera's measurement is from the estimate, and how surprising that is.
 *
 * @param xMeters the position's innovation along the field's x: measured less predicted
 * @param yMeters and along y
 * @param withHeading whether the measurement has a heading of its own
 * @param headingRadians the heading's innovation; NaN for a measurement without a heading
 * @param chiSquared the whole innovation's squared distance from zero, measured in its own
 *     uncertainty (its squared Mahalanobis distance): what the estimator's gate tests
 * @param positionChiSquared the same for the position alone
 * @param stamp which of the filter's innovations this is. {@link PoseFilter#correct(Innovation)}
 *     applies only the latest, because the filter holds the working of only one at a time
 */
record Innovation(
    double xMeters,
    double yMeters,
    boolean withHeading,
    double headingRadians,
    double chiSquared,
    double positionChiSquared,
    long stamp) {
  double distanceMeters() {
    return Math.hypot(xMeters, yMeters);
  }
}
