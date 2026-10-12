package com.michaelgrundvig.frc.localization;

import io.soabase.recordbuilder.core.RecordBuilder;

/**
 * How far localization trusts one camera, beyond its configuration, as of an update.
 *
 * @param mountDoubtMeters how far the camera's measurements may be off from where its mount moved
 *     since it was calibrated ({@link MountCheck#doubtMeters}), as the robot last said: every image
 *     it takes is widened by this. Zero for a camera that sits where its configuration says
 * @param mountDistanceMeters how far its tags were, on average, when that was judged; NaN until
 *     then
 * @param keptApart whether the estimator keeps this camera's repeated errors (its turn and shift on
 *     its mount) apart from the others'; a camera beyond the estimator's places ({@value
 *     ConsideredErrors#MAX_CAMERAS}) counts each image's errors as new, so its images, averaged,
 *     grow surer than they should
 */
@OutputRecord
public record CameraTrust(
    double mountDoubtMeters,
    @RecordBuilder.Initializer(value = "NONE", source = Verdict.class) double mountDistanceMeters,
    @RecordBuilder.Initializer("KEPT_APART") boolean keptApart) {
  /** Every camera is kept apart, up to the estimator's places: what a builder leaves it as. */
  public static final boolean KEPT_APART = true;

  /** Builds one by name: no doubt, judged at no distance, and kept apart, unless said. */
  public static CameraTrustBuilder builder() {
    return CameraTrustBuilder.builder();
  }
}
