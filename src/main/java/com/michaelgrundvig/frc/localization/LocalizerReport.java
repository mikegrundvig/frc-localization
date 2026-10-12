package com.michaelgrundvig.frc.localization;

import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;

/**
 * What a {@link Localizer} says of the robot's pose at one moment: the pose, and each option the
 * localizer reports about it. An option it does not report is empty, and whoever reads this says
 * "not reported" in its place. Immutable, so the loop can hand it to a view on another thread
 * whole.
 *
 * @param timestampNanos the time of the newest wheel reading the pose includes
 * @param pose the robot's pose on the field
 * @param covariance how sure the pose is, if the localizer says
 * @param mode what the pose rests on, if the localizer says
 * @param health the numbers behind the mode, if the localizer says
 */
public record LocalizerReport(
    long timestampNanos,
    Pose2d pose,
    Optional<PoseCovariance> covariance,
    Optional<LocalizationEstimate.Mode> mode,
    Optional<LocalizationEstimate.Health> health) {
  public LocalizerReport {
    Objects.requireNonNull(pose, "the pose");
    Objects.requireNonNull(covariance, "the covariance, or empty");
    Objects.requireNonNull(mode, "the mode, or empty");
    Objects.requireNonNull(health, "the health, or empty");
  }

  /** What a localizer says now. */
  public static LocalizerReport of(Localizer localizer) {
    return new LocalizerReport(
        localizer.timestampNanos(),
        localizer.pose(),
        localizer.covariance(),
        localizer.mode(),
        localizer.health());
  }
}
