package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.Locale;
import java.util.Optional;

/**
 * Whether an estimate is good enough to act on where precision matters: scoring, aiming, a final
 * approach. A precise-looking pose from dead reckoning, or one the cameras just disagreed with, is
 * not one to shoot from. A top team gated its scoring on vision within 0.5 s and agreeing within
 * 0.2 m; Top teams would not shoot while tilted.
 *
 * <p>Commands ask {@link #whyNot} before committing, and show the reason to the drive team when the
 * answer is no.
 *
 * @param maxVisionAgeSeconds the newest accepted camera measurement may be no older than this
 * @param maxDisagreementMeters and the measurements accepted lately may have been no farther than
 *     this from the estimate they corrected
 * @param maxPositionStdDevMeters the position's uncertainty, along its worst direction, may be no
 *     more than this
 * @param maxTiltDegrees the robot may be tilted no more than this
 */
@ConfigRecord
public record LocalizationGate(
    double maxVisionAgeSeconds,
    double maxDisagreementMeters,
    double maxPositionStdDevMeters,
    double maxTiltDegrees) {
  /** For scoring: a top team's thresholds, and level. */
  public static final LocalizationGate SCORING =
      builder()
          .maxVisionAgeSeconds(0.5)
          .maxDisagreementMeters(0.2)
          .maxPositionStdDevMeters(0.15)
          .maxTiltDegrees(5)
          .build();

  public LocalizationGate {
    Checks.on(LocalizationGate.class)
        .field("maxVisionAgeSeconds", maxVisionAgeSeconds, positive())
        .field("maxDisagreementMeters", maxDisagreementMeters, positive())
        .field("maxPositionStdDevMeters", maxPositionStdDevMeters, positive())
        .field("maxTiltDegrees", maxTiltDegrees, positive())
        .orThrow();
  }

  /** Starts building a gate, one named limit at a time. */
  public static LocalizationGateBuilder.MaxVisionAgeSecondsStage builder() {
    return LocalizationGateBuilder.builder();
  }

  /**
   * Why the estimate isn't good enough, for the drive team to read. Empty when it is good enough.
   *
   * @return the first reason found, the most basic first
   */
  public Optional<String> whyNot(LocalizationEstimate estimate) {
    return whyNot(estimate.mode(), estimate.health(), estimate.covariance());
  }

  /**
   * Why a pose isn't good enough, from what a localizer reports of it ({@link Localizer#mode},
   * {@link Localizer#health}, {@link Localizer#covariance}). Empty when it is good enough.
   *
   * @return the first reason found, the most basic first
   */
  public Optional<String> whyNot(
      LocalizationEstimate.Mode mode,
      LocalizationEstimate.Health health,
      PoseCovariance covariance) {
    Optional<String> restsOnTooLittle = whyItsModeIsNotEnough(mode);
    if (restsOnTooLittle.isPresent()) {
      return restsOnTooLittle;
    }
    // Each limit is written so that a value that isn't a number is past it.
    boolean visionIsRecent = health.visionAgeSeconds() <= maxVisionAgeSeconds;
    if (!visionIsRecent) {
      return Optional.of(
          String.format(
              Locale.ROOT, "the last camera measurement is %.1f s old", health.visionAgeSeconds()));
    }
    double disagreement = health.recentDisagreementMeters();
    boolean camerasAgree = disagreement <= maxDisagreementMeters;
    if (!camerasAgree) {
      if (Double.isNaN(disagreement)) {
        // No measurement was accepted lately, so there is nothing to agree or disagree.
        return Optional.of("no camera measurement lately");
      }
      return Optional.of(
          String.format(Locale.ROOT, "the cameras and odometry disagree by %.2f m", disagreement));
    }
    double spread = covariance.largestPositionStdDev();
    boolean positionIsSure = spread <= maxPositionStdDevMeters;
    if (!positionIsSure) {
      return Optional.of(String.format(Locale.ROOT, "the position is uncertain by %.2f m", spread));
    }
    double tiltDegrees = Math.toDegrees(health.tiltRadians());
    boolean level = tiltDegrees <= maxTiltDegrees;
    if (!level) {
      return Optional.of(
          String.format(Locale.ROOT, "the robot is tilted %.0f degrees", tiltDegrees));
    }
    if (health.collisionActive()) {
      return Optional.of("the robot was just hit");
    }
    return Optional.empty();
  }

  /** Why an estimate in a mode rests on too little: every mode but tracking does. */
  private static Optional<String> whyItsModeIsNotEnough(LocalizationEstimate.Mode mode) {
    return switch (mode) {
      case TRACKING -> Optional.empty();
      case INITIALIZING -> Optional.of("not placed or seen yet");
      case DEAD_RECKONING -> Optional.of("no camera measurement lately");
      case LOST -> Optional.of("the cameras disagree with the estimate");
      case IMU_FAULT -> Optional.of("the gyro is lost");
    };
  }

  /** Whether the estimate is good enough. */
  public boolean allows(LocalizationEstimate estimate) {
    return whyNot(estimate).isEmpty();
  }
}
