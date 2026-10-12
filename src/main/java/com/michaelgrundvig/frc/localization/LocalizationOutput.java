package com.michaelgrundvig.frc.localization;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

/**
 * What one {@link LocalizationService#update} made of the loop: the pose with what the localizer
 * reports about it, how far each source is trusted, and what became of each camera image.
 * Immutable, so the loop can hand it to a view on another thread whole.
 *
 * @param report the pose, with its uncertainty, mode, and health where the localizer reports them
 * @param trust how far each source is trusted; empty when the localizer does not report it
 * @param frames what became of each camera image given since the last update: those refused or held
 *     back before they became measurements first, in the order they came; then the measurements. A
 *     localizer that reports its decisions has them in the order it decided them (for the default
 *     localizer, those it couldn't apply, then the rest by capture time). One that does not has
 *     them in the order they were given, each with {@link Verdict#GIVEN}
 */
@OutputRecord
public record LocalizationOutput(
    LocalizerReport report, Optional<Trust> trust, List<FrameVerdict> frames) {
  public LocalizationOutput {
    Objects.requireNonNull(report, "the report");
    Objects.requireNonNull(trust, "the trust, or empty");
    frames = List.copyOf(frames);
  }

  /** Builds one by name: no frames, unless said. */
  public static LocalizationOutputBuilder builder() {
    return LocalizationOutputBuilder.builder();
  }
}
