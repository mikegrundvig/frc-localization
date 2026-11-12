package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.localization.EstimatorConfig.GyroErrors;
import com.michaelgrundvig.frc.localization.LocalizerBench.Option;
import org.junit.jupiter.api.Nested;

/**
 * The default localizer keeps the whole contract, and reports every option: with its defaults, and
 * with each other way it can correct the heading and treat the gyro's errors, since each changes
 * how its measurements move the estimate.
 */
class DefaultLocalizerConformanceTest extends LocalizerConformance {
  DefaultLocalizerConformanceTest() {
    super(bench(HeadingCorrection.EVERY_FIX, EstimatorConfig.defaults().gyroErrors()));
  }

  /** A bench for the default localizer, its defaults but for these two choices. */
  static LocalizerBench bench(HeadingCorrection heading, GyroErrors gyro) {
    EstimatorConfig config =
        EstimatorConfigBuilder.builder(EstimatorConfig.defaults()).gyroErrors(gyro).build();
    return LocalizerBench.of(
        "the default localizer, " + heading + " and " + gyro,
        DefaultLocalizer.with(config, SlipCheck.Settings.defaults(), heading),
        Option.values());
  }

  @Nested
  class EveryFixConsideredGyro extends LocalizerConformance {
    EveryFixConsideredGyro() {
      super(bench(HeadingCorrection.EVERY_FIX, GyroErrors.CONSIDERED));
    }
  }

  @Nested
  class HeadingMeasurementsEstimatedGyro extends LocalizerConformance {
    HeadingMeasurementsEstimatedGyro() {
      super(bench(HeadingCorrection.HEADING_MEASUREMENTS, GyroErrors.ESTIMATED));
    }
  }

  @Nested
  class HeadingMeasurementsConsideredGyro extends LocalizerConformance {
    HeadingMeasurementsConsideredGyro() {
      super(bench(HeadingCorrection.HEADING_MEASUREMENTS, GyroErrors.CONSIDERED));
    }
  }

  @Nested
  class HeadingMeasurementsAndMountDoubtEstimatedGyro extends LocalizerConformance {
    HeadingMeasurementsAndMountDoubtEstimatedGyro() {
      super(bench(HeadingCorrection.HEADING_MEASUREMENTS_AND_MOUNT_DOUBT, GyroErrors.ESTIMATED));
    }
  }

  @Nested
  class HeadingMeasurementsAndMountDoubtConsideredGyro extends LocalizerConformance {
    HeadingMeasurementsAndMountDoubtConsideredGyro() {
      super(bench(HeadingCorrection.HEADING_MEASUREMENTS_AND_MOUNT_DOUBT, GyroErrors.CONSIDERED));
    }
  }
}
