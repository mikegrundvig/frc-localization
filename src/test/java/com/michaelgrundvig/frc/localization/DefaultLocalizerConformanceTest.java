package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.localization.LocalizerBench.Option;

/** The default localizer keeps the whole contract, and reports every option. */
class DefaultLocalizerConformanceTest extends LocalizerConformance {
  DefaultLocalizerConformanceTest() {
    super(
        LocalizerBench.of(
            "the default localizer",
            DefaultLocalizer.with(EstimatorConfig.DEFAULTS),
            Option.values()));
  }
}
