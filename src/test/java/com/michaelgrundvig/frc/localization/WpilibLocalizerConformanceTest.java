package com.michaelgrundvig.frc.localization;

/** WPILib's estimator keeps the required contract, and reports none of the options. */
class WpilibLocalizerConformanceTest extends LocalizerConformance {
  WpilibLocalizerConformanceTest() {
    super(LocalizerBench.of("WPILib's pose estimator", WpilibLocalizer::new));
  }
}
