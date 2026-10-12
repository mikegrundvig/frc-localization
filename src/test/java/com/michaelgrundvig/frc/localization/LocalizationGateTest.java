package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.localization.LocalizationEstimate.Health;
import com.michaelgrundvig.frc.localization.LocalizationEstimate.Mode;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/** Whether an estimate is good enough to score from, and why not. */
class LocalizationGateTest {
  private static final LocalizationGate GATE = LocalizationGate.SCORING;

  /** A good estimate: tracking, a fresh measurement that agreed within 3 cm, level, not hit. */
  private static LocalizationEstimate estimate(
      Mode mode,
      double visionAgeSeconds,
      double disagreementMeters,
      double stdDevMeters,
      double tiltDegrees,
      boolean collision) {
    return new LocalizationEstimate(
        0,
        Pose2d.ZERO,
        PoseCovariance.fromStdDevs(stdDevMeters, stdDevMeters, 0.01),
        Translation2d.ZERO,
        0,
        mode,
        new Health(
            visionAgeSeconds,
            100,
            2,
            0.03,
            disagreementMeters,
            0.001,
            0.01,
            Math.toRadians(tiltDegrees),
            collision,
            0,
            true));
  }

  private static LocalizationEstimate good() {
    return estimate(Mode.TRACKING, 0.05, 0.03, 0.04, 1, false);
  }

  @Test
  void aTrackingEstimateThatAgreesIsGoodToScoreFrom() {
    assertThat(GATE.whyNot(good())).isEmpty();
    assertThat(GATE.allows(good())).isTrue();
  }

  @Test
  void eachReasonSaysWhatIsWrong() {
    assertThat(GATE.whyNot(estimate(Mode.INITIALIZING, 0.05, 0.03, 0.04, 1, false)))
        .contains("not placed or seen yet");
    assertThat(GATE.whyNot(estimate(Mode.DEAD_RECKONING, 0.05, 0.03, 0.04, 1, false)))
        .contains("no camera measurement lately");
    assertThat(GATE.whyNot(estimate(Mode.LOST, 0.05, 0.03, 0.04, 1, false)))
        .contains("the cameras disagree with the estimate");
    assertThat(GATE.whyNot(estimate(Mode.IMU_FAULT, 0.05, 0.03, 0.04, 1, false)))
        .contains("the gyro is lost");
    assertThat(GATE.whyNot(estimate(Mode.TRACKING, 0.8, 0.03, 0.04, 1, false)))
        .contains("the last camera measurement is 0.8 s old");
    assertThat(GATE.whyNot(estimate(Mode.TRACKING, 0.05, 0.3, 0.04, 1, false)))
        .contains("the cameras and odometry disagree by 0.30 m");
    assertThat(GATE.whyNot(estimate(Mode.TRACKING, 0.05, Double.NaN, 0.04, 1, false)))
        .contains("no camera measurement lately");
    assertThat(GATE.whyNot(estimate(Mode.TRACKING, 0.05, 0.03, 0.2, 1, false)))
        .contains("the position is uncertain by 0.20 m");
    assertThat(GATE.whyNot(estimate(Mode.TRACKING, 0.05, 0.03, 0.04, 8, false)))
        .contains("the robot is tilted 8 degrees");
    assertThat(GATE.whyNot(estimate(Mode.TRACKING, 0.05, 0.03, 0.04, 1, true)))
        .contains("the robot was just hit");
  }

  @Test
  void theMostBasicReasonComesFirst() {
    assertThat(GATE.whyNot(estimate(Mode.DEAD_RECKONING, 3, Double.NaN, 0.5, 20, true)))
        .contains("no camera measurement lately");
  }

  @Test
  void limitsMustBePositive() {
    assertThatThrownBy(() -> LocalizationGateBuilder.builder(GATE).maxDisagreementMeters(0).build())
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(
            () ->
                LocalizationGateBuilder.builder(GATE)
                    .maxVisionAgeSeconds(Double.POSITIVE_INFINITY)
                    .build())
        .isInstanceOf(IllegalArgumentException.class);
  }
}
