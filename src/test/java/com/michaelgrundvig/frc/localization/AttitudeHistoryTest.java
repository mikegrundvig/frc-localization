package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;

/** The robot's attitude when an image was taken, from the readings around it. */
class AttitudeHistoryTest {
  private static final long MS = 1_000_000L;

  @Test
  void beforeAnyReadingTheRobotIsLevel() {
    assertThat(new AttitudeHistory(5).at(0)).isEqualTo(AttitudeHistory.Attitude.LEVEL);
  }

  @Test
  void betweenReadingsItInterpolates() {
    AttitudeHistory history = new AttitudeHistory(5);
    history.add(100 * MS, 0.0, 0.2);
    history.add(120 * MS, 0.1, 0.0);

    AttitudeHistory.Attitude between = history.at(115 * MS);

    assertThat(between.pitchRadians()).isCloseTo(0.075, within(1e-12));
    assertThat(between.rollRadians()).isCloseTo(0.05, within(1e-12));
  }

  @Test
  void outsideTheReadingsItHoldsTheNearest() {
    AttitudeHistory history = new AttitudeHistory(3);
    for (int k = 0; k < 5; k++) {
      history.add((100 + 20 * k) * MS, k, -k); // the first two fall out of three slots
    }

    assertThat(history.at(500 * MS)).isEqualTo(new AttitudeHistory.Attitude(4, -4));
    assertThat(history.at(0)).isEqualTo(new AttitudeHistory.Attitude(2, -2));
    assertThat(history.at(160 * MS)).isEqualTo(new AttitudeHistory.Attitude(3, -3));
  }

  @Test
  void aReadingNoNewerThanTheLastIsIgnored() {
    AttitudeHistory history = new AttitudeHistory(5);
    history.add(100 * MS, 0.1, 0.1);
    history.add(100 * MS, 0.5, 0.5);
    history.add(90 * MS, 0.5, 0.5);

    assertThat(history.at(100 * MS)).isEqualTo(new AttitudeHistory.Attitude(0.1, 0.1));
  }

  @Test
  void itKeepsAtLeastTwoReadings() {
    assertThatThrownBy(() -> new AttitudeHistory(1)).isInstanceOf(IllegalArgumentException.class);
  }
}
