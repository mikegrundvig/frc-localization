package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;

/** A gyro read once a loop, asked for its yaw at each of the wheels' 250 Hz readings. */
class YawTrackTest {
  private static final long MS = 1_000_000L;

  @Test
  void atAReadingsOwnTimeItIsThatReadingExactly() {
    YawTrack track = new YawTrack();
    track.update(new long[] {4 * MS, 8 * MS}, new double[] {0.1, 0.3000000000000001});

    assertThat(track.at(8 * MS)).isEqualTo(0.3000000000000001);
  }

  @Test
  void betweenLoopsItInterpolatesFromTheLastLoopsNewest() {
    YawTrack track = new YawTrack();
    track.update(new long[] {20 * MS}, new double[] {1.0});
    track.update(new long[] {40 * MS}, new double[] {1.2});

    assertThat(track.at(24 * MS)).isCloseTo(1.04, within(1e-12));
    assertThat(track.at(36 * MS)).isCloseTo(1.16, within(1e-12));
  }

  @Test
  void itInterpolatesTheShortWayAround() {
    YawTrack track = new YawTrack();
    track.update(new long[] {0}, new double[] {Math.PI - 0.1});
    track.update(new long[] {20 * MS}, new double[] {-Math.PI + 0.1});

    assertThat(track.at(10 * MS)).isCloseTo(Math.PI, within(1e-12));
  }

  @Test
  void outsideItsReadingsItHoldsTheNearest() {
    YawTrack track = new YawTrack();
    track.update(new long[] {20 * MS, 24 * MS}, new double[] {1.0, 1.1});

    assertThat(track.at(10 * MS)).as("before any earlier reading").isEqualTo(1.0);
    assertThat(track.at(30 * MS)).isEqualTo(1.1);
  }

  @Test
  void clearedItHasNoReadingsAndStartsOver() {
    YawTrack track = new YawTrack();
    track.update(new long[] {20 * MS}, new double[] {1.0});
    track.clear();

    assertThat(track.hasReadings()).isFalse();
    assertThatThrownBy(() -> track.at(20 * MS)).isInstanceOf(IllegalStateException.class);
    track.update(new long[] {40 * MS}, new double[] {2.0});
    assertThat(track.at(30 * MS)).as("nothing before the reset to interpolate from").isEqualTo(2.0);
  }

  @Test
  void eachYawHasItsTime() {
    assertThatThrownBy(() -> new YawTrack().update(new long[] {1, 2}, new double[] {0}))
        .isInstanceOf(IllegalArgumentException.class);
  }
}
