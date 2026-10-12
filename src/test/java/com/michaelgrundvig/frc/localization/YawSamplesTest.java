package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import org.junit.jupiter.api.Test;

/** An IMU's yaw readings over a loop: copied, or handed over as they are. */
class YawSamplesTest {
  /** Copied readings are their own: changing the arrays afterwards changes nothing. */
  @Test
  void copiedReadingsAreTheirOwn() {
    long[] times = {10, 20};
    double[] yaws = {0.1, 0.2};

    YawSamples samples = YawSamples.of(times, yaws);
    times[0] = 99;
    yaws[1] = 9.9;

    assertThat(samples.count()).isEqualTo(2);
    assertThat(samples.timeNanos(0)).isEqualTo(10);
    assertThat(samples.timeNanos(1)).isEqualTo(20);
    assertThat(samples.yawRadians(0)).isEqualTo(0.1);
    assertThat(samples.yawRadians(1)).isEqualTo(0.2);
  }

  /** One time for each yaw, or the readings are refused. */
  @Test
  void copiedReadingsNeedATimeForEachYaw() {
    assertThatThrownBy(() -> YawSamples.of(new long[] {1, 2, 3}, new double[] {0.3}))
        .hasMessage(
            "ImuReadings.yawsRadians: must have one reading for each time: 3 times, 1 readings");
  }

  /** Handed over, only as many readings count as are said to, from the start of both arrays. */
  @Test
  void handedOverReadingsCountOnlyAsManyAsSaid() {
    YawSamples samples = YawSamples.handedOver(new long[] {10, 20, 30}, new double[] {0.1, 0.2}, 2);

    assertThat(samples.count()).isEqualTo(2);
    assertThat(samples.timeNanos(1)).isEqualTo(20);
    assertThat(samples.yawRadians(1)).isEqualTo(0.2);
    assertThatThrownBy(() -> samples.timeNanos(2))
        .isInstanceOf(IndexOutOfBoundsException.class)
        .hasMessage("reading 2 of 2");
    assertThatThrownBy(() -> samples.yawRadians(-1))
        .isInstanceOf(IndexOutOfBoundsException.class)
        .hasMessage("reading -1 of 2");
  }

  /** More readings counted than either array has, or fewer than none, is refused. */
  @Test
  void handedOverReadingsCannotCountMoreThanThereAre() {
    long[] times = {10, 20, 30};
    double[] yaws = {0.1, 0.2};

    assertThat(YawSamples.handedOver(times, yaws, 0).count()).isZero();
    assertThatThrownBy(() -> YawSamples.handedOver(times, yaws, 3))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("3 readings counted, of 3 times and 2 yaws");
    assertThatThrownBy(() -> YawSamples.handedOver(yawsLong(2), new double[3], 3))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("3 readings counted, of 2 times and 3 yaws");
    assertThatThrownBy(() -> YawSamples.handedOver(times, yaws, -1))
        .isInstanceOf(IllegalArgumentException.class);
  }

  private static long[] yawsLong(int length) {
    return new long[length];
  }

  /** A yaw that isn't a number is noticed only among the readings that count. */
  @Test
  void everyYawThatCountsIsANumberOrItSaysSo() {
    double[] lastIsNot = {0.1, 0.2, Double.NaN};
    long[] times = {1, 2, 3};

    assertThat(YawSamples.NONE.areAllNumbers()).isTrue();
    assertThat(YawSamples.handedOver(times, lastIsNot, 3).areAllNumbers()).isFalse();
    assertThat(YawSamples.handedOver(times, lastIsNot, 2).areAllNumbers()).isTrue();
    assertThat(YawSamples.of(times, new double[] {Double.NEGATIVE_INFINITY, 0, 0}).areAllNumbers())
        .isFalse();
  }

  /** Equal when the readings that count are the same, whatever lies past them. */
  @Test
  void equalByTheReadingsThatCount() {
    YawSamples copied = YawSamples.of(new long[] {10, 20}, new double[] {0.1, 0.2});
    YawSamples handedOver =
        YawSamples.handedOver(new long[] {10, 20, 30}, new double[] {0.1, 0.2, 0.3}, 2);

    assertThat(handedOver).isEqualTo(copied).hasSameHashCodeAs(copied);
    assertThat(copied)
        .isNotEqualTo(YawSamples.of(new long[] {10, 21}, new double[] {0.1, 0.2}))
        .isNotEqualTo(YawSamples.of(new long[] {10, 20}, new double[] {0.1, 0.25}))
        .isNotEqualTo(YawSamples.of(new long[] {10}, new double[] {0.1}))
        .isNotEqualTo("readings");
    assertThat(copied.hashCode())
        .isNotEqualTo(YawSamples.of(new long[] {10, 21}, new double[] {0.1, 0.2}).hashCode())
        .isNotEqualTo(YawSamples.NONE.hashCode());
    assertThat(handedOver).hasToString("YawSamples[0.1 rad at 10 ns, 0.2 rad at 20 ns]");
    assertThat(YawSamples.NONE).hasToString("YawSamples[]");
  }
}
