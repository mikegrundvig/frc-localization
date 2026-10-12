package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.function.DoubleUnaryOperator;
import org.junit.jupiter.api.Test;

/** Two IMUs' yaw, compared over a sliding second, at 50 Hz. */
class ImuCrossCheckTest {
  private static final long LOOP = 20_000_000L;

  private final ImuCrossCheck check = new ImuCrossCheck(ImuCrossCheck.Settings.DEFAULTS);
  private long time;
  private double truth;

  /** Turns at a rate for a while; each IMU reports what it makes of the true yaw. */
  private void turn(
      double seconds,
      double radiansPerSecond,
      DoubleUnaryOperator primary,
      DoubleUnaryOperator backup) {
    for (int i = 0; i < Math.round(seconds / 0.02); i++) {
      time += LOOP;
      truth += radiansPerSecond * 0.02;
      check.update(time, primary.applyAsDouble(truth), backup.applyAsDouble(truth));
    }
  }

  @Test
  void twoGoodGyrosAgreeEvenWithSlightlyDifferentScalesAndZeros() {
    // A full turn a second, one reading 2% high from a different zero, wrapping as they go.
    turn(3, 2 * Math.PI, yaw -> yaw, yaw -> Math.IEEEremainder(1.02 * yaw + 1.3, 2 * Math.PI));

    assertThat(check.disagrees()).isFalse();
    assertThat(check.disagreementRadians()).isCloseTo(-0.02 * 2 * Math.PI, within(0.01));
    assertThat(check.turnedRadians()).isCloseTo(2 * Math.PI, within(0.2));
  }

  @Test
  void aFrozenGyroDisagreesAsSoonAsTheRobotTurns() {
    turn(1, 0, yaw -> yaw, yaw -> yaw);
    boolean still = check.disagrees();

    turn(0.2, 1.0, yaw -> 0, yaw -> yaw); // stuck at zero while the robot turns 11°

    assertThat(still).isFalse();
    assertThat(check.disagrees()).isTrue();
    assertThat(Math.toDegrees(check.disagreementRadians())).isCloseTo(-11.5, within(0.6));
  }

  @Test
  void anOldDisagreementLeavesTheWindow() {
    turn(0.2, 1.0, yaw -> 0, yaw -> yaw);
    turn(1.5, 0, yaw -> 0, yaw -> yaw); // still since: the frozen reading is right again

    assertThat(check.disagrees()).isFalse();
  }

  @Test
  void resetStartsOver() {
    turn(0.2, 1.0, yaw -> 0, yaw -> yaw);
    check.reset();

    assertThat(check.disagreementRadians()).isZero();
    assertThat(check.turnedRadians()).isZero();
    assertThat(check.disagrees()).isFalse();
  }

  @Test
  void settingsMustMakeSense() {
    assertThatThrownBy(() -> settings().windowSeconds(0).build())
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> settings().toleranceDegrees(0).build())
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> settings().scaleTolerance(-1).build())
        .isInstanceOf(IllegalArgumentException.class);
  }

  private static ImuCrossCheckSettingsBuilder settings() {
    return ImuCrossCheckSettingsBuilder.builder(ImuCrossCheck.Settings.DEFAULTS);
  }
}
