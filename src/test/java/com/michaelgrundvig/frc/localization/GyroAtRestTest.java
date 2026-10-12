package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;

/**
 * The gyro's drift read off a stretch of standing still, fed by hand: a gyro whose noise adds a
 * heading variance of 4e-6 rad² each second, and which may drift 0.001 rad/s before anything is
 * known of it. A one-second stretch then reads the drift with a variance of 4e-6, and counts while
 * it reads no more than 3 sqrt(0.001² + 4e-6) = 0.00671 rad/s.
 */
class GyroAtRestTest {
  private static final BodyMotion STILL = new BodyMotion(0, 0, 0, 0);

  private final GyroAtRest atRest = new GyroAtRest(4e-6, 0.001);

  /** Stands for some steps, the gyro turning at a rate; the last step's answer. */
  private double stand(int steps, double stepSeconds, double gyroRate) {
    double last = Double.NaN;
    for (int i = 0; i < steps; i++) {
      last = atRest.step(stepSeconds, STILL, gyroRate * stepSeconds);
    }
    return last;
  }

  @Test
  void aStretchOfStandingReadsTheDriftAsItEnds() {
    assertThat(atRest.variance()).isCloseTo(4e-6, within(1e-15));

    assertThat(stand(3, 0.3, 0.002)).as("0.9 s is not a stretch yet").isNaN();
    // The fourth step ends the stretch at 1.2 s: the robot turned 0.002 rad/s slower than its
    // gyro said.
    assertThat(stand(1, 0.3, 0.002)).isCloseTo(-0.002, within(1e-12));
    assertThat(stand(3, 0.3, 0.002)).as("the next stretch starts from nothing").isNaN();
  }

  @Test
  void aStretchEndsAtOneSecondExactly() {
    assertThat(stand(3, 0.25, 0.002)).isNaN();
    assertThat(stand(1, 0.25, 0.002)).isCloseTo(-0.002, within(1e-12));
  }

  @Test
  void aDriftNoGyroCouldHaveIsARobotBeingCarried() {
    assertThat(stand(4, 0.25, 0.0067)).as("just believable").isCloseTo(-0.0067, within(1e-12));
    assertThat(stand(4, 0.25, 0.0068)).as("just past it").isNaN();
    assertThat(stand(4, 0.25, -0.0068)).isNaN();
  }

  /** Wheels that creep turn the robot a little, and the drift is what the gyro reads beyond it. */
  @Test
  void whatTheWheelsTurnTheRobotIsNotDrift() {
    double read = Double.NaN;
    for (int i = 0; i < 4; i++) {
      read = atRest.step(0.25, new BodyMotion(0.001, 0, 0.25 * 0.004, 0), 0.25 * 0.005);
    }
    assertThat(read).isCloseTo(-0.001, within(1e-12));
  }

  @Test
  void aRobotThatMovesOrTurnsIsNotStanding() {
    // Still is under 0.02 m/s and under 0.05 rad/s: over a 0.25 s step, 5 mm and 0.0125 rad.
    atRest.step(0.25, new BodyMotion(0.003, 0.0039, 0, 0), 0);
    atRest.step(0.25, new BodyMotion(0, 0, -0.0124, 0), -0.0124);
    atRest.step(0.25, STILL, 0);
    assertThat(atRest.step(0.25, STILL, 0))
        .as("just still, throughout")
        .isCloseTo(0, within(1e-12));

    assertThat(stand(3, 0.25, 0)).isNaN();
    assertThat(atRest.step(0.25, new BodyMotion(0.003, 0.004, 0, 0), 0))
        .as("5 mm in a step")
        .isNaN();
    assertThat(stand(3, 0.25, 0)).as("which started the stretch again").isNaN();
    assertThat(atRest.step(0.25, new BodyMotion(0, 0, 0.0125, 0), 0.0125)).as("turning").isNaN();
    assertThat(stand(3, 0.25, 0)).isNaN();
    assertThat(stand(1, 0.25, 0)).isZero();
  }

  @Test
  void clearedItStartsAgain() {
    stand(3, 0.25, 0.001);
    atRest.clear();
    assertThat(stand(3, 0.25, 0.001)).isNaN();
    assertThat(stand(1, 0.25, 0.001)).isCloseTo(-0.001, within(1e-12));
  }
}
