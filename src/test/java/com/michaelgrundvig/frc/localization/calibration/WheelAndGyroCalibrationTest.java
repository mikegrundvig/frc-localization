package com.michaelgrundvig.frc.localization.calibration;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/** Spinning in place: the wheels against the gyro, and the gyro against the cameras. */
class WheelAndGyroCalibrationTest {
  private static final Translation2d[] SQUARE = {
    new Translation2d(0.3, 0.3),
    new Translation2d(0.3, -0.3),
    new Translation2d(-0.3, 0.3),
    new Translation2d(-0.3, -0.3)
  };
  private static final double WHEEL = 0.05;

  /** Spins one turn in 100 steps; each wheel turns by how far it rolls over its own radius. */
  private static WheelRadiusCalibration spin(double gyroScale, double... wheelRadii) {
    WheelRadiusCalibration calibration = new WheelRadiusCalibration(SQUARE);
    calibration.start(1.0, new double[4]); // the gyro started at 1 rad
    for (int step = 1; step <= 100; step++) {
      double turned = 2 * Math.PI * step / 100;
      double[] wheels = new double[4];
      for (int i = 0; i < 4; i++) {
        wheels[i] = turned * SQUARE[i].getNorm() / wheelRadii[i];
      }
      calibration.update(MathUtil.angleModulus(gyroScale * turned + 1.0), wheels);
    }
    return calibration;
  }

  @Test
  void theWheelsRadiusIsHowFarTheyRolledOverHowFarTheyTurned() {
    WheelRadiusCalibration calibration = spin(1, WHEEL, WHEEL, WHEEL, WHEEL);

    assertThat(calibration.turnedRadians()).isCloseTo(2 * Math.PI, within(1e-9));
    assertThat(calibration.radiusMeters()).isCloseTo(WHEEL, within(1e-12));
    assertThat(calibration.radiiMeters()).containsOnly(WHEEL);
  }

  @Test
  void aWornWheelShowsInItsOwnRadius() {
    WheelRadiusCalibration calibration = spin(1, WHEEL, WHEEL * 0.98, WHEEL, WHEEL);

    assertThat(calibration.radiiMeters()[1]).isCloseTo(WHEEL * 0.98, within(1e-12));
    assertThat(calibration.radiusMeters()).isBetween(WHEEL * 0.98, WHEEL);
  }

  @Test
  void theGyrosScaleErrorCarriesIntoTheRadius() {
    assertThat(spin(1.003, WHEEL, WHEEL, WHEEL, WHEEL).radiusMeters())
        .isCloseTo(WHEEL * 1.003, within(1e-12));
  }

  @Test
  void aWheelRadiusSpinChecksItsInputs() {
    WheelRadiusCalibration calibration = new WheelRadiusCalibration(SQUARE);

    assertThatThrownBy(() -> calibration.update(0, new double[4]))
        .isInstanceOf(IllegalStateException.class);
    assertThatThrownBy(() -> calibration.start(0, new double[3]))
        .isInstanceOf(IllegalArgumentException.class);
    calibration.start(0, new double[4]);
    assertThat(calibration.radiusMeters()).isNaN();
    assertThat(calibration.radiiMeters()[0]).isNaN();
    assertThatThrownBy(() -> new WheelRadiusCalibration(new Translation2d[] {Translation2d.ZERO}))
        .isInstanceOf(IllegalArgumentException.class);
  }

  /** Five turns counterclockwise, stopping a little past where it started. */
  @Test
  void theGyrosScaleIsTheTrueTurnOverTheOneItCounted() {
    double scale = 1.003;
    double trueTurn = 5 * 2 * Math.PI + 0.1;
    GyroScaleCalibration calibration = new GyroScaleCalibration();

    calibration.start(0.4, Rotation2d.fromRadians(3.0)); // near the wrap on purpose
    for (int step = 1; step <= 500; step++) {
      calibration.update(MathUtil.angleModulus(0.4 + scale * trueTurn * step / 500));
    }
    calibration.finish(
        MathUtil.angleModulus(0.4 + scale * trueTurn), Rotation2d.fromRadians(3.0 + 0.1));

    assertThat(calibration.gyroTurnedRadians()).isCloseTo(scale * trueTurn, within(1e-9));
    assertThat(calibration.trueTurnedRadians()).isCloseTo(trueTurn, within(1e-9));
    assertThat(calibration.scale()).isCloseTo(1 / scale, within(1e-12));
  }

  @Test
  void clockwiseWorksTheSame() {
    GyroScaleCalibration calibration = new GyroScaleCalibration();
    calibration.start(0, Rotation2d.ZERO);
    double trueTurn = -3 * 2 * Math.PI;
    for (int step = 1; step <= 300; step++) {
      calibration.update(MathUtil.angleModulus(0.99 * trueTurn * step / 300));
    }
    calibration.finish(MathUtil.angleModulus(0.99 * trueTurn), Rotation2d.ZERO);

    assertThat(calibration.scale()).isCloseTo(1 / 0.99, within(1e-12));
  }

  @Test
  void aGyroScaleSpinChecksItsInputs() {
    GyroScaleCalibration calibration = new GyroScaleCalibration();

    assertThatThrownBy(() -> calibration.update(0)).isInstanceOf(IllegalStateException.class);
    calibration.start(0, Rotation2d.ZERO);
    assertThat(calibration.scale()).as("not finished").isNaN();
    calibration.finish(0, Rotation2d.ZERO);
    assertThat(calibration.scale()).as("never turned").isNaN();
  }

  @Test
  void headingsAverageAroundTheCircle() {
    HeadingAverage average = new HeadingAverage();
    assertThat(average.average()).isEqualTo(Rotation2d.ZERO);
    assertThat(average.agreement()).isZero();

    average.add(Rotation2d.fromDegrees(179));
    average.add(Rotation2d.fromDegrees(-179));

    assertThat(Math.abs(average.average().getDegrees())).isCloseTo(180, within(1e-9));
    assertThat(average.agreement()).isCloseTo(Math.cos(Math.toRadians(1)), within(1e-12));
    assertThat(average.count()).isEqualTo(2);
    average.clear();
    assertThat(average.count()).isZero();
  }

  @Test
  void theLevelCheckAveragesAndSaysWhatToDo() {
    LevelCheck level = new LevelCheck();
    assertThat(level.pitchRadians()).isZero();

    level.add(Math.toRadians(0.2), Math.toRadians(-0.4));
    level.add(Math.toRadians(0.4), Math.toRadians(-0.6));

    assertThat(Math.toDegrees(level.pitchRadians())).isCloseTo(0.3, within(1e-9));
    assertThat(Math.toDegrees(level.rollRadians())).isCloseTo(-0.5, within(1e-9));
    assertThat(level.describe(Math.toRadians(1)))
        .isEqualTo("Level: pitch 0.30°, roll -0.50° over 2 readings");
    assertThat(level.describe(Math.toRadians(0.4))).startsWith("Not level:").endsWith("mount");
  }
}
