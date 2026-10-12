package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;

class OdometrySampleTest {
  @Test
  void itKeepsItsOwnCopyOfTheReadings() {
    double[] distances = {1.0, 2.0};
    double[] angles = {0.0, 0.5};
    OdometrySample sample = new OdometrySample(20_000_000L, distances, angles);

    distances[0] = 99;
    angles[1] = 99;

    assertThat(sample.timestampNanos()).isEqualTo(20_000_000L);
    assertThat(sample.wheels()).isEqualTo(2);
    assertThat(sample.distanceMeters(0)).isEqualTo(1.0);
    assertThat(sample.angleRadians(1)).isEqualTo(0.5);
  }

  @Test
  void swerveModulesGoInAndComeBackOut() {
    SwerveModulePosition[] modules = {
      new SwerveModulePosition(1.5, Rotation2d.fromDegrees(30)),
      new SwerveModulePosition(-0.25, Rotation2d.fromDegrees(-90)),
    };

    OdometrySample sample = OdometrySample.ofSwerve(5L, modules);

    assertThat(sample.toSwerveModulePositions()).containsExactly(modules);
  }

  /** A tank drive is two wheels that always point straight ahead. */
  @Test
  void aTankDriveIsTwoWheelsAtAngleZero() {
    OdometrySample tank = new OdometrySample(0, new double[] {3.0, 3.2}, new double[] {0, 0});

    assertThat(tank.wheels()).isEqualTo(2);
  }

  @Test
  void samplesWithTheSameReadingsAreEqual() {
    OdometrySample one = new OdometrySample(1, new double[] {1, 2}, new double[] {0, 0.1});
    OdometrySample same = new OdometrySample(1, new double[] {1, 2}, new double[] {0, 0.1});
    OdometrySample later = new OdometrySample(2, new double[] {1, 2}, new double[] {0, 0.1});

    assertThat(one).isEqualTo(same).hasSameHashCodeAs(same).isNotEqualTo(later);
    assertThat(one).isNotEqualTo("a string");
    assertThat(one.toString()).contains("t=1 ns", "[1.0, 2.0]");
  }

  /** A wheel that didn't report reads a value that isn't a number, its distance or its angle. */
  @Test
  void aWheelThatReadsNoNumberDidntReport() {
    OdometrySample sample =
        new OdometrySample(
            0, new double[] {1, Double.NaN, 2}, new double[] {0, 0, Double.POSITIVE_INFINITY});

    assertThat(sample.reported(0)).isTrue();
    assertThat(sample.reported(1)).isFalse();
    assertThat(sample.reported(2)).isFalse();
  }

  @Test
  void everyWheelNeedsADistanceAndAnAngle() {
    assertThatThrownBy(() -> new OdometrySample(0, new double[] {}, new double[] {}))
        .hasMessageContaining("each wheel");
    assertThatThrownBy(() -> new OdometrySample(0, new double[] {1, 2}, new double[] {0}))
        .hasMessageContaining("2 distances, 1 angles");
    assertThatThrownBy(() -> new OdometrySample(-1, new double[] {0}, new double[] {0}))
        .hasMessageContaining("robot's clock");
  }
}
