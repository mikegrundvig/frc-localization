package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.kinematics.ChassisVelocities;

/** One loop's readings, as a drivetrain hands them to localization. */
class DrivetrainReadingsTest {
  /** Each IMU is named where it's set, so the gyro and the backup can't trade places. */
  @Test
  void eachImuIsTheOneItsNamed() {
    ImuReadings gyro = ImuReadingsBuilder.builder(ImuReadings.NONE).yawRadians(0.5).build();

    DrivetrainReadings readings = readings(new ArrayList<>(), gyro);

    assertThat(readings.gyro()).isEqualTo(gyro);
    assertThat(readings.backupImu()).isEqualTo(ImuReadings.NONE);
    assertThat(readings.velocities()).isEqualTo(new ChassisVelocities(1, 0, 0));
  }

  /** The wheels' readings are the loop's: the drivetrain can't change them once handed over. */
  @Test
  void theWheelsReadingsCantChangeAfterwards() {
    List<OdometrySample> wheels =
        new ArrayList<>(List.of(new OdometrySample(20_000_000L, new double[4], new double[4])));
    DrivetrainReadings readings = readings(wheels, ImuReadings.NONE);

    wheels.clear();

    assertThat(readings.wheels()).hasSize(1);
  }

  private static DrivetrainReadings readings(List<OdometrySample> wheels, ImuReadings gyro) {
    return DrivetrainReadings.builder()
        .wheels(wheels)
        .gyro(gyro)
        .backupImu(ImuReadings.NONE)
        .velocities(new ChassisVelocities(1, 0, 0))
        .build();
  }
}
