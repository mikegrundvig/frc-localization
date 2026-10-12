package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.List;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * What the drivetrain read over one robot loop, for localization: every reading of its wheels since
 * the last loop, its IMUs, and how it's moving. Any drivetrain that reports these can be localized,
 * whatever its wheels: this is all the localization needs from it. Built by name, so the gyro and
 * the backup IMU can't trade places.
 *
 * <p>Whether its drive motors are at their current limit isn't among them: that says how hard the
 * motors push, not whether the wheels slip. A current limit set below the tread's grip, as it
 * should be (docs/values.md, {@code driveStatorCurrent}), keeps them from slipping; whether they
 * slip anyway, the wheels and the IMU say ({@link SlipCheck}).
 *
 * @param wheels every reading of the wheels since the last loop, oldest first
 * @param gyro the main gyro, which owns the robot's heading
 * @param backupImu a second IMU (Systemcore's own) that stands in while the gyro is lost, or {@link
 *     ImuReadings#NONE}
 * @param velocities how the robot is moving, robot-relative, as its wheels measure it
 */
@ConfigRecord
public record DrivetrainReadings(
    List<OdometrySample> wheels,
    ImuReadings gyro,
    ImuReadings backupImu,
    ChassisVelocities velocities) {
  public DrivetrainReadings {
    wheels = List.copyOf(wheels);
  }

  public static DrivetrainReadingsBuilder.WheelsStage builder() {
    return DrivetrainReadingsBuilder.builder();
  }
}
