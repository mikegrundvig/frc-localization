package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.field.Bump;
import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * A swerve drivetrain driven by script, for localization's tests without the simulator: what its
 * wheels, gyro and backup IMU read each loop, as a real drivetrain reports them. The robot moves
 * exactly as told, gaining speed as fast as its tread allows: its wheels roll and steer with it,
 * read at 250 Hz; the gyro turns with it (unless frozen) and is read with them; the accelerometers
 * feel the speed it gains, and gravity's share as it tilts; and over a bump, it tilts as the ramps
 * do. Its true pose is kept, to hold the estimate against.
 */
final class ScriptedDrivetrain {
  /** A loop, and the wheel readings in one: 250 Hz. */
  static final long LOOP_NANOS = 20_000_000L;

  private static final int SAMPLES = 5;
  private static final double SAMPLE_SECONDS = LOOP_NANOS / 1e9 / SAMPLES;

  /** How fast the robot gains speed, and turning speed: within its tread's grip. */
  private static final double ACCELERATION = 4;

  private static final double ANGULAR_ACCELERATION = 8;

  private final Translation2d[] modules;
  private final double[] distances;
  private final double[] angles;
  private long now;
  private Pose2d truth;
  private double gyroYaw;
  private ChassisVelocities velocity = new ChassisVelocities();

  /** The gyro stops answering. */
  boolean gyroLost;

  /** The backup IMU stops answering. */
  boolean backupLost;

  /**
   * The gyro keeps its reading, however the robot turns: what wheels sliding in a turn look like.
   */
  boolean gyroFrozen;

  /** The robot tilted by hand, nose down and left side up, beyond any bump. */
  double pitch;

  double roll;

  /** The bump on the floor, if there is one. */
  @Nullable Bump bump;

  /** The backup IMU's yaw is its own: it started at another zero. */
  private static final double BACKUP_ZERO = 1.0;

  /**
   * @param modules where each module sits from the robot's center
   * @param start where the robot stands
   * @param startNanos the time of the loop before the first
   * @param gyroYaw what the gyro reads there
   */
  ScriptedDrivetrain(Translation2d[] modules, Pose2d start, long startNanos, double gyroYaw) {
    this.modules = modules.clone();
    distances = new double[modules.length];
    angles = new double[modules.length];
    truth = start;
    now = startNanos;
    this.gyroYaw = gyroYaw;
  }

  /** The time of the last loop. */
  long now() {
    return now;
  }

  /** Where the robot really is. */
  Pose2d truth() {
    return truth;
  }

  /** Where the robot really is in 3D: as high as the bump under it, and tilted as it is. */
  Pose3d truth3d() {
    Bump on = bump;
    return new Pose3d(
        truth.getX(),
        truth.getY(),
        on == null ? 0 : on.heightAt(truth.getTranslation()),
        new Rotation3d(roll + bumpRoll(), pitch + bumpPitch(), truth.getRotation().getRadians()));
  }

  /** One loop, gaining toward this velocity, in the robot's frame: what the drivetrain reports. */
  DrivetrainReadings loop(ChassisVelocities toward) {
    List<OdometrySample> wheels = new ArrayList<>(SAMPLES);
    List<Long> yawTimes = new ArrayList<>(SAMPLES);
    List<Double> yaws = new ArrayList<>(SAMPLES);
    ChassisVelocities before = velocity;
    for (int k = 1; k <= SAMPLES; k++) {
      velocity =
          new ChassisVelocities(
              approach(velocity.vx, toward.vx, ACCELERATION),
              approach(velocity.vy, toward.vy, ACCELERATION),
              approach(velocity.omega, toward.omega, ANGULAR_ACCELERATION));
      for (int i = 0; i < modules.length; i++) {
        double vx = velocity.vx - velocity.omega * modules[i].getY();
        double vy = velocity.vy + velocity.omega * modules[i].getX();
        double speed = Math.hypot(vx, vy);
        if (speed > 1e-9) {
          angles[i] = Math.atan2(vy, vx);
        }
        distances[i] += speed * SAMPLE_SECONDS;
      }
      truth =
          truth.plus(
              new Twist2d(
                      velocity.vx * SAMPLE_SECONDS,
                      velocity.vy * SAMPLE_SECONDS,
                      velocity.omega * SAMPLE_SECONDS)
                  .exp());
      if (!gyroFrozen) {
        gyroYaw += velocity.omega * SAMPLE_SECONDS;
      }
      long t = now + k * LOOP_NANOS / SAMPLES;
      wheels.add(new OdometrySample(t, distances, angles));
      yawTimes.add(t);
      yaws.add(gyroYaw);
    }
    now += LOOP_NANOS;
    double seconds = LOOP_NANOS / 1e9;
    double forward = (velocity.vx - before.vx) / seconds;
    double left = (velocity.vy - before.vy) / seconds;
    double pitchNow = pitch + bumpPitch();
    double rollNow = roll + bumpRoll();
    return DrivetrainReadings.builder()
        .wheels(wheels)
        .gyro(imu(gyroLost, 0, yawTimes, yaws, pitchNow, rollNow, forward, left))
        .backupImu(imu(backupLost, BACKUP_ZERO, yawTimes, yaws, pitchNow, rollNow, forward, left))
        .velocities(velocity)
        .build();
  }

  /** One IMU's readings: its yaw on its own zero, the robot's tilt, and what it felt. */
  private ImuReadings imu(
      boolean lost,
      double zero,
      List<Long> yawTimes,
      List<Double> yaws,
      double pitch,
      double roll,
      double forward,
      double left) {
    if (lost) {
      return ImuReadings.NONE;
    }
    return ImuReadings.builder()
        .reporting(true)
        .yawRadians(gyroYaw + zero)
        .yawRateRadiansPerSecond(gyroFrozen ? 0 : velocity.omega)
        .pitchRadians(pitch)
        .rollRadians(roll)
        // What it felt, and gravity's share as the robot tilts.
        .forwardAccelerationMetersPerSecSq(forward - ImuReadings.STANDARD_GRAVITY * Math.sin(pitch))
        .leftAccelerationMetersPerSecSq(
            left + ImuReadings.STANDARD_GRAVITY * Math.sin(roll) * Math.cos(pitch))
        .yaws(
            YawSamples.of(
                yawTimes.stream().mapToLong(Long::longValue).toArray(),
                yaws.stream().mapToDouble(yaw -> yaw + zero).toArray()))
        .build();
  }

  /**
   * How the bump under the robot pitches it: its ramp rising toward the crest, along the way the
   * robot faces; level off the bump.
   */
  private double bumpPitch() {
    return bumpTilt(truth.getRotation());
  }

  /** How the bump rolls it: the ramp rising toward its left side lifts it. */
  private double bumpRoll() {
    return -bumpTilt(truth.getRotation().plus(Rotation2d.fromDegrees(90)));
  }

  /**
   * The ramp's slope under the robot, seen along a direction: positive where the ground falls away
   * that way (a nose-down pitch, facing it).
   */
  private double bumpTilt(Rotation2d facing) {
    Bump on = bump;
    if (on == null) {
      return 0;
    }
    double offset = on.acrossOffset(truth.getTranslation());
    if (Math.abs(offset) >= on.depthMeters() / 2 || offset == 0) {
      return 0;
    }
    // The ground rises toward the crest: against the across direction on its far side.
    double rising = -Math.signum(offset);
    double along = Math.cos(facing.minus(on.across()).getRadians());
    return Math.atan(-rising * Math.tan(on.slopeRadians()) * along);
  }

  /** A value moved toward a target by no more than a rate allows in one sample. */
  private static double approach(double value, double target, double rate) {
    double step = rate * SAMPLE_SECONDS;
    return value + Math.clamp(target - value, -step, step);
  }
}
