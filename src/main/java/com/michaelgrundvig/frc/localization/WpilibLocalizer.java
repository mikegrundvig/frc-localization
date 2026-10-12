package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.TreeSet;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.linalg.VecBuilder;

/**
 * WPILib's {@code SwerveDrivePoseEstimator} as a {@link Localizer}, for a swerve robot. Choose it
 * in a robot's configuration:
 *
 * <pre>{@code
 * .localizer(WpilibLocalizer::new)
 * }</pre>
 *
 * <p>It is given what the {@link DefaultLocalizer} is given: every wheel reading, at the rate the
 * drivetrain reads them, and the vision pipeline's measurements with their standard deviations. It
 * keeps the required part of the contract and reports none of the options, because WPILib's
 * estimator keeps no uncertainty, no mode, and no record of what it did with a measurement. The
 * robot program says "not reported" for each.
 *
 * <p>One consequence reaches the measurements themselves. With no {@link #covariance}, nothing says
 * its heading is sure, so the pipeline solves no image with it: every image is solved from its tags
 * alone, where the default localizer has most of its images solved with the gyro's heading.
 *
 * <p>WPILib's estimator leaves some things to the code that feeds it. What this does about each:
 *
 * <ul>
 *   <li><b>Which sensor turns it.</b> WPILib takes one gyro angle beside each wheel reading. This
 *       gives it the gyro's while the gyro has readings, the backup IMU's while only that has, and
 *       the wheels' own turning with neither. The angle carries on from where it was when the
 *       source changes, so two IMUs' different zeros never show as a turn.
 *   <li><b>A wheel that did not report.</b> It is taken to be where it last was.
 *   <li><b>Placing the robot.</b> WPILib's reset takes a pose and nothing more. How sure the
 *       placement is has nowhere to go, and is dropped. A placement that says nothing of the
 *       heading keeps the heading.
 *   <li><b>The order of measurements.</b> When WPILib's estimator is given a measurement older than
 *       corrections it already made, it discards those corrections. So each loop's measurements are
 *       given to it oldest first. {@link #discardedLater} counts the corrections still lost that
 *       way, to an image that arrives a loop after a newer one.
 *   <li><b>A measurement with no heading.</b> WPILib has no way to say a measurement knows nothing
 *       of the heading. Such a measurement is given a heading standard deviation of a million
 *       radians, which turns the estimate by less than a millionth of the difference.
 *   <li><b>How x and y vary together.</b> WPILib takes a standard deviation for each of x, y and
 *       heading. A measurement's covariance between x and y is dropped.
 *   <li><b>Measurements it cannot use.</b> WPILib drops one older than the second and a half of
 *       odometry it keeps, without saying so. {@link #tooOld} counts them.
 *   <li><b>How far odometry is trusted.</b> The state standard deviations are WPILib's defaults,
 *       0.1 m and 0.1 rad, unless the robot gives its own. WPILib uses them with each measurement's
 *       to set one fixed gain for that measurement. It does not grow less sure as the robot drives
 *       on without a measurement, and it gates nothing.
 * </ul>
 *
 * <p>Times go to WPILib in seconds on the robot's clock.
 */
public final class WpilibLocalizer implements Localizer {
  /** WPILib's default state standard deviation for x and y, in meters. */
  public static final double DEFAULT_STATE_STD_DEV_METERS = 0.1;

  /** WPILib's default state standard deviation for the heading, in radians. */
  public static final double DEFAULT_STATE_STD_DEV_RADIANS = 0.1;

  /** A heading standard deviation that keeps a measurement from correcting the heading. */
  private static final double NO_HEADING_STD_DEV_RADIANS = 1e6;

  /** WPILib keeps this much odometry, in seconds: an older measurement is dropped. */
  private static final double BUFFER_SECONDS = 1.5;

  /**
   * A moment this long after the newest wheel reading is still "now", in nanoseconds: one robot
   * loop, since an image can be stamped after the loop's last wheel reading.
   */
  private static final long STILL_NOW_NANOS = 20_000_000L;

  /** Which sensor the yaw came from at a wheel reading. */
  private enum Turning {
    GYRO,
    BACKUP,
    WHEELS,
    /** Before the first wheel reading. */
    NOTHING_YET
  }

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator estimator;

  /** The yaw given to WPILib, in radians: carried across a change of the sensor that turns it. */
  private double yawRadians;

  private Turning turnedBy = Turning.NOTHING_YET;

  /** What the sensor that turned the yaw last read, to measure its next reading from. */
  private double lastYawReading;

  /** The wheels as they were last read: zero until the first reading. */
  private SwerveModulePosition[] wheels;

  /** The time of the newest wheel reading, and zero before the first. */
  private long newestNanos;

  /** Whether a wheel reading has come since the localizer was made or the robot last placed. */
  private boolean hasOdometry;

  /** The time of the oldest wheel reading WPILib can still hold since the robot was placed. */
  private long historyStartNanos;

  /** The capture times of the corrections WPILib still holds, in seconds, as its own map has. */
  private final TreeSet<Double> corrections = new TreeSet<>();

  private int offered;
  private int added;
  private int discardedLater;
  private int tooOld;

  /**
   * WPILib's estimator for a robot, with WPILib's default state standard deviations. Until the
   * robot is placed its pose is the middle of the field, facing along the field's x.
   *
   * @param robot what the robot is: where its swerve modules are
   * @param field the field the robot plays on
   */
  public WpilibLocalizer(RobotDefinition robot, FieldModel field) {
    this(robot, field, DEFAULT_STATE_STD_DEV_METERS, DEFAULT_STATE_STD_DEV_RADIANS);
  }

  /**
   * The same, trusting odometry as the robot says. A smaller value follows the wheels more closely
   * and each camera measurement less.
   *
   * @param stateStdDevMeters the state standard deviation for x and y, in meters
   * @param stateStdDevRadians and for the heading, in radians
   */
  public WpilibLocalizer(
      RobotDefinition robot,
      FieldModel field,
      double stateStdDevMeters,
      double stateStdDevRadians) {
    kinematics = new SwerveDriveKinematics(robot.moduleTranslations());
    wheels = new SwerveModulePosition[kinematics.getModules().length];
    for (int i = 0; i < wheels.length; i++) {
      wheels[i] = new SwerveModulePosition();
    }
    Pose2d notPlacedYet = new Pose2d(field.length() / 2, field.width() / 2, Rotation2d.ZERO);
    estimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.ZERO,
            wheels,
            notPlacedYet,
            VecBuilder.fill(stateStdDevMeters, stateStdDevMeters, stateStdDevRadians),
            // Every measurement comes with its own, so these are never used.
            VecBuilder.fill(0.9, 0.9, 0.9));
  }

  /** A maker of one that trusts odometry as given, for a robot's configuration. */
  public static LocalizerMaker with(double stateStdDevMeters, double stateStdDevRadians) {
    return (robot, field) ->
        new WpilibLocalizer(robot, field, stateStdDevMeters, stateStdDevRadians);
  }

  // ---- What it is told ----

  @Override
  public void odometry(DrivetrainReadings readings, LoopContext loop) {
    for (OdometrySample sample : readings.wheels()) {
      SwerveModulePosition[] read = positions(sample);
      Turning before = turnedBy;
      Rotation2d yaw = yawAt(sample, read, readings);
      wheels = read;
      newestNanos = sample.timestampNanos();
      double seconds = Timestamps.toSeconds(newestNanos);
      if (before == Turning.NOTHING_YET) {
        // Wheels that have already rolled are where the robot starts, not a distance it drove.
        estimator.resetPosition(yaw, read, estimator.getEstimatedPosition());
      }
      if (!hasOdometry) {
        hasOdometry = true;
        historyStartNanos = newestNanos;
      }
      estimator.updateWithTime(seconds, yaw, read);
    }
  }

  /** Resets WPILib's estimator to the pose. How sure the placement is is dropped. */
  @Override
  public void place(PoseReset placed) {
    Pose2d pose = placed.pose();
    if (!placed.covariance().hasHeading()) {
      pose = new Pose2d(pose.getTranslation(), pose().getRotation());
    }
    estimator.resetPosition(new Rotation2d(yawRadians), wheels, pose);
    // The reset empties WPILib's history and its corrections.
    corrections.clear();
    hasOdometry = false;
  }

  @Override
  public void measurements(List<VisionMeasurement> taken) {
    List<VisionMeasurement> oldestFirst = new ArrayList<>(taken);
    oldestFirst.sort(Comparator.comparingLong(VisionMeasurement::timestampNanos));
    for (VisionMeasurement measurement : oldestFirst) {
      offered++;
      double seconds = Timestamps.toSeconds(measurement.timestampNanos());
      if (!willCorrect(seconds)) {
        continue;
      }
      PoseCovariance covariance = measurement.covariance();
      double headingStdDev = NO_HEADING_STD_DEV_RADIANS;
      if (covariance.hasHeading()) {
        headingStdDev = covariance.headingStdDev();
      }
      estimator.addVisionMeasurement(
          measurement.pose(),
          seconds,
          VecBuilder.fill(covariance.xStdDev(), covariance.yStdDev(), headingStdDev));
    }
  }

  // ---- What it says ----

  @Override
  public Pose2d pose() {
    return estimator.getEstimatedPosition();
  }

  /**
   * WPILib answers for a moment outside its history with the nearest pose it has. A guess is not an
   * answer here, so a moment outside the history is empty.
   */
  @Override
  public Optional<Pose2d> poseAt(long timestampNanos) {
    boolean now = timestampNanos >= newestNanos && timestampNanos <= newestNanos + STILL_NOW_NANOS;
    if (now) {
      return Optional.of(pose());
    }
    if (!hasOdometry) {
      return Optional.empty();
    }
    long oldestKeptNanos =
        Math.max(historyStartNanos, newestNanos - Timestamps.fromSeconds(BUFFER_SECONDS));
    if (timestampNanos < oldestKeptNanos || timestampNanos > newestNanos) {
      return Optional.empty();
    }
    return estimator.sampleAt(Timestamps.toSeconds(timestampNanos));
  }

  @Override
  public long timestampNanos() {
    return newestNanos;
  }

  // ---- What WPILib did with the measurements: counts, since it reports nothing for each ----

  /** Measurements it was given since it was made. */
  public int offered() {
    return offered;
  }

  /** Measurements that reached WPILib's estimator and corrected it. */
  public int corrected() {
    return added;
  }

  /** Corrections WPILib discarded because an older measurement arrived after them. */
  public int discardedLater() {
    return discardedLater;
  }

  /** Measurements WPILib dropped as older than its odometry history. */
  public int tooOld() {
    return tooOld;
  }

  // ---- The yaw and the wheels, as WPILib takes them ----

  /** A wheel reading as WPILib takes it. A wheel that didn't report stays where it last was. */
  private SwerveModulePosition[] positions(OdometrySample sample) {
    SwerveModulePosition[] read = sample.toSwerveModulePositions();
    for (int i = 0; i < read.length; i++) {
      if (!sample.reported(i)) {
        read[i] = i < wheels.length ? wheels[i] : new SwerveModulePosition();
      }
    }
    return read;
  }

  /**
   * The yaw at one wheel reading of a loop's readings: the gyro's while it has readings, the backup
   * IMU's while only that has, and the wheels' own turning with neither. It carries on from where
   * it was when the source changes; a jump in one IMU's own reading passes through.
   */
  private Rotation2d yawAt(
      OdometrySample sample, SwerveModulePosition[] read, DrivetrainReadings loop) {
    Turning source = Turning.WHEELS;
    double reading = 0;
    if (hasYaws(loop.gyro())) {
      source = Turning.GYRO;
      reading = yawAt(loop.gyro().yaws(), sample.timestampNanos());
    } else if (hasYaws(loop.backupImu())) {
      source = Turning.BACKUP;
      reading = yawAt(loop.backupImu().yaws(), sample.timestampNanos());
    }
    boolean sameImuAsBefore = source != Turning.WHEELS && source == turnedBy;
    if (sameImuAsBefore) {
      yawRadians += reading - lastYawReading;
    } else if (turnedBy != Turning.NOTHING_YET) {
      // With no IMU, or in the step one takes over from another, the wheels say the turn.
      yawRadians += kinematics.toTwist2d(wheels, read).dtheta;
    }
    turnedBy = source;
    lastYawReading = reading;
    return new Rotation2d(yawRadians);
  }

  private static boolean hasYaws(ImuReadings imu) {
    return imu.usable() && imu.yaws().count() > 0;
  }

  /** An IMU's yaw at a moment, between its readings either side; its nearest outside them. */
  private static double yawAt(YawSamples yaws, long nanos) {
    int count = yaws.count();
    if (nanos <= yaws.timeNanos(0)) {
      return yaws.yawRadians(0);
    }
    for (int i = 1; i < count; i++) {
      if (nanos <= yaws.timeNanos(i)) {
        long span = yaws.timeNanos(i) - yaws.timeNanos(i - 1);
        double along = span == 0 ? 1 : (double) (nanos - yaws.timeNanos(i - 1)) / span;
        return yaws.yawRadians(i - 1) + along * (yaws.yawRadians(i) - yaws.yawRadians(i - 1));
      }
    }
    return yaws.yawRadians(count - 1);
  }

  /**
   * Whether WPILib's estimator will correct by a measurement taken then, keeping count of what it
   * does with it: it drops one older than its odometry history, and a measurement older than
   * corrections it already made discards those corrections.
   */
  private boolean willCorrect(double seconds) {
    if (!hasOdometry) {
      return false;
    }
    double newestSeconds = Timestamps.toSeconds(newestNanos);
    if (newestSeconds - BUFFER_SECONDS > seconds) {
      tooOld++;
      return false;
    }
    int later = corrections.tailSet(seconds, false).size();
    discardedLater += later;
    corrections.tailSet(seconds, false).clear();
    corrections.add(seconds);
    corrections.headSet(newestSeconds - 2 * BUFFER_SECONDS).clear();
    added++;
    return true;
  }
}
