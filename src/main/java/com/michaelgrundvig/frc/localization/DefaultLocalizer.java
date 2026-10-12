package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The localizer a robot gets unless its configuration chooses another: this library's own {@link
 * PoseEstimator}, with everything it needs around it. It picks which IMU turns odometry, weighs the
 * wheels by what the IMU says of them (a tilt, a jolt, the wheels sliding: {@link SlipCheck}), and
 * reports every option a {@link Localizer} has.
 *
 * <p>Pure, like the rest of this package: it reads no clock, keeps no static state, and knows
 * nothing of the robot program, the simulator, or Pitwall. Use it from one thread.
 *
 * <p>Independent of the drivetrain: its wheels come in as {@link OdometrySample}s, through its
 * {@link DriveKinematicsModel}, so swerve, tank, or any other localizes the same way.
 */
public final class DefaultLocalizer implements Localizer {
  /**
   * On known raised ground, a tilt up to this much steeper than its slopes is still the ground's:
   * bumpers, suspension, and the IMU's mount.
   */
  private static final double GROUND_SLOPE_MARGIN = Math.toRadians(3);

  /** How far off a surface's footprint the estimate may put the robot and still be on it. */
  private static final double MAX_GROUND_REACH_METERS = 1.0;

  private final PoseEstimator estimator;
  private final Terrain ground;

  /** How far the wheels reach from the robot's center, in meters. */
  private final double wheelReach;

  /** Each loop's wheel readings as the estimator's steps, with the yaw at each. */
  private final OdometrySteps odometrySteps;

  /** The wheels against the IMU: whether they slide over the floor together. */
  private final WheelsAgainstImu wheelsAgainstImu;

  /**
   * Which IMU turns odometry this loop, and so says everything else an IMU says this loop: the
   * gyro, the backup, or neither ({@link YawSource#WHEELS}).
   */
  private YawSource imuThisLoop = YawSource.WHEELS;

  /** What that IMU read this loop: null when neither IMU turns odometry. */
  private @Nullable ImuReadings imuReadingsThisLoop;

  /** What the IMU and the robot's state said about trusting the wheels this loop. */
  private DriveConditions conditions = DriveConditions.NORMAL;

  private LocalizationEstimate estimate;

  /** What the estimator decided about each measurement in the last update. */
  private List<VisionDecision> decisions = List.of();

  /** The robot's cameras, by name, in the order of its definition. */
  private final List<String> cameras = new ArrayList<>();

  /** What the robot last said of each camera's mount, by the camera's name. */
  private final Map<String, MountDoubt> mountDoubts = new HashMap<>();

  /**
   * The default localizer for a robot on a field: its estimator starts with the robot not placed
   * yet, so anywhere on the field, and the first camera measurement (or placing it) wins.
   *
   * @param field the field: its tags, whose placement errors the estimator keeps apart; its
   *     terrain, which a tilt the ground explains comes from; and its edges, past which no
   *     measurement can put the robot
   * @param robot what the robot is: its wheels, its IMUs, and its cameras
   * @param kinematics how the drivetrain's wheels move the robot: {@link SwerveKinematicsModel} for
   *     the robot's swerve modules
   * @param config how the estimator weighs its sources
   */
  public DefaultLocalizer(
      FieldModel field,
      RobotDefinition robot,
      DriveKinematicsModel kinematics,
      EstimatorConfig config) {
    this(
        new PoseEstimator(config, field::contains, field.tags().keySet(), notPlacedYet(field)),
        kinematics,
        field.terrain(),
        robot);
  }

  /**
   * The default localizer from its parts: an estimator already made (in a test, one that starts
   * where the robot stands), and a terrain other than the field's own.
   *
   * @param estimator the pose estimator, which this owns from now on
   * @param kinematics how the drivetrain's wheels move the robot
   * @param terrain the ground the robot drives on: a tilt it explains keeps the wheels trusted
   * @param robot what the robot is: its wheels, its IMUs, and its cameras
   */
  public DefaultLocalizer(
      PoseEstimator estimator,
      DriveKinematicsModel kinematics,
      Terrain terrain,
      RobotDefinition robot) {
    this.estimator = estimator;
    odometrySteps = new OdometrySteps(kinematics);
    double reach = 0;
    for (Translation2d wheel : robot.moduleTranslations()) {
      reach = Math.max(reach, wheel.getNorm());
    }
    wheelReach = reach;
    ground = terrain;
    // The most the tread's grip can change the robot's velocity is its friction times gravity.
    double gripMetersPerSecSq =
        robot.drivetrain().wheelCoefficientOfFriction() * ImuReadings.STANDARD_GRAVITY;
    wheelsAgainstImu =
        new WheelsAgainstImu(
            new SlipCheck(SlipCheck.Settings.DEFAULTS, reach, gripMetersPerSecSq), robot.imus());
    for (CameraConfig camera : robot.cameras()) {
      cameras.add(camera.name());
    }
    estimate = estimator.estimate();
  }

  /**
   * How a robot's configuration asks for this localizer with its own weights: a maker of one for a
   * swerve robot, its estimator weighing its sources as the configuration says.
   */
  public static LocalizerMaker with(EstimatorConfig config) {
    return (robot, field) ->
        new DefaultLocalizer(
            field, robot, new SwerveKinematicsModel(robot.moduleTranslations()), config);
  }

  /**
   * What this localizer doubts of a robot, said as it starts: treads said to grip at or past the
   * jolt the slip check calls a whole collision, where it can't tell a slide from a collision.
   */
  public static List<Finding> doubts(RobotDefinition robot) {
    double grip = robot.drivetrain().wheelCoefficientOfFriction();
    double most = SlipCheck.Settings.DEFAULTS.joltMetersPerSecSq() / ImuReadings.STANDARD_GRAVITY;
    if (grip < most) {
      return List.of();
    }
    return List.of(
        Finding.doubt(
            robot.name(),
            String.format(
                Locale.ROOT,
                "the treads' grip, %.3g, is past %.3g, above which the slip check can't tell a"
                    + " slide from a collision: a jolt up to %.3g g may be the floor's, so it's"
                    + " none, and a hit under it is seen only as a slide. Give the tread's own,"
                    + " about 1.0 worn and 1.3 new, from a pull test (docs/characterization.md,"
                    + " tread friction); a path planner's friction is a tuning value, not the"
                    + " tread's",
                grip,
                most,
                grip)));
  }

  /**
   * Until the robot is placed it could be anywhere: the middle of the field stands in, with an
   * uncertainty as big as the field.
   */
  private static PoseReset notPlacedYet(FieldModel field) {
    return new PoseReset(
        0,
        new Pose2d(field.length() / 2, field.width() / 2, Rotation2d.ZERO),
        PoseCovariance.fromStdDevs(field.length(), field.width(), Math.PI),
        "not placed yet");
  }

  // ---- What it is told ----

  /**
   * Picks what turns odometry (the gyro, the backup IMU while the gyro is lost, or the wheels),
   * weighs the wheels by what the IMU says (a tilt, a jolt, the wheels sliding: {@link SlipCheck}),
   * and feeds every wheel reading to the estimator.
   */
  @Override
  public void odometry(DrivetrainReadings readings, LoopContext loop) {
    ImuReadings gyro = readings.gyro();
    ImuReadings backup = readings.backupImu();
    OdometrySteps.Loop steps = odometrySteps.take(readings.wheels(), gyro, backup);
    // One IMU says everything this loop: the one whose yaw turns odometry.
    imuThisLoop = steps.yawFrom();
    imuReadingsThisLoop =
        switch (imuThisLoop) {
          case GYRO -> gyro;
          case BACKUP -> backup;
          case WHEELS -> null;
        };
    wheelsAgainstImu.compare(steps, imuReadingsThisLoop, readings.velocities());
    conditions = conditions(loop.enabled());
    estimator.setConditions(conditions);
    for (OdometrySteps.Step step : steps.steps()) {
      estimator.addOdometry(
          step.timestampNanos(), step.motion(), step.yawRadians(), steps.yawFrom());
    }
    estimate = estimator.estimate();
  }

  /** The estimator weighs the placement like any measurement, by how sure the teller is. */
  @Override
  public void place(PoseReset placed) {
    estimator.reset(placed);
    estimate = estimator.estimate();
  }

  /**
   * The position stays, with its uncertainty. If the heading is wrong, the cameras' headings will
   * agree against it and the estimate turns back to them.
   */
  @Override
  public void resetHeading(Rotation2d heading, double stdDevRadians) {
    estimator.resetHeading(heading, stdDevRadians);
    estimate = estimator.estimate();
  }

  /**
   * With {@link EstimatorConfig.HeadingCorrection#HEADING_MEASUREMENTS_AND_MOUNT_DOUBT} the
   * estimator takes the doubt as part of the camera's repeated errors.
   */
  @Override
  public void mountDoubt(String camera, double doubtMeters, double distanceMeters) {
    estimator.mountDoubt(camera, doubtMeters, distanceMeters);
    mountDoubts.put(camera, new MountDoubt(doubtMeters, distanceMeters));
  }

  /** Applies the loop's measurements, each at its capture time. */
  @Override
  public void measurements(List<VisionMeasurement> taken) {
    for (VisionMeasurement measurement : taken) {
      estimator.addVision(measurement);
    }
    decisions = List.copyOf(estimator.update());
    estimate = estimator.estimate();
  }

  // ---- What it says ----

  @Override
  public Pose2d pose() {
    return estimator.pose();
  }

  @Override
  public Optional<Pose2d> poseAt(long timestampNanos) {
    return estimator.poseAt(timestampNanos);
  }

  @Override
  public long timestampNanos() {
    return estimate.timestampNanos();
  }

  /**
   * Empty while the heading is a driver's reset no camera has backed. A heading in doubt solves no
   * image, however sure the driver's reset says it is. Solved with it, an image lands off by its
   * error, and only an image solved without it can back it.
   */
  @Override
  public Optional<Rotation2d> headingAt(long timestampNanos) {
    if (estimator.headingInDoubt()) {
      return Optional.empty();
    }
    return estimator.poseAt(timestampNanos).map(Pose2d::getRotation);
  }

  /** The estimate whole, with its uncertainty, mode, and health, as of the last step. */
  public LocalizationEstimate estimate() {
    return estimate;
  }

  @Override
  public Optional<PoseCovariance> covariance() {
    return Optional.of(estimate.covariance());
  }

  @Override
  public Optional<LocalizationEstimate.Mode> mode() {
    return Optional.of(estimate.mode());
  }

  @Override
  public Optional<LocalizationEstimate.Health> health() {
    return Optional.of(estimate.health());
  }

  /** In the order they were applied: those that couldn't be, then the rest by capture time. */
  @Override
  public Optional<List<VisionDecision>> decisions() {
    return Optional.of(decisions);
  }

  @Override
  public Optional<ChassisVelocities> fieldVelocities() {
    Translation2d velocity = estimate.fieldVelocity();
    return Optional.of(
        new ChassisVelocities(
            velocity.getX(), velocity.getY(), estimate.yawRateRadiansPerSecond()));
  }

  @Override
  public Optional<WheelTrust> wheels() {
    return Optional.of(wheelTrust());
  }

  /**
   * Each camera the estimator has heard from, in the order it first did, then the robot's others;
   * and the wheels.
   */
  @Override
  public Optional<Trust> trust() {
    Map<String, CameraTrust> trust = new LinkedHashMap<>();
    List<String> beyond = estimator.camerasWithoutPlaces();
    for (String camera : estimator.cameras()) {
      trust.put(camera, trust(camera, !beyond.contains(camera)));
    }
    for (String camera : cameras) {
      trust.putIfAbsent(camera, trust(camera, true));
    }
    for (String camera : mountDoubts.keySet()) {
      trust.putIfAbsent(camera, trust(camera, true));
    }
    return Optional.of(
        Trust.builder()
            .cameras(trust)
            .wheels(wheelTrust())
            .tagsWithoutPlaces(new LinkedHashSet<>(estimator.tagsWithoutPlaces()))
            .build());
  }

  /** What the yaws came from in the last loop that had wheel readings. */
  @Override
  public Optional<YawSource> yawSource() {
    return Optional.of(odometrySteps.turnsOdometryNow());
  }

  /** What the IMU said about trusting the wheels this loop. */
  public DriveConditions conditions() {
    return conditions;
  }

  /** How far the wheels are trusted this loop, and why. */
  private WheelTrust wheelTrust() {
    SlipCheck slipCheck = wheelsAgainstImu.slipCheck();
    return WheelTrust.builder()
        .imu(imuThisLoop)
        .conditions(conditions)
        .turnSlipMetersPerSecond(slipCheck.turnSlipMetersPerSecond())
        .speedSlipMetersPerSecond(slipCheck.speedSlipMetersPerSecond())
        .turnApartRadians(slipCheck.turnApartRadians())
        .slideMetersPerSecond(slipCheck.slideMetersPerSecond())
        .unexplainedMetersPerSecSq(slipCheck.unexplainedMetersPerSecSq())
        .accelerometerBiasMetersPerSecSq(slipCheck.biasMetersPerSecSq())
        .accelerometerLeaksGravity(slipCheck.leaksGravity())
        .build();
  }

  /** One camera's trust: whether its errors are kept apart, and its mount's doubt if it has one. */
  private CameraTrust trust(String camera, boolean keptApart) {
    CameraTrustBuilder trust = CameraTrust.builder().keptApart(keptApart);
    MountDoubt doubt = mountDoubts.get(camera);
    if (doubt == null) {
      return trust.build();
    }
    return trust
        .mountDoubtMeters(doubt.meters())
        .mountDistanceMeters(doubt.tagDistanceMeters())
        .build();
  }

  /**
   * What the IMU says about trusting the wheels: the robot's tilt, a jolt beyond driving, the
   * wheels sliding over the floor, and whether people might be carrying it. The IMU that turns
   * odometry says (the gyro, or the backup while the gyro is lost). Without one, nothing can say
   * the robot is tilted.
   */
  private DriveConditions conditions(boolean enabled) {
    ImuReadings imu = imuReadingsThisLoop;
    double tilt = 0;
    double groundSlope = 0;
    if (imu != null) {
      tilt = WheelsAgainstImu.tiltFromLevel(imu);
      groundSlope = groundSlope(tilt);
    }
    SlipCheck slipCheck = wheelsAgainstImu.slipCheck();
    return new DriveConditions(
        tilt, groundSlope, slipCheck.collisionShare(), slipCheck.slipMetersPerSecond(), !enabled);
  }

  /**
   * How much of a tilt the ground explains. On known raised ground it is the tilt itself, up to the
   * ground's steepest slope and a margin. On flat ground it is none.
   *
   * <p>The robot counts as on raised ground when the estimate puts it within reach of a surface's
   * footprint. That reach is the wheels' own reach from the robot's center plus twice the
   * estimate's position spread, since the estimate may be off by that much, and never more than
   * {@link #MAX_GROUND_REACH_METERS}.
   */
  private double groundSlope(double tilt) {
    if (ground.surfaces().isEmpty() || tilt == 0) {
      return 0;
    }
    // The estimate as it was last worked out is still how the estimator stands. Everything that
    // changes it (odometry, a placement, the update that applies the cameras) works it out again
    // as it does, and none of those has run since. So the estimate isn't worked out again here,
    // which would be a second full estimate every loop the robot is tilted.
    double estimateMayBeOffBy = 2 * estimate.covariance().largestPositionStdDev();
    double reach = Math.min(MAX_GROUND_REACH_METERS, wheelReach + estimateMayBeOffBy);
    return ground
        .under(pose().getTranslation(), reach)
        .map(surface -> Math.min(tilt, surface.steepestSlopeRadians() + GROUND_SLOPE_MARGIN))
        .orElse(0.0);
  }
}
