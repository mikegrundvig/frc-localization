package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Where the robot is: localization as one service, told what the robot's sensors read and asked
 * what it makes of them. It owns the robot's {@link Localizer}, which keeps the pose, and what
 * stays in front of every localizer: solving each camera image, as the robot was when it was taken,
 * into the measurement the localizer is given.
 *
 * <p>Each loop, on the robot loop's thread, in this order:
 *
 * <ol>
 *   <li>{@link #drivetrain}, with every wheel reading since the last loop and both IMUs;
 *   <li>{@link #place}, if the drive team placed the robot this loop;
 *   <li>{@link #mountDoubt} for each camera, and {@link #camera} for each image that arrived,
 *       stamped when it was taken;
 *   <li>{@link #update}: the one call that hands the localizer the loop's measurements, and says
 *       what became of each.
 * </ol>
 *
 * <p>Pure, like the rest of this package: it reads no clock (each loop's time comes in a {@link
 * LoopContext}), keeps no static state, and knows nothing of the robot program, the simulator, or
 * Pitwall. Its outputs are a function of its inputs and its configuration, so a replay of a logged
 * match reproduces every one. Use it from one thread.
 *
 * <p>What a localizer does not report ({@link Localizer}'s options) this does not make up: the
 * answer is empty, and whoever asked says "not reported". Two things an image is solved with have a
 * measured stand-in instead. Without {@link Localizer#fieldVelocities} the robot's speed is the
 * wheels' own and its turning the IMU's. Without {@link Localizer#covariance} nothing says the
 * heading is sure, so no image is solved with it.
 *
 * <p>Each image is solved against the field's tags through one {@link VisionPipeline}, and with its
 * camera's mount from the robot's definition, looked up by name in {@link #solve}: the two places a
 * venue's surveyed tags and a camera that rides a joint would change what an image is solved with.
 */
public final class LocalizationService {
  /** How far back "lately" reaches, for how fast the robot has turned: a top team used 0.3 s. */
  private static final double RECENT_TURNING_SECONDS = 0.3;

  /** How much of the robot's attitude is kept, for each image's capture time. */
  private static final double ATTITUDE_SECONDS = 0.5;

  /** Where the ring of recent yaw rates stands before any loop: the first loop takes place 0. */
  private static final int BEFORE_THE_FIRST_LOOP = -1;

  /** A history of loops is never shorter than this: one loop either side of a moment. */
  private static final int FEWEST_LOOPS_KEPT = 2;

  private final Localizer localizer;
  private final Terrain ground;

  /** The gyro and the backup IMU should turn alike. */
  private final ImuCrossCheck crossCheck = new ImuCrossCheck(ImuCrossCheck.Settings.DEFAULTS);

  /**
   * What the IMU that has readings this loop read: the gyro, the backup IMU while the gyro has
   * none, or null when neither has. The robot's tilt comes from it.
   */
  private @Nullable ImuReadings imuReadingsThisLoop;

  /** How the wheels say the robot moves this loop, for a localizer that does not say itself. */
  private ChassisVelocities wheelVelocities = new ChassisVelocities();

  /** The measurements made since the last update, in the order the cameras were read. */
  private final List<VisionMeasurement> pending = new ArrayList<>();

  /** The measurements the last update handed the localizer, in the order it was handed them. */
  private List<VisionMeasurement> given = List.of();

  // The cameras: each one's mount and the field's tags to solve its images with (the venue's, once
  // one is selected), and how far the pit has tuned them to be trusted.
  private VisionPipeline pipeline;
  private final Set<Integer> tagIds;
  private final Map<String, CameraConfig> cameras = new LinkedHashMap<>();
  private VisionConfig vision = VisionConfig.DEFAULTS;

  /** What the robot last said of each camera's mount, by the camera's name. */
  private final Map<String, MountDoubt> mountDoubts = new HashMap<>();

  /** The images refused or held since the last update, before they became measurements. */
  private final List<FrameVerdict> refused = new ArrayList<>();

  /** Half a second of the robot's attitude, for each image's capture time. */
  private final AttitudeHistory attitudes;

  /** What the robot is, for where a camera on a joint was. */
  private final RobotMath math;

  /** Half a second of the joints' positions, for where a camera on one was at its capture time. */
  private final JointHistory jointPositions;

  /** How fast the robot turned in each of the latest loops, for how fast it has turned lately. */
  private final double[] recentYawRates;

  /**
   * This loop's place among {@link #recentYawRates}, which is used as a ring: {@link
   * #BEFORE_THE_FIRST_LOOP} until a loop has come.
   */
  private int yawRateLoop = BEFORE_THE_FIRST_LOOP;

  /** Whether a loop is under way: {@link #drivetrain} has come, and {@link #update} hasn't yet. */
  private boolean inLoop;

  /**
   * How the robot moves this loop, and how sure its heading is: what this loop's images are solved
   * with. Taken as the loop's readings come in, and again if the robot is placed during the loop.
   */
  private Motion motion;

  /**
   * @param speedMetersPerSecond how fast the robot moves over the field
   * @param recentYawRateRadiansPerSecond the fastest it turned lately, either way
   * @param headingStdDevRadians how sure the localizer is of its heading; infinite when it does not
   *     say
   */
  private record Motion(
      double speedMetersPerSecond,
      double recentYawRateRadiansPerSecond,
      double headingStdDevRadians) {}

  /**
   * Localization for a robot on a field, with the default localizer: its estimator starts with the
   * robot not placed yet, so anywhere on the field, and the first camera measurement (or placing
   * it) wins.
   *
   * @param field the field: its tags, which images are solved against; its terrain, which a tilt
   *     the ground explains and a crest crossed come from; and its edges, past which no measurement
   *     can put the robot
   * @param robot what the robot is: its wheels, its IMUs, and its cameras' mounts
   * @param kinematics how the drivetrain's wheels move the robot: {@link SwerveKinematicsModel} for
   *     the robot's swerve modules
   * @param config how the estimator weighs its sources
   * @param loopSeconds how long one robot loop is
   */
  public LocalizationService(
      FieldModel field,
      RobotDefinition robot,
      DriveKinematicsModel kinematics,
      EstimatorConfig config,
      double loopSeconds) {
    this(field, robot, new DefaultLocalizer(field, robot, kinematics, config), loopSeconds);
  }

  /**
   * Localization with the default localizer from its parts: an estimator already made (in a test,
   * one that starts where the robot stands), and a terrain other than the field's own.
   *
   * @param estimator the pose estimator, which the localizer owns from now on
   * @param kinematics how the drivetrain's wheels move the robot
   * @param field the field whose tags images are solved against
   * @param terrain the ground the robot drives on: a tilt it explains keeps the wheels trusted
   * @param robot what the robot is: its wheels, its IMUs, and its cameras' mounts
   * @param loopSeconds how long one robot loop is
   */
  public LocalizationService(
      PoseEstimator estimator,
      DriveKinematicsModel kinematics,
      FieldModel field,
      Terrain terrain,
      RobotDefinition robot,
      double loopSeconds) {
    this(
        field,
        terrain,
        robot,
        new DefaultLocalizer(estimator, kinematics, terrain, robot),
        loopSeconds);
  }

  /**
   * Localization for a robot on a field, with the localizer the robot chose.
   *
   * @param field the field: its tags, which images are solved against, and its terrain
   * @param robot what the robot is: its cameras' mounts, and the joints a camera rides
   * @param localizer the localizer, which this owns from now on
   * @param loopSeconds how long one robot loop is
   */
  public LocalizationService(
      FieldModel field, RobotDefinition robot, Localizer localizer, double loopSeconds) {
    this(field, field.terrain(), robot, localizer, loopSeconds);
  }

  /**
   * The same, on a terrain other than the field's own.
   *
   * @param terrain the ground the robot drives on, for how high a pose stands
   */
  public LocalizationService(
      FieldModel field,
      Terrain terrain,
      RobotDefinition robot,
      Localizer localizer,
      double loopSeconds) {
    if (!(loopSeconds > 0) || Double.isInfinite(loopSeconds)) {
      throw new IllegalArgumentException("a loop takes some time: " + loopSeconds);
    }
    this.localizer = localizer;
    ground = terrain;
    pipeline = new VisionPipeline(field);
    tagIds = Set.copyOf(field.tags().keySet());
    for (CameraConfig camera : robot.cameras()) {
      cameras.put(camera.name(), camera);
    }
    attitudes = new AttitudeHistory(loops(ATTITUDE_SECONDS, loopSeconds));
    math = robot.math();
    jointPositions = new JointHistory(math.rests(), loops(ATTITUDE_SECONDS, loopSeconds));
    recentYawRates = new double[loops(RECENT_TURNING_SECONDS, loopSeconds)];
    motion = motionNow(0);
  }

  /** The localizer that keeps the pose: what a test or a study asks beyond what this says. */
  public Localizer localizer() {
    return localizer;
  }

  /**
   * Solves images against a venue's tags from now on: the field as surveyed, its tags where the
   * survey found them. The estimate carries on as it was; a venue changes where the tags are, not
   * where the robot is. The caller changes it only while the robot is disabled.
   *
   * @param venue the same field, with the same tags, surveyed
   * @throws IllegalArgumentException if its tags aren't the ones the estimator keeps apart
   */
  public void venue(FieldModel venue) {
    if (!venue.tags().keySet().equals(tagIds)) {
      throw new IllegalArgumentException(
          "a venue has the field's tags, surveyed: " + venue.name() + " has others");
    }
    pipeline = new VisionPipeline(venue);
  }

  /** How many loops a stretch of time takes, to the nearest loop, and at least two. */
  private static int loops(double seconds, double loopSeconds) {
    return Math.max(FEWEST_LOOPS_KEPT, (int) Math.round(seconds / loopSeconds));
  }

  /**
   * One loop of the drivetrain's readings: after it reads its sensors, and before the cameras. The
   * localizer moves the pose along by them.
   */
  public void drivetrain(DrivetrainReadings readings, LoopContext loop) {
    ImuReadings gyro = readings.gyro();
    ImuReadings backup = readings.backupImu();
    localizer.odometry(readings, loop);
    imuReadingsThisLoop = imuWithReadings(gyro, backup);
    wheelVelocities = readings.velocities();
    checkImus(gyro, backup, loop.nowNanos());
    // The loop's images are solved as the robot is now. Its attitude and its turning are recorded
    // once a loop for that.
    attitudes.add(localizer.timestampNanos(), pitchRadians(), rollRadians());
    yawRateLoop = (yawRateLoop + 1) % recentYawRates.length;
    inLoop = true;
    takeMotion();
  }

  /**
   * The IMU that says how the robot is tilted this loop: the gyro while it has yaw readings, the
   * backup IMU while only that has, and null with neither. This is the IMU the default localizer
   * turns odometry by.
   */
  private static @Nullable ImuReadings imuWithReadings(ImuReadings gyro, ImuReadings backup) {
    if (hasYawReadings(gyro)) {
      return gyro;
    }
    if (hasYawReadings(backup)) {
      return backup;
    }
    return null;
  }

  private static boolean hasYawReadings(ImuReadings imu) {
    return imu.usable() && imu.yaws().count() > 0;
  }

  /**
   * The drive team put the robot here (on its start before the match, say), or a driver's "against
   * the wall here". After {@link #drivetrain}, so a turn made in placing it isn't counted twice.
   */
  public void place(PoseReset reset) {
    localizer.place(reset);
    if (inLoop) {
      takeMotion();
    }
  }

  /**
   * The robot faces this heading, as sure as the given spread; its position stays. With the default
   * localizer, if that's wrong, the cameras' headings will agree against it and the estimate turns
   * back to them.
   */
  public void resetHeading(Rotation2d heading, double stdDevRadians) {
    localizer.resetHeading(heading, stdDevRadians);
    if (inLoop) {
      takeMotion();
    }
  }

  /**
   * How far the cameras' images are trusted, from the next image on: as committed, or as the pit
   * tuned it. Until told, {@link VisionConfig#DEFAULTS}.
   */
  public void vision(VisionConfig settings) {
    vision = settings;
  }

  /**
   * How far a camera's mount may have moved since it was calibrated ({@link MountCheck}), as of
   * now: its images from now on are widened by it, and the localizer is told ({@link
   * Localizer#mountDoubt}). Before the cameras' images, each loop.
   *
   * @param doubtMeters how far its measurements may be off from where its mount moved
   * @param distanceMeters how far its tags were, on average, when that was judged; NaN until then
   */
  public void mountDoubt(String camera, double doubtMeters, double distanceMeters) {
    localizer.mountDoubt(camera, doubtMeters, distanceMeters);
    mountDoubts.put(camera, new MountDoubt(doubtMeters, distanceMeters));
  }

  /** One camera image, from every tag it saw: see {@link #camera(TagDetections, TagAllowlist)}. */
  public VisionResult camera(TagDetections frame) {
    return camera(frame, null);
  }

  /**
   * One camera image, stamped when it was taken: solved as the robot was then ({@link #capture}),
   * widened by its camera's mount's doubt ({@link #mountDoubt}), and its measurement, if it makes
   * one, applied at that moment on the next {@link #update}.
   *
   * @param onlyTags the tags the current task trusts ({@link TagAllowlist}); null for every tag
   * @return what the image made: a measurement, or why not. The next update says it too, among its
   *     frames, as it does what became of the measurement
   * @throws IllegalArgumentException if the robot's definition has no camera by the frame's name
   */
  public VisionResult camera(TagDetections frame, @Nullable TagAllowlist onlyTags) {
    VisionResult result = solve(frame, onlyTags);
    VisionMeasurement measurement = result.measurement();
    if (measurement != null) {
      camera(measurement);
    } else {
      refused.add(FrameVerdict.refused(result));
    }
    return result;
  }

  /**
   * An image the robot holds back, from a camera being calibrated or while autonomous starts:
   * solved as {@link #camera(TagDetections, TagAllowlist)} would, never applied, and among the next
   * update's frames with why it was held, or why the pipeline refused it.
   *
   * @param why why it's held: {@link Verdict#CALIBRATING} or {@link Verdict#HELD}
   * @return what the image made: a measurement, or why not, for the log
   */
  public VisionResult hold(TagDetections frame, @Nullable TagAllowlist onlyTags, Verdict why) {
    if (why != Verdict.CALIBRATING && why != Verdict.HELD) {
      throw new IllegalArgumentException(
          "the robot holds an image back for its own reasons: " + why);
    }
    VisionResult result = solve(frame, onlyTags);
    boolean pipelineRefusedIt = result.measurement() == null;
    if (pipelineRefusedIt) {
      refused.add(FrameVerdict.refused(result));
    } else {
      refused.add(FrameVerdict.held(frame, why));
    }
    return result;
  }

  /** What an image makes, solved as the robot was when it was taken: nothing is kept. */
  private VisionResult solve(TagDetections frame, @Nullable TagAllowlist onlyTags) {
    CameraConfig camera = cameras.get(frame.camera());
    if (camera == null) {
      throw new IllegalArgumentException(
          "the robot's definition has no camera named " + frame.camera());
    }
    MountDoubt doubt = mountDoubts.get(frame.camera());
    double mountDoubtMeters = doubt == null ? 0 : doubt.meters();
    return pipeline.process(
        frame,
        camera,
        mount(camera, frame.timestampNanos()),
        capture(frame.timestampNanos()),
        vision,
        new VisionPipeline.Trust(mountDoubtMeters, onlyTags));
  }

  /**
   * A camera measurement solved elsewhere (a Limelight's own pose, say, or a test's), applied at
   * its capture time on the next {@link #update}.
   */
  public void camera(VisionMeasurement measurement) {
    pending.add(measurement);
  }

  /**
   * The joints' positions as the robot read them, each loop: where a camera on a joint is when its
   * image was taken. A robot whose cameras are all on the chassis needs none; before the first,
   * each joint is at its reference.
   *
   * @param timestampNanos when they were read, on the robot's clock
   * @param positions each driven joint's position, in its units; a joint not named stays where it
   *     was
   */
  public void joints(long timestampNanos, Map<Joint, Double> positions) {
    jointPositions.add(timestampNanos, positions);
  }

  /** Where a camera was on the robot at a moment: on a joint, where the joint had carried it. */
  private Transform3d mount(CameraConfig camera, long timestampNanos) {
    if (camera.joint().isEmpty()) {
      return camera.robotToCamera();
    }
    Pose3d pose = math.cameraPose(camera, jointPositions.at(timestampNanos));
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * The robot when an image was taken, as the vision pipeline solves it: its heading then, if the
   * localizer has one to solve with ({@link Localizer#headingAt}), its tilt then, and how it moves
   * this loop. Read-only: asking changes nothing, however often or early in the loop it's asked.
   */
  public CaptureContext capture(long timestampNanos) {
    Motion now = motion;
    AttitudeHistory.Attitude tilted = attitudes.at(timestampNanos);
    return new CaptureContext(
        // The pipeline takes null for "solve it without a heading".
        localizer.headingAt(timestampNanos).orElse(null),
        now.headingStdDevRadians(),
        tilted.pitchRadians(),
        tilted.rollRadians(),
        now.speedMetersPerSecond(),
        now.recentYawRateRadiansPerSecond());
  }

  /**
   * Hands the localizer the loop's measurements, and says what became of each. After the cameras;
   * once a loop, closing the loop {@link #drivetrain} opened.
   */
  public LocalizationOutput update() {
    inLoop = false;
    given = List.copyOf(pending);
    pending.clear();
    localizer.measurements(given);
    Optional<List<VisionDecision>> decisions = localizer.decisions();
    List<FrameVerdict> frames = new ArrayList<>(refused.size() + given.size());
    frames.addAll(refused);
    refused.clear();
    if (decisions.isPresent()) {
      for (VisionDecision decision : decisions.get()) {
        frames.add(FrameVerdict.of(decision));
      }
    } else {
      for (VisionMeasurement measurement : given) {
        frames.add(FrameVerdict.given(measurement));
      }
    }
    return LocalizationOutput.builder()
        .report(report())
        .trust(localizer.trust())
        .frames(frames)
        .build();
  }

  /**
   * Every measurement the last {@link #update} handed the localizer, each whole (its pose, its
   * uncertainty, its tags), in the order it was handed them: for a study that hands another
   * localizer the same measurements.
   */
  public List<VisionMeasurement> given() {
    return given;
  }

  /** Where the robot is: cheap, for driving every loop. */
  public Pose2d pose() {
    return localizer.pose();
  }

  /** Where the robot was at a moment, if the localizer reaches back that far. */
  public Optional<Pose2d> poseAt(long timestampNanos) {
    return localizer.poseAt(timestampNanos);
  }

  /** The pose, with what the localizer reports about it, as of the last step. */
  public LocalizerReport report() {
    return LocalizerReport.of(localizer);
  }

  /** The time of the newest wheel reading the pose includes. */
  public long timestampNanos() {
    return localizer.timestampNanos();
  }

  /**
   * How fast the robot moves over the field, in meters a second: as the localizer says, or as the
   * wheels measured it this loop where the localizer does not.
   */
  public double speedMetersPerSecond() {
    Optional<ChassisVelocities> moving = localizer.fieldVelocities();
    ChassisVelocities velocities = moving.orElse(wheelVelocities);
    return Math.hypot(velocities.vx, velocities.vy);
  }

  /**
   * How fast the robot turns, counterclockwise, in radians a second: as the localizer says. Where
   * it does not, as the IMU read it this loop, or the wheels with no IMU.
   */
  public double yawRateRadiansPerSecond() {
    Optional<ChassisVelocities> moving = localizer.fieldVelocities();
    if (moving.isPresent()) {
      return moving.get().omega;
    }
    ImuReadings imu = imuReadingsThisLoop;
    if (imu != null) {
      return imu.yawRateRadiansPerSecond();
    }
    return wheelVelocities.omega;
  }

  /** A pose on the field in 3D: as high as the ground under it, tilted as the IMU says now. */
  public Pose3d pose3d(Pose2d pose) {
    return new Pose3d(
        pose.getX(),
        pose.getY(),
        ground.heightAt(pose.getTranslation()),
        new Rotation3d(rollRadians(), pitchRadians(), pose.getRotation().getRadians()));
  }

  /** The robot's pitch, from the IMU: zero without one. Positive is nose down. */
  public double pitchRadians() {
    ImuReadings imu = imuReadingsThisLoop;
    return imu == null ? 0 : imu.pitchRadians();
  }

  /** The robot's roll, from the IMU: zero without one. */
  public double rollRadians() {
    ImuReadings imu = imuReadingsThisLoop;
    return imu == null ? 0 : imu.rollRadians();
  }

  /** How far the gyro and the backup IMU have turned apart lately, while both report. */
  public double imuDisagreementRadians() {
    return crossCheck.disagreementRadians();
  }

  /**
   * Whether the gyro and the backup IMU disagree for long: one is failing, loose, or needs its
   * scale calibrated.
   */
  public boolean imusDisagree() {
    return crossCheck.disagrees();
  }

  /**
   * This loop's images' motion, as the localizer has it now: how fast the robot moves, the fastest
   * it turned lately (this loop's turning in its place among the latest loops'), and how sure its
   * heading is. Taken as the loop's readings come in, and again when a placement moves the estimate
   * before the loop's images.
   */
  private void takeMotion() {
    recentYawRates[yawRateLoop] = Math.abs(yawRateRadiansPerSecond());
    double recentYawRate = 0;
    for (double rate : recentYawRates) {
      recentYawRate = Math.max(recentYawRate, rate);
    }
    motion = motionNow(recentYawRate);
  }

  /** The robot's motion now, with the fastest it turned lately as given. */
  private Motion motionNow(double recentYawRate) {
    // A localizer that does not say how sure its heading is has no image solved with it.
    double headingStdDev =
        localizer.covariance().map(PoseCovariance::headingStdDev).orElse(Double.POSITIVE_INFINITY);
    return new Motion(speedMetersPerSecond(), recentYawRate, headingStdDev);
  }

  /**
   * While both report, compares the gyro with the backup IMU: a sustained disagreement means one is
   * failing, loose, or needs its scale calibrated. It raises the question rather than silently
   * picking one.
   */
  private void checkImus(ImuReadings gyro, ImuReadings backup, long nowNanos) {
    if (gyro.usable() && backup.usable()) {
      crossCheck.update(nowNanos, gyro.yawRadians(), backup.yawRadians());
    } else {
      crossCheck.reset();
    }
  }
}
