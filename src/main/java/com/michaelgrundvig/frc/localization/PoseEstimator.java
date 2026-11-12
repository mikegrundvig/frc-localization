package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Where the robot is: odometry, corrected by camera measurements, with the gyro owning heading.
 *
 * <p>The state is the robot's position, a heading offset, the gyro's two errors (its scale and its
 * drift), and each camera's turn on its mount. The field heading is the gyro's reading plus the
 * offset, so the gyro keeps all short-term authority over heading, and cameras correct what it gets
 * wrong: the share of every turn its scale misreads, and what it drifts each second. Each camera's
 * turn starts as unsure as its mount's tolerance and is learned from its images relative to the
 * other cameras' ({@link PoseFilter}). A camera whose headings turn at once, as a knocked one's do,
 * has its turn opened up and learned anew ({@link CameraMounts}). A covariance holds the
 * uncertainty of all of them and how they relate: an unsure heading makes every meter driven add
 * position uncertainty sideways, an unsure scale makes every turn add heading uncertainty, and an
 * unsure drift every second.
 *
 * <p><b>Odometry.</b> Each odometry step predicts ({@link OdometryPrediction}): the wheels' motion,
 * along the arc the gyro turned through, moves the position, and the uncertainty grows with the
 * distance driven, the turning, and time, and with whatever makes the wheels suspect ({@link
 * WheelDoubt}).
 *
 * <p><b>Cameras.</b> A camera measurement is applied at the moment its image was taken, however
 * late it arrives ({@link VisionReplay}). Each is gated against the estimate first ({@link
 * VisionGate}), and again whenever a replay applies it anew: one that a later-arriving, earlier
 * image rules out is taken back, as it would have been rejected had the two come in order. What a
 * camera is known to miss by is taken off first ({@link CameraMissCorrection}).
 *
 * <p><b>Recovery.</b> When several rejected measurements agree with each other, for reasons
 * independent of each other, odometry was the one that was wrong (a collision, a spin), and the
 * estimate recovers to them ({@link Recovery}). Independent means two cameras, two tags, or a
 * camera and a sign from the other sensors that the robot may have been moved unseen ({@link
 * MotionEvidence}): one camera repeating one tag's answer is one source, and a misread tag repeats
 * itself as faithfully as a shove would. It recovers to their heading first, if that is what they
 * agree is wrong, since every position worked out with a wrong heading is wrong too.
 *
 * <p><b>Honesty.</b> Its uncertainty is meant to be honest, since gates, χ² tests, and any bound on
 * the error trust it: the error should go past 3.72 times the spread it claims no more than once in
 * a thousand loops, which the consistency study checks in the simulator (docs/testing.md). So
 * errors that repeat are never averaged away as if each were new ({@link PoseFilter} says which,
 * and how). Each camera is told apart by its name, and each tag by its number ({@link
 * ErrorPlaces}).
 *
 * <p>This class feeds its parts and keeps the books ({@link MeasurementBooks}). It is pure logic:
 * no clock, no hardware, no static state. Every input comes in through a method, stamped with its
 * capture time, so a replay reproduces every decision. Use it from one thread.
 */
public final class PoseEstimator {
  /** The robot's velocity is taken over the odometry steps of this last stretch of time. */
  private static final long VELOCITY_WINDOW_NANOS = 100_000_000L;

  private static final double SECONDS_PER_MINUTE = 60;
  private static final double PERCENT = 100;

  private final EstimatorConfig config;

  private final long historyNanos;
  private final long futureToleranceNanos;
  private final long visionTimeoutNanos;
  private final long disagreementWindowNanos;

  private final OdometryHistory history;
  private final PoseFilter filter;
  private final OdometryPrediction prediction;
  private final WheelDoubt wheelDoubt;
  private final ErrorPlaces places;
  private final CameraMissCorrection corrections;
  private final CameraMounts mounts;
  private final VisionReplay replay;
  private final VisionUpdate vision;

  /** What besides the cameras says the robot may have been moved unseen. */
  private final MotionEvidence motionEvidence = new MotionEvidence();

  /** Reads the gyro's drift off each stretch of standing still. */
  private final GyroAtRest gyroAtRest;

  /** Where the wheels and the IMU alone put the robot since it was last placed. */
  private final OdometryOnly odometryOnly;

  /** The IMUs' readings, unwrapped into one continuous yaw. */
  private final ContinuousYaw yaw = new ContinuousYaw();

  /** Whether the first odometry step has come: nothing can be estimated before it. */
  private boolean started;

  /** Where the robot is taken to be when odometry starts: as constructed, or as last reset. */
  private PoseReset initial;

  /** How many times the drive team has reset the heading. */
  private int headingResets;

  /**
   * An estimator that takes each camera, as it first gives a measurement, to sit as a mount
   * measured from CAD and checked does ({@link CameraConfig.MountTolerance#MEASURED}). The
   * parameters are as for the constructor that names the cameras.
   */
  public PoseEstimator(
      EstimatorConfig config,
      Predicate<Translation2d> onField,
      Collection<Integer> tags,
      PoseReset initial) {
    this(config, onField, tags, List.of(), initial);
  }

  /**
   * @param config how to weigh the sources
   * @param onField whether a point is on the field: measurements off it are rejected
   * @param tags the field's tags: each one's placement is kept apart, the same for every camera
   *     that sees it and every time it's seen. A measurement from any other counts its placement as
   *     new in each image ({@link #tagsWithoutPlaces})
   * @param cameras the robot's cameras, each known by its name: its turn on its mount starts as
   *     unsure as its mount's tolerance says. A camera not among them that gives a measurement is
   *     taken to sit as a measured mount does ({@link CameraConfig.MountTolerance#MEASURED})
   * @param initial where the robot is at the start, and how sure that is; for a placeholder (the
   *     robot not placed yet) give a large uncertainty, and the first camera measurement wins
   */
  public PoseEstimator(
      EstimatorConfig config,
      Predicate<Translation2d> onField,
      Collection<Integer> tags,
      List<CameraConfig> cameras,
      PoseReset initial) {
    this(config, HeadingCorrection.EVERY_FIX, onField, tags, cameras, initial);
  }

  /**
   * An estimator that corrects the heading another way than the default localizer does: for the
   * consistency study, which compares them.
   *
   * @param heading what corrects the heading beside the gyro
   */
  public PoseEstimator(
      EstimatorConfig config,
      HeadingCorrection heading,
      Predicate<Translation2d> onField,
      Collection<Integer> tags,
      List<CameraConfig> cameras,
      PoseReset initial) {
    this.config = config;
    this.initial = initial;
    odometryOnly = new OdometryOnly(initial.pose());
    historyNanos = Timestamps.fromSeconds(config.historySeconds());
    futureToleranceNanos = Timestamps.fromSeconds(config.futureToleranceSeconds());
    visionTimeoutNanos = Timestamps.fromSeconds(config.visionTimeoutSeconds());
    disagreementWindowNanos = Timestamps.fromSeconds(config.disagreementWindowSeconds());
    wheelDoubt = new WheelDoubt(config);
    places = new ErrorPlaces(tags);
    gyroAtRest =
        new GyroAtRest(
            OdometryPrediction.headingVariancePerSecond(config),
            Math.toRadians(config.gyroDriftStdDevDegreesPerMinute()) / SECONDS_PER_MINUTE);
    filter = EstimatorParts.filter(config, heading, places.tagCount());
    history = EstimatorParts.history(config, filter);
    prediction = new OdometryPrediction(config, history, filter);
    corrections = new CameraMissCorrection(EstimatorParts.watch(config));
    mounts = new CameraMounts(config, heading, places, filter, history, corrections);
    replay = new VisionReplay(config, history, filter, prediction, mounts);
    vision =
        new VisionUpdate(
            config,
            onField,
            new VisionUpdate.Parts(
                filter, history, places, corrections, mounts, replay, wheelDoubt, motionEvidence));
    for (CameraConfig camera : cameras) {
      mounts.mount(camera.name(), camera.mountTolerance());
    }
    filter.cameraCount(places.placedCameraCount());
    filter.set(initial, yaw.radians());
  }

  /**
   * One odometry step: how the robot moved since the previous step, ending at this time, and what
   * an IMU read then. Steps come in time order, at whatever rate odometry runs (250 Hz on a robot);
   * a step no newer than the last is ignored. The first step only starts the clock and the IMU's
   * reference.
   *
   * <p>An IMU turns a step only when it also read the step before. A step after the IMU was lost,
   * came back (its reading may have jumped), or took over from the other IMU (which has a zero of
   * its own) is turned by the wheels, and the IMU's next steps carry on from its new reading.
   *
   * @param yawRadians the IMU's yaw, counterclockwise; any zero, wrapped or not. One that isn't a
   *     number counts as no reading, so one bad reading can't poison the heading.
   * @param source which IMU read it: the gyro, or the backup IMU while the gyro is lost; the wheels
   *     when neither does, and the yaw is ignored
   */
  public void addOdometry(
      long timestampNanos, BodyMotion motion, double yawRadians, YawSource source) {
    YawSource readBy = ContinuousYaw.readerOf(source, yawRadians);
    if (!started) {
      start(timestampNanos, readBy, yawRadians);
      return;
    }
    long previousTime = history.newestTime();
    if (timestampNanos <= previousTime) {
      return;
    }
    YawSource turnedBy = yaw.turnerOf(readBy);
    double stepSeconds = Timestamps.toSeconds(timestampNanos - previousTime);
    double driftSeen = Double.NaN;
    if (turnedBy == YawSource.WHEELS) {
      yaw.turnByWheels(motion.dthetaRadians());
      gyroAtRest.clear();
    } else {
      double imuTurn = yaw.turnByImu(yawRadians);
      if (turnedBy == YawSource.GYRO) {
        driftSeen = gyroAtRest.step(stepSeconds, motion, imuTurn);
      } else {
        gyroAtRest.clear();
      }
    }
    odometryOnly.step(motion, yaw.radians());
    yaw.read(readBy, yawRadians);

    double shareThatCounts = wheelDoubt.shareOfTravelThatCounts();
    double shareAcrossTheField = wheelDoubt.shareOfTravelAcrossTheField();
    double addedVariance =
        wheelDoubt.addedVariance(
            stepSeconds,
            Math.hypot(motion.dxMeters(), motion.dyMeters()),
            shareThatCounts,
            motion.residualMeters());
    int step =
        history.add(
            timestampNanos,
            shareThatCounts * shareAcrossTheField * motion.dxMeters(),
            shareThatCounts * shareAcrossTheField * motion.dyMeters(),
            motion.residualMeters(),
            addedVariance,
            turnedBy,
            yaw.radians());
    if (!Double.isNaN(driftSeen)) {
      history.sawDriftOverNewestStep(driftSeen, gyroAtRest.variance());
    }
    double xBefore = filter.x();
    double yBefore = filter.y();
    prediction.predict(step, 0, 1);
    history.movedOverNewestStep(
        filter.x() - xBefore, filter.y() - yBefore, prediction.positionVarianceOf(step));
    prediction.learnDrift(step);
    history.store(step, filter);
    pruneHistory();
  }

  /** The first odometry step: the history starts at it, with the filter where the robot was put. */
  private void start(long timestampNanos, YawSource readBy, double yawRadians) {
    started = true;
    yaw.start(readBy, yawRadians);
    history.start(timestampNanos, readBy, yaw.radians());
    filter.set(initial, yaw.radians());
    odometryOnly.place(initial.pose(), yaw.radians());
    replay.startedHere();
  }

  /**
   * What the robot's other sensors say about its wheels, for the odometry steps added after this
   * until the next call. A new collision makes the next step uncertain by {@link
   * EstimatorConfig#collisionStdDevMeters}: the robot may have moved without its wheels seeing it.
   * Jolts within {@link EstimatorConfig#collisionWindowSeconds} of the last are the same collision.
   * A share of one (a jolt the wheels partly explain) adds that share of its spread, outside a
   * collision's window, and opens none.
   */
  public void setConditions(DriveConditions conditions) {
    if (!started) {
      wheelDoubt.conditionsBeforeOdometry(conditions);
      return;
    }
    wheelDoubt.conditions(conditions, history.newestTime());
    motionEvidence.conditions(
        conditions, wheelDoubt.shareOfTravelThatCounts(), history.newestTime());
  }

  /** Drops steps older than the history keeps, and the measurements at or before its new start. */
  private void pruneHistory() {
    history.dropStepsNotNeededSince(history.newestTime() - historyNanos);
    replay.forgetAtOrBefore(history.oldestTime());
  }

  /** A camera measurement, to apply at the next {@link #update}. */
  public void addVision(VisionMeasurement measurement) {
    vision.add(measurement);
  }

  /**
   * A camera's doubt about its mount ({@link MountCheck}), as of now. With {@link
   * HeadingCorrection#HEADING_MEASUREMENTS_AND_MOUNT_DOUBT}, the next {@link #update} adds it to
   * what the camera repeats in every image: the doubt to its shift, and the doubt over its tags'
   * distance to its turn. Otherwise it's ignored here (the vision pipeline widens each image by it
   * either way).
   *
   * <p>A doubt that comes after the camera's fixes counted is taken as new from then on. What the
   * estimate already took from them stays as sure as it was, and what's left of the doubt holds the
   * camera's later fixes off. The mount check judges over its latest few hundred tags, so the doubt
   * always comes a little after the knock.
   *
   * @param doubtMeters how far its measurements may be off from where its mount moved
   * @param distanceMeters how far its tags were, on average, when that was judged; NaN until then
   */
  public void mountDoubt(String camera, double doubtMeters, double distanceMeters) {
    mounts.doubt(camera, doubtMeters, distanceMeters);
  }

  /**
   * Tells the estimator where the robot is, now: placed on its start, or a driver's "against the
   * wall here". History before it is dropped. A reset with no heading (infinite heading variance)
   * keeps the heading as it was.
   *
   * <p>The heading it gives is a person's word, as a driver's heading reset is: a robot set down
   * crooked, or backwards, on its start believes the start's heading all the same. So it is in
   * doubt until a camera backs it ({@link #headingDoubt}), and no image is solved with it
   * meanwhile.
   */
  public void reset(PoseReset reset) {
    vision.placed(reset);
    wheelDoubt.forgetSuspectTravel();
    placeOdometryOnly(reset);
    if (!started) {
      initial = reset;
      filter.set(reset, yaw.radians());
      return;
    }
    filter.set(reset, yaw.radians());
    vision.restartHistory();
  }

  /** The odometry-only pose starts again where the robot was placed: its heading too, if given. */
  private void placeOdometryOnly(PoseReset reset) {
    double heading = odometryOnly.headingRadians();
    odometryOnly.place(reset.pose(), yaw.radians());
    if (!reset.covariance().hasHeading()) {
      odometryOnly.face(heading);
    }
  }

  /**
   * Tells the estimator which way the robot faces, now, keeping where it is: a driver's "facing
   * straight downfield" when the heading has gone wrong. History before it is dropped.
   *
   * <p>The driver may be wrong: facing elsewhere, or pressing at the wrong moment. Until a camera's
   * own heading agrees with it, a position solved with it can't vote to move the estimate, since a
   * wrong heading puts every such position off alike. If the cameras' headings agree on another,
   * the estimate turns to theirs.
   *
   * @param stdDevRadians how sure: how straight a driver lines the robot up by eye
   */
  public void resetHeading(Rotation2d heading, double stdDevRadians) {
    boolean positive = stdDevRadians > 0;
    if (!positive || Double.isInfinite(stdDevRadians)) {
      throw new IllegalArgumentException("a heading reset's uncertainty: " + stdDevRadians);
    }
    vision.headingReset();
    headingResets++;
    odometryOnly.face(heading.getRadians());
    double variance = square(stdDevRadians);
    if (!started) {
      PoseCovariance before = initial.covariance();
      initial =
          new PoseReset(
              initial.timestampNanos(),
              new Pose2d(initial.pose().getTranslation(), heading),
              new PoseCovariance(before.xx(), before.xy(), before.yy(), variance),
              initial.reason());
      filter.set(initial, yaw.radians());
      return;
    }
    filter.resetHeading(heading.getRadians() - yaw.radians(), variance);
    vision.restartHistory();
  }

  /**
   * Whether the heading is a person's word no camera has backed yet: a placement's ({@link #reset})
   * or a driver's reset ({@link #resetHeading}). An image solved with it would land off by its
   * error, and could never back it.
   */
  public boolean headingInDoubt() {
    return vision.headingDoubt() != HeadingDoubt.NONE;
  }

  /** Whose word the heading rests on while no camera has backed it; {@code NONE} once one has. */
  public HeadingDoubt headingDoubt() {
    return vision.headingDoubt();
  }

  /** How many times the drive team has reset the heading ({@link #resetHeading}). */
  public int headingResets() {
    return headingResets;
  }

  /** What the latest recovery did, and on what evidence; empty before the first. */
  public Optional<Recovered> lastRecovery() {
    return vision.lastRecovery();
  }

  /** Where the wheels and the IMU alone put the robot since it was last placed. */
  public OdometryOnly odometryOnly() {
    return odometryOnly;
  }

  /**
   * Applies the measurements added since the last update, each at its capture time, and decides
   * whether odometry needs rescuing. Call it once per loop, after adding the loop's odometry and
   * measurements.
   *
   * @return what happened to each measurement, for the log
   */
  public List<MeasurementVerdict> update() {
    if (!started) {
      return vision.refuseAll(Verdict.BEFORE_ODOMETRY);
    }
    return vision.update();
  }

  /**
   * How far the estimator's own watch has a camera's measurements in doubt, in meters: how far they
   * keep landing to one side of the other cameras', beyond what noise explains. Zero for a camera
   * it has not heard enough from, and for one with no other to be judged against.
   */
  public double cameraDoubtMeters(String camera) {
    return corrections.doubtMeters(places.placeIfAny(camera));
  }

  /**
   * The cameras beyond the {@value ConsideredErrors#MAX_CAMERAS} whose repeated errors the
   * estimator keeps apart: each image's turn and shift are taken as new, so their images, averaged,
   * grow surer than they should. Empty unless a robot has more cameras than that.
   */
  public List<String> camerasWithoutPlaces() {
    return places.camerasWithoutPlaces();
  }

  /**
   * The tags measurements have named that aren't the field's, in the order each first came: their
   * placement can't be kept apart, so each image's is taken as new, and their images, averaged,
   * grow surer than they should. Empty unless a measurement comes from a tag off the layout.
   */
  public List<Integer> tagsWithoutPlaces() {
    return places.tagsWithoutPlaces();
  }

  /**
   * Every camera it has weighed a measurement from (or been told a mount's doubt about), in the
   * order each first came: those whose repeated errors it keeps apart, then any beyond them.
   */
  public List<String> cameras() {
    return places.cameras();
  }

  /**
   * Where the robot was at a moment in the history, as the estimate now has it: for tying a camera
   * image to the robot's heading when the image was taken.
   *
   * @return empty outside the history
   */
  public Optional<Pose2d> poseAt(long timestampNanos) {
    if (!started || timestampNanos < history.oldestTime()) {
      return Optional.empty();
    }
    long newestTime = history.newestTime();
    if (timestampNanos < newestTime) {
      return Optional.of(
          history.poseAt(
              timestampNanos, filter.headingOffset(), filter.gyroScale(), filter.gyroDrift()));
    }
    // At the newest step or a little after it, the robot is taken to be where it is now.
    boolean tooFarAhead = timestampNanos > newestTime + futureToleranceNanos;
    if (tooFarAhead) {
      return Optional.empty();
    }
    return Optional.of(pose());
  }

  /**
   * The gyro's scale error as the estimator has it, in percent: 3 for a gyro it takes to read 3%
   * more turn than the robot makes. Zero until the cameras have said anything of it, and always
   * zero when the gyro's errors are only considered ({@link
   * EstimatorConfig.GyroErrors#CONSIDERED}).
   *
   * <p>The filter keeps the scale as the heading each radian of the gyro's turn adds, s. A gyro
   * that reads (1 + e) times the true turn adds -e / (1 + e) of each radian it reads, so e is -s /
   * (1 + s).
   */
  public double gyroScalePercent() {
    double addedPerRadian = filter.gyroScale();
    return -addedPerRadian / (1 + addedPerRadian) * PERCENT;
  }

  /** How sure that is, as a standard deviation, in percent. */
  public double gyroScaleStdDevPercent() {
    return filter.gyroScaleStdDev() * PERCENT;
  }

  /**
   * The gyro's steady drift as the estimator has it, in degrees a minute, counterclockwise: 10 for
   * a gyro it takes to turn 10° a minute while the robot stands still. Zero until the cameras have
   * said anything of it, and always zero when the gyro's errors are only considered.
   *
   * <p>The filter keeps the drift as the heading each second adds to undo it, so the gyro's own
   * drift is the opposite.
   */
  public double gyroDriftDegreesPerMinute() {
    return -Math.toDegrees(filter.gyroDrift()) * SECONDS_PER_MINUTE;
  }

  /** How sure that is, as a standard deviation, in degrees a minute. */
  public double gyroDriftStdDevDegreesPerMinute() {
    return Math.toDegrees(filter.gyroDriftStdDev()) * SECONDS_PER_MINUTE;
  }

  /**
   * How late the estimator's own watch takes a camera's images to be stamped, in seconds: negative
   * for early. What that puts each of its measurements off by is taken off before they are weighed
   * ({@link CameraWatch}). Zero for a camera it has not heard enough from, and for one with no
   * other to be judged against.
   */
  public double cameraLateSeconds(String camera) {
    return corrections.lateSeconds(places.placeIfAny(camera));
  }

  /**
   * How far the estimator takes a camera to be turned on its mount, in degrees, counterclockwise:
   * what it has learned of the camera's turn from its images. Zero for a camera it has no place
   * for, and for one it has heard nothing from.
   */
  public double cameraTurnDegrees(String camera) {
    return Math.toDegrees(filter.cameraTurn(places.placeIfAny(camera)));
  }

  /**
   * How sure that is, as a standard deviation, in degrees: its mount's tolerance until its images
   * say more. NaN for a camera it has no place for, whose turn it takes as new in every image.
   */
  public double cameraTurnStdDevDegrees(String camera) {
    return Math.toDegrees(filter.cameraTurnStdDev(places.placeIfAny(camera)));
  }

  /** How it weighs its sources. */
  EstimatorConfig config() {
    return config;
  }

  /** The current pose: cheap, for driving every loop. */
  public Pose2d pose() {
    return filter.pose(yaw.radians());
  }

  /** The current estimate, with its uncertainty, velocity, mode, and health. */
  public LocalizationEstimate estimate() {
    MeasurementBooks books = vision.books();
    NewestStep newest = newestStep();
    long now = newest.timeNanos();
    // Before odometry starts, nothing has said the gyro is lost.
    boolean gyroConnected = !started || yaw.lastReader() != YawSource.WHEELS;
    ModeFacts facts =
        new ModeFacts(
            gyroConnected,
            vision.isPlaced(),
            books.camerasKeepDisagreeing(config.lostAfterRejections()),
            books.visionRecent(now, visionTimeoutNanos));
    return new LocalizationEstimate(
        now,
        pose(),
        filter.covariance(),
        newest.motion().fieldVelocity(),
        newest.motion().yawRateRadiansPerSecond(),
        facts.mode(),
        LocalizationEstimate.Health.builder()
            .visionAgeSeconds(books.visionAgeSeconds(now))
            .accepted(books.accepted())
            .rejected(books.rejected())
            .lastInnovationMeters(books.lastInnovationMeters())
            .recentDisagreementMeters(books.largestDisagreementSince(now - disagreementWindowNanos))
            .residualMeters(newest.wheelResidualMeters())
            .suspectTravelMeters(wheelDoubt.suspectTravelMeters())
            .tiltRadians(wheelDoubt.conditions().tiltRadians())
            .collisionActive(started && wheelDoubt.inCollisionAt(now))
            .recoveries(books.recoveries())
            .gyroConnected(gyroConnected)
            .build());
  }

  /**
   * The newest odometry step as the estimate reports it: when it ended, how the robot moved over
   * the last stretch, and how far the wheels disagreed in it.
   */
  private record NewestStep(
      long timeNanos, OdometryHistory.Motion motion, double wheelResidualMeters) {
    /** Before odometry starts: no time, no motion, no disagreement. */
    static final NewestStep NONE_YET = new NewestStep(0, OdometryHistory.Motion.NONE, 0);
  }

  private NewestStep newestStep() {
    if (!started) {
      return NewestStep.NONE_YET;
    }
    return new NewestStep(
        history.newestTime(),
        history.motionOver(VELOCITY_WINDOW_NANOS, filter.headingOffset()),
        history.wheelResidual(history.newestStep()));
  }

  private static double square(double value) {
    return value * value;
  }
}
