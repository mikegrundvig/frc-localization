package com.michaelgrundvig.frc.localization;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Predicate;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * Where the robot is: odometry, corrected by camera measurements, with the gyro owning heading.
 *
 * <p>The state is the robot's position and a heading offset: the field heading is the gyro's
 * reading plus the offset, so the gyro keeps all short-term authority over heading, and cameras
 * only correct its slow drift. A 3x3 covariance holds the uncertainty of all three and how they
 * relate: an unsure heading makes every meter driven add position uncertainty sideways.
 *
 * <p><b>Odometry.</b> Each odometry step predicts: the wheels' motion, along the arc the gyro
 * turned through, moves the position, and the uncertainty grows with the distance driven, the
 * turning, and time, and with whatever makes the wheels suspect ({@link WheelDoubt}).
 *
 * <p><b>Cameras.</b> A camera measurement is applied at the moment its image was taken, however
 * late it arrives. The estimator keeps {@link EstimatorConfig#historySeconds} of steps, rewinds to
 * the measurement, applies it, and replays the steps since ({@link #replay}), so measurements give
 * the same result in any order of arrival. Each is gated against the estimate first, and again
 * whenever a replay applies it anew: one that a later-arriving, earlier image rules out is taken
 * back, as it would have been rejected had the two come in order.
 *
 * <p><b>Recovery.</b> When several rejected measurements agree with each other, odometry was the
 * one that was wrong (a collision, a spin), and the estimate recovers to them ({@link Recovery}).
 * It recovers to their heading first, if that is what they agree is wrong (a bad heading reset, a
 * robot placed backwards), since every position worked out with a wrong heading is wrong too.
 *
 * <p><b>Honesty.</b> Its uncertainty is meant to be honest, since gates, χ² tests, and any bound on
 * the error trust it: the error should go past 3.72 times the spread it claims no more than once in
 * a thousand loops, which the consistency study checks in the simulator (docs/testing.md). So
 * errors that repeat are never averaged away as if each were new ({@link PoseFilter} says which,
 * and how). Each camera is told apart by its name, and each tag by its number ({@link
 * ErrorPlaces}).
 *
 * <p>Its parts: {@link OdometryHistory} keeps the steps, {@link PoseFilter} does the filter's math,
 * {@link WheelDoubt} says how far each step's wheels are doubted, {@link ErrorPlaces} tells cameras
 * and tags apart, and {@link Recovery} judges when the cameras agree against odometry. This class
 * feeds them and keeps the books.
 *
 * <p>Pure logic: no clock, no hardware, no static state. Every input comes in through a method,
 * stamped with its capture time, so a replay reproduces every decision. Use it from one thread.
 */
public final class PoseEstimator {
  /** The robot's velocity is taken over the odometry steps of this last stretch of time. */
  private static final long VELOCITY_WINDOW_NANOS = 100_000_000L;

  /** The history has room for its seconds of odometry at up to this many steps a second. */
  private static final int MOST_STEPS_PER_SECOND = 1000;

  private static final int FEWEST_STEPS_KEPT = 16;

  private static final double RADIANS_PER_TURN = 2 * Math.PI;
  private static final double SECONDS_PER_MINUTE = 60;

  private final EstimatorConfig config;
  private final Predicate<Translation2d> onField;

  // How fast the uncertainty grows as the robot drives, from the configuration. Each is a variance
  // for one unit of what grows it: a meter driven, a radian turned, a second passed.
  private final double positionVariancePerMeter;
  private final double positionVariancePerRadian;
  private final double headingVariancePerSecond;

  /** The heading variance a radian of turning adds when an IMU turned the step. */
  private final double imuHeadingVariancePerRadian;

  /** And when the wheels turned it, which drift far faster. */
  private final double wheelHeadingVariancePerRadian;

  // What a camera's turn and shift on its mount are taken to be before any doubt about its mount.
  private final double usualCameraTurnVariance;
  private final double usualCameraShiftVariance;

  private final long historyNanos;
  private final long futureToleranceNanos;
  private final long visionTimeoutNanos;
  private final long disagreementWindowNanos;

  /** A camera's own heading within this of the estimate's backs a driver's heading reset. */
  private final double headingAgreementRadians;

  private final OdometryHistory history;
  private final PoseFilter filter;
  private final WheelDoubt wheelDoubt;
  private final ErrorPlaces places;
  private final Recovery recovery;

  // The IMUs' readings, unwrapped into one continuous yaw.

  /** Whether the first odometry step has come: nothing can be estimated before it. */
  private boolean started;

  /** The continuous yaw as of the newest step, in radians: it never wraps. */
  private double yaw;

  /** The last reading an IMU gave, as it gave it: the next reading's turn is measured from it. */
  private double lastImuYaw;

  /** What read the yaw at the newest step: an IMU, or the wheels when none did. */
  private YawSource lastSource = YawSource.WHEELS;

  // Where the robot was told it is.

  /** Where the robot is taken to be when odometry starts: as constructed, or as last reset. */
  private PoseReset initial;

  /** Whether the robot has been placed: told where it is, or found by a camera. */
  private boolean placed;

  /** Measurements taken at or before this moment are from before the estimate was last reset. */
  private long lastResetNanos = Long.MIN_VALUE;

  /**
   * Whether the heading is a driver's reset that no camera has backed yet. A camera backs it when
   * its own heading agrees with it, or when the cameras' headings turn the estimate. Until then a
   * position solved with that heading cannot vote to move the estimate: if the heading is wrong, it
   * puts every such position off alike, and they agree with each other.
   */
  private boolean headingInDoubt;

  // The cameras.

  /** The measurements added since the last update. */
  private final List<VisionMeasurement> pending = new ArrayList<>();

  /** The measurements the history has applied and still reaches back to, in time order. */
  private final List<AppliedMeasurement> applied = new ArrayList<>();

  /** Each camera's doubt about its mount, as last told, for the next update to apply. */
  private final Map<String, MountDoubt> mountDoubts = new HashMap<>();

  // The books, for the estimate's mode and health.

  /** How far each recently accepted measurement was from the estimate. */
  private final List<Agreement> agreements = new ArrayList<>();

  private long lastAcceptedNanos = Long.MIN_VALUE;
  private int consecutiveRejections;
  private int acceptedCount;
  private int rejectedCount;
  private int recoveries;
  private double lastInnovationMeters = Double.NaN;

  /**
   * A camera's measurement and the moment the history applies it at.
   *
   * @param time when it is applied: its capture time, or the newest step's time if it was stamped a
   *     little after that
   */
  private record AppliedMeasurement(long time, VisionMeasurement measurement) {}

  /**
   * A measurement to apply during a replay.
   *
   * @param newThisUpdate whether it arrived this update, or was applied before and is being applied
   *     again. Both are gated against the estimate as the replay has it. One applied before that an
   *     earlier image, new this update, now rules out is taken back, as it would have been rejected
   *     had the two come in order.
   */
  private record ReplayedMeasurement(AppliedMeasurement applied, boolean newThisUpdate) {
    long time() {
      return applied.time();
    }
  }

  /** How far an accepted measurement was from the estimate, for the health's disagreement. */
  private record Agreement(long time, double meters) {}

  /**
   * @param config how to weigh the sources
   * @param onField whether a point is on the field: measurements off it are rejected
   * @param tags the field's tags: each one's placement is kept apart, the same for every camera
   *     that sees it and every time it's seen. A measurement from any other counts its placement as
   *     new in each image ({@link #tagsWithoutPlaces})
   * @param initial where the robot is at the start, and how sure that is; for a placeholder (the
   *     robot not placed yet) give a large uncertainty, and the first camera measurement wins
   */
  public PoseEstimator(
      EstimatorConfig config,
      Predicate<Translation2d> onField,
      Collection<Integer> tags,
      PoseReset initial) {
    this.config = config;
    this.onField = onField;
    this.initial = initial;
    positionVariancePerMeter = square(config.translationStdDevPerMeter());
    positionVariancePerRadian = square(config.translationStdDevPerRadian());
    headingVariancePerSecond =
        square(Math.toRadians(config.headingDriftDegreesPerMinute())) / SECONDS_PER_MINUTE;
    imuHeadingVariancePerRadian =
        square(Math.toRadians(config.headingNoiseDegreesPerTurn())) / RADIANS_PER_TURN;
    usualCameraTurnVariance = square(Math.toRadians(config.cameraTurnStdDevDegrees()));
    usualCameraShiftVariance = square(config.cameraShiftStdDevMeters());
    wheelHeadingVariancePerRadian =
        square(Math.toRadians(config.wheelHeadingStdDevDegreesPerTurn())) / RADIANS_PER_TURN;
    historyNanos = Timestamps.fromSeconds(config.historySeconds());
    futureToleranceNanos = Timestamps.fromSeconds(config.futureToleranceSeconds());
    visionTimeoutNanos = Timestamps.fromSeconds(config.visionTimeoutSeconds());
    disagreementWindowNanos = Timestamps.fromSeconds(config.disagreementWindowSeconds());
    headingAgreementRadians = Math.toRadians(config.recoveryAgreementDegrees());
    wheelDoubt = new WheelDoubt(config);
    places = new ErrorPlaces(tags);
    recovery = new Recovery(config);
    filter =
        new PoseFilter(
            new PoseFilter.Considered(
                Math.toRadians(config.headingStdDevDegreesPerTurn()) / RADIANS_PER_TURN,
                Math.toRadians(config.backupHeadingStdDevDegreesPerTurn()) / RADIANS_PER_TURN,
                Math.toRadians(config.cameraTurnStdDevDegrees()),
                config.cameraShiftStdDevMeters(),
                Math.toRadians(config.cameraShiftStdDevDegrees()),
                config.tagPlacementStdDevMeters(),
                config.cameraShiftCorrelationMeters(),
                Math.toRadians(config.cameraShiftCorrelationDegrees())),
            places.tagCount(),
            config.headingCorrection() == EstimatorConfig.HeadingCorrection.EVERY_FIX);
    int stepsInTheHistory = (int) Math.ceil(config.historySeconds() * MOST_STEPS_PER_SECOND) + 1;
    history =
        new OdometryHistory(
            Math.max(FEWEST_STEPS_KEPT, stepsInTheHistory), filter.uncertaintyLength());
    filter.set(initial, yaw);
  }

  // ---------------------------------------------------------------------------------------------
  // Odometry.
  // ---------------------------------------------------------------------------------------------

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
    YawSource readBy = whatReadTheYaw(source, yawRadians);
    if (!started) {
      start(timestampNanos, readBy, yawRadians);
      return;
    }
    long previousTime = history.newestTime();
    if (timestampNanos <= previousTime) {
      return;
    }
    YawSource turnedBy = whatTurnsTheStep(readBy);
    if (turnedBy == YawSource.WHEELS) {
      yaw += motion.dthetaRadians();
    } else {
      yaw += MathUtil.angleModulus(yawRadians - lastImuYaw);
    }
    if (readBy != YawSource.WHEELS) {
      lastImuYaw = yawRadians;
    }
    lastSource = readBy;

    double shareThatCounts = wheelDoubt.shareOfTravelThatCounts();
    double shareAcrossTheField = wheelDoubt.shareOfTravelAcrossTheField();
    double addedVariance =
        wheelDoubt.addedVariance(
            Timestamps.toSeconds(timestampNanos - previousTime),
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
            yaw);
    predictThrough(step, 0, 1);
    history.store(step, filter);
    dropWhatTheHistoryNoLongerReaches();
  }

  /** What read the yaw: the IMU named, or the wheels if none was or its reading isn't a number. */
  private static YawSource whatReadTheYaw(YawSource source, double yawRadians) {
    if (source == YawSource.WHEELS || !Double.isFinite(yawRadians)) {
      return YawSource.WHEELS;
    }
    return source;
  }

  /**
   * What turns a step: the IMU that read its yaw, if the same IMU read the step before too, so the
   * difference between the two readings is a turn. Otherwise the wheels' own turning stands in.
   */
  private YawSource whatTurnsTheStep(YawSource readBy) {
    boolean sameImuReadTheStepBefore = readBy != YawSource.WHEELS && readBy == lastSource;
    if (sameImuReadTheStepBefore) {
      return readBy;
    }
    return YawSource.WHEELS;
  }

  /** The first odometry step: the history starts at it, with the filter where the robot was put. */
  private void start(long timestampNanos, YawSource readBy, double yawRadians) {
    started = true;
    yaw = readBy == YawSource.WHEELS ? 0 : yawRadians;
    lastImuYaw = yaw;
    lastSource = readBy;
    history.start(timestampNanos, readBy, yaw);
    filter.set(initial, yaw);
    history.store(OdometryHistory.OLDEST_STEP, filter);
    lastResetNanos = timestampNanos;
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
  }

  /**
   * The filter's prediction through part of a step in the history: from one fraction of the way
   * through it to another. A whole step is from 0 to 1. A replay predicts up to each measurement
   * that falls inside a step, applies it, and carries on from there.
   *
   * <p>The step's travel, turn and added uncertainty are shared out evenly over it, so each part
   * takes its fraction of them. The uncertainty a whole step adds is:
   *
   * <ul>
   *   <li>to the position, a variance for each meter driven and each radian turned, plus the step's
   *       own added variance for slip, tilt and collisions;
   *   <li>to the heading, a variance for each second passed (the gyro's drift) and for each radian
   *       turned (the noise of whatever turned the step).
   * </ul>
   */
  private void predictThrough(int step, double fromFraction, double toFraction) {
    double fraction = toFraction - fromFraction;
    if (fraction <= 0) {
      return;
    }
    double stepTurn = history.turnOver(step);
    double stepForward = history.travelForward(step);
    double stepLeft = history.travelLeft(step);
    double stepDistance = Math.hypot(stepForward, stepLeft);
    double stepSeconds = Timestamps.toSeconds(history.durationNanos(step));
    YawSource turnedBy = history.turnedBy(step);

    double stepPositionVariance =
        positionVariancePerMeter * stepDistance
            + positionVariancePerRadian * Math.abs(stepTurn)
            + history.addedPositionVariance(step);
    double headingVariancePerRadian =
        turnedBy == YawSource.WHEELS ? wheelHeadingVariancePerRadian : imuHeadingVariancePerRadian;
    double stepHeadingVariance =
        headingVariancePerSecond * stepSeconds + headingVariancePerRadian * Math.abs(stepTurn);

    filter.predict(
        history.yawAfter(step - 1) + fromFraction * stepTurn,
        fraction * stepTurn,
        fraction * stepForward,
        fraction * stepLeft,
        fraction * stepPositionVariance,
        fraction * stepHeadingVariance,
        turnedBy);
  }

  /** Drops steps older than the history keeps, and the measurements at or before its new start. */
  private void dropWhatTheHistoryNoLongerReaches() {
    history.dropStepsNotNeededSince(history.newestTime() - historyNanos);
    long oldestTime = history.oldestTime();
    applied.removeIf(done -> done.time() <= oldestTime);
  }

  // ---------------------------------------------------------------------------------------------
  // What the estimator is told.
  // ---------------------------------------------------------------------------------------------

  /** A camera measurement, to apply at the next {@link #update}. */
  public void addVision(VisionMeasurement measurement) {
    pending.add(measurement);
  }

  /**
   * A camera's doubt about its mount ({@link MountCheck}), as of now. With {@link
   * EstimatorConfig.HeadingCorrection#HEADING_MEASUREMENTS_AND_MOUNT_DOUBT}, the next {@link
   * #update} adds it to what the camera repeats in every image: the doubt to its shift, and the
   * doubt over its tags' distance to its turn. Otherwise it's ignored here (the vision pipeline
   * widens each image by it either way).
   *
   * <p>A doubt that comes after the camera's fixes counted is taken as new from then on: what the
   * estimate already took from them stays as sure as it was, and what's left of the doubt holds the
   * camera's later fixes off. Since the mount check judges over its latest few hundred tags, the
   * doubt always comes a little after the knock.
   *
   * @param doubtMeters how far its measurements may be off from where its mount moved
   * @param distanceMeters how far its tags were, on average, when that was judged; NaN until then
   */
  public void mountDoubt(String camera, double doubtMeters, double distanceMeters) {
    boolean mountDoubtIsConsidered =
        config.headingCorrection()
            == EstimatorConfig.HeadingCorrection.HEADING_MEASUREMENTS_AND_MOUNT_DOUBT;
    if (mountDoubtIsConsidered) {
      mountDoubts.put(camera, new MountDoubt(doubtMeters, distanceMeters));
    }
  }

  /**
   * Tells the estimator where the robot is, now: placed on its start, or a driver's "against the
   * wall here". History before it is dropped. A reset with no heading (infinite heading variance)
   * keeps the heading as it was.
   */
  public void reset(PoseReset reset) {
    placed = true;
    if (reset.covariance().hasHeading()) {
      headingInDoubt = false;
    }
    recovery.clear();
    consecutiveRejections = 0;
    wheelDoubt.forgetSuspectTravel();
    if (!started) {
      initial = reset;
      filter.set(reset, yaw);
      return;
    }
    filter.set(reset, yaw);
    startHistoryHere();
  }

  /**
   * Tells the estimator which way the robot faces, now, keeping where it is: a driver's "facing
   * straight downfield" when the heading has gone wrong. History before it is dropped.
   *
   * <p>The driver may be wrong: facing elsewhere, or pressing at the wrong moment. Until a camera's
   * own heading agrees with it (within {@link EstimatorConfig#recoveryAgreementDegrees}), a
   * position solved with it can't vote to move the estimate, since a wrong heading puts every such
   * position off alike; if the cameras' headings agree on another, the estimate turns to theirs.
   *
   * @param stdDevRadians how sure: how straight a driver lines the robot up by eye
   */
  public void resetHeading(Rotation2d heading, double stdDevRadians) {
    if (!(stdDevRadians > 0) || Double.isInfinite(stdDevRadians)) {
      throw new IllegalArgumentException("a heading reset's uncertainty: " + stdDevRadians);
    }
    headingInDoubt = true;
    recovery.clear();
    consecutiveRejections = 0;
    double variance = square(stdDevRadians);
    if (!started) {
      PoseCovariance before = initial.covariance();
      initial =
          new PoseReset(
              initial.timestampNanos(),
              new Pose2d(initial.pose().getTranslation(), heading),
              new PoseCovariance(before.xx(), before.xy(), before.yy(), variance),
              initial.reason());
      filter.set(initial, yaw);
      return;
    }
    filter.resetHeading(heading.getRadians() - yaw, variance);
    startHistoryHere();
  }

  /**
   * Whether the heading is a driver's reset no camera has backed yet ({@link #resetHeading}): an
   * image solved with it would land off by its error, and could never back it.
   */
  boolean headingInDoubt() {
    return headingInDoubt;
  }

  /** Drops the history: the newest step, with the filter as it is now, becomes its start. */
  private void startHistoryHere() {
    history.keepOnlyNewest();
    history.store(OdometryHistory.OLDEST_STEP, filter);
    applied.clear();
    agreements.clear();
    lastResetNanos = history.oldestTime();
  }

  // ---------------------------------------------------------------------------------------------
  // The update: the loop's camera measurements.
  // ---------------------------------------------------------------------------------------------

  /**
   * Applies the measurements added since the last update, each at its capture time, and decides
   * whether odometry needs rescuing. Call it once per loop, after adding the loop's odometry and
   * measurements.
   *
   * @return what happened to each measurement, for the log
   */
  public List<VisionDecision> update() {
    List<VisionDecision> decisions = new ArrayList<>();
    List<VisionMeasurement> usable = new ArrayList<>();
    for (VisionMeasurement measurement : pending) {
      Optional<Verdict> refusal = whyItCannotBeApplied(measurement);
      if (refusal.isPresent()) {
        rejectedCount++;
        decisions.add(
            new VisionDecision(measurement, refusal.get(), Double.NaN, Double.NaN, Double.NaN));
      } else {
        usable.add(measurement);
      }
    }
    pending.clear();
    if (!usable.isEmpty()) {
      replay(usable, decisions);
    }
    recoverIfTheCamerasAgree();
    applyMountDoubts();
    if (started) {
      long windowStart = history.newestTime() - disagreementWindowNanos;
      agreements.removeIf(agreement -> agreement.time() < windowStart);
    }
    return decisions;
  }

  /**
   * Why a measurement cannot be applied at all, before it is ever compared with the estimate: its
   * image was taken at a moment the history does not reach, or it puts the robot off the field.
   * Empty for a measurement that can be applied.
   */
  private Optional<Verdict> whyItCannotBeApplied(VisionMeasurement measurement) {
    long taken = measurement.timestampNanos();
    if (!started) {
      return Optional.of(Verdict.BEFORE_ODOMETRY);
    }
    if (taken <= lastResetNanos) {
      return Optional.of(Verdict.BEFORE_RESET);
    }
    if (taken <= history.oldestTime()) {
      return Optional.of(Verdict.STALE);
    }
    if (taken > history.newestTime() + futureToleranceNanos) {
      return Optional.of(Verdict.FROM_THE_FUTURE);
    }
    if (!onField.test(measurement.pose().getTranslation())) {
      return Optional.of(Verdict.OFF_FIELD);
    }
    return Optional.empty();
  }

  /**
   * Rewinds to just before the earliest new measurement and replays the history from there,
   * applying the new measurements and the ones applied before, in time order.
   *
   * <p>The filter is put back as it was after the last step before the earliest new measurement.
   * Then each later step is predicted through again. A measurement that falls inside a step splits
   * it: the filter predicts up to the measurement's moment, the measurement is gated and applied,
   * and the prediction carries on from there. Each step's stored filter state is rewritten as the
   * replay passes it.
   */
  private void replay(List<VisionMeasurement> usable, List<VisionDecision> decisions) {
    long newestTime = history.newestTime();
    long earliestNew = newestTime;
    for (VisionMeasurement measurement : usable) {
      earliestNew = Math.min(earliestNew, measurement.timestampNanos());
    }
    int startStep = history.lastStepBefore(earliestNew);
    long startTime = history.endTime(startStep);

    // Everything to apply after the starting step: what was applied there before, and what is new.
    // A measurement stamped a little after the newest step is applied at the newest step.
    List<ReplayedMeasurement> toApply = new ArrayList<>();
    for (AppliedMeasurement done : applied) {
      if (done.time() > startTime) {
        toApply.add(new ReplayedMeasurement(done, false));
      }
    }
    for (VisionMeasurement measurement : usable) {
      long appliedAt = Math.min(measurement.timestampNanos(), newestTime);
      toApply.add(new ReplayedMeasurement(new AppliedMeasurement(appliedAt, measurement), true));
    }
    toApply.sort(Comparator.comparingLong(ReplayedMeasurement::time));
    // The replay adds back each one that passes the gate again.
    applied.removeIf(done -> done.time() > startTime);

    history.restore(startStep, filter);
    int next = 0;
    for (int step = startStep + 1; step < history.stepCount(); step++) {
      long stepStart = history.endTime(step - 1);
      long stepEnd = history.endTime(step);
      double predictedUpTo = 0;
      while (next < toApply.size() && toApply.get(next).time() <= stepEnd) {
        ReplayedMeasurement replayed = toApply.get(next++);
        double fraction = (double) (replayed.time() - stepStart) / (stepEnd - stepStart);
        predictThrough(step, predictedUpTo, fraction);
        predictedUpTo = fraction;
        gateAndApply(replayed, history.yawPartWayThrough(step, fraction), decisions);
      }
      predictThrough(step, predictedUpTo, 1);
      history.store(step, filter);
    }
    // Anything left is at the starting step itself, which happens only when the history is a single
    // step: there is no later step to split.
    while (next < toApply.size()) {
      gateAndApply(toApply.get(next++), history.yawAfter(history.newestStep()), decisions);
    }
    history.store(history.newestStep(), filter);
  }

  /**
   * Compares a camera measurement with the estimate as it was when its image was taken, gates it,
   * and if it passes applies it as a Kalman update.
   *
   * @param yawThen the yaw when its image was taken
   */
  private void gateAndApply(
      ReplayedMeasurement replayed, double yawThen, List<VisionDecision> decisions) {
    VisionMeasurement measurement = replayed.applied().measurement();
    PoseFilter.Innovation innovation =
        filter.innovation(
            measurement,
            yawThen,
            places.cameraPlace(measurement.camera()),
            places.tagPlaces(measurement.tagIds()));
    if (innovation == null) {
      // It cannot be compared with the estimate at all, so it is left out. It is counted as a
      // rejection and given a verdict of its own, new this update or applied before, so that no
      // measurement leaves the estimator without one. It is not a camera disagreeing with the
      // estimate, so it does not count toward the rejections in a row that mean the estimate is
      // lost.
      rejectedCount++;
      decisions.add(
          new VisionDecision(
              measurement, Verdict.NOT_COMPARABLE, Double.NaN, Double.NaN, Double.NaN));
      return;
    }
    if (!passesTheGate(replayed, innovation, decisions)) {
      return;
    }
    filter.correct(innovation);
    applied.add(replayed.applied());
  }

  /**
   * Whether a camera measurement passes the gate, which is a chi-squared test: its innovation's
   * squared distance from zero, measured in its own uncertainty, may not exceed the configured
   * limit for a measurement of its kind (a position alone, or a pose). Keeps the books as it
   * decides.
   *
   * <p>A measurement that was applied before and passes again was counted when it first passed, and
   * changes nothing here. One applied before that now fails is taken back, and counted as a
   * rejection like a new one.
   */
  private boolean passesTheGate(
      ReplayedMeasurement replayed,
      PoseFilter.Innovation innovation,
      List<VisionDecision> decisions) {
    double gate =
        innovation.withHeading() ? config.gateChiSquaredPose() : config.gateChiSquaredPosition();
    double chiSquared = innovation.chiSquared();
    // Written so a chi-squared that isn't a number fails the gate instead of passing it.
    boolean passes = chiSquared <= gate;
    if (passes && !replayed.newThisUpdate()) {
      return true;
    }
    lastInnovationMeters = innovation.distanceMeters();
    boolean itsOwnHeadingAgrees =
        innovation.withHeading()
            && Math.abs(innovation.headingRadians()) <= headingAgreementRadians;
    if (itsOwnHeadingAgrees) {
      // A camera's own heading that agrees with a driver's reset backs it.
      headingInDoubt = false;
    }
    VisionMeasurement measurement = replayed.applied().measurement();
    if (!passes) {
      noteRejection(replayed, innovation);
      decisions.add(
          new VisionDecision(
              measurement, Verdict.GATED, innovation.distanceMeters(), chiSquared, gate));
      return false;
    }
    noteAcceptance(replayed, innovation);
    decisions.add(
        new VisionDecision(
            measurement, Verdict.ACCEPTED, innovation.distanceMeters(), chiSquared, gate));
    return true;
  }

  /** Counts a rejection, and hands recovery what it needs to judge whether the cameras agree. */
  private void noteRejection(ReplayedMeasurement replayed, PoseFilter.Innovation innovation) {
    rejectedCount++;
    consecutiveRejections++;
    VisionMeasurement measurement = replayed.applied().measurement();
    boolean positionAlonePasses =
        innovation.positionChiSquared() <= config.gateChiSquaredPosition();
    boolean solvedWithTheHeading = !measurement.lever().equals(VisionMeasurement.Lever.NONE);
    boolean solvedWithAHeadingInDoubt = solvedWithTheHeading && headingInDoubt;
    recovery.rejected(
        new Recovery.Rejected(
            replayed.time(),
            innovation.xMeters(),
            innovation.yMeters(),
            innovation.headingRadians(),
            positionAlonePasses,
            !solvedWithAHeadingInDoubt));
  }

  /** Counts an acceptance: the estimate rests on a camera again, and nothing needs recovering. */
  private void noteAcceptance(ReplayedMeasurement replayed, PoseFilter.Innovation innovation) {
    acceptedCount++;
    consecutiveRejections = 0;
    recovery.clear();
    lastAcceptedNanos = Math.max(lastAcceptedNanos, replayed.time());
    agreements.add(new Agreement(replayed.time(), innovation.distanceMeters()));
    placed = true;
  }

  /**
   * When enough recent rejected measurements agree with each other, the cameras are right and
   * odometry is wrong. The estimate turns to the heading they agree on, if they agree its heading
   * is wrong. Otherwise it jumps to where they put the robot. Either way its uncertainty starts
   * afresh.
   */
  private void recoverIfTheCamerasAgree() {
    if (!started) {
      return;
    }
    recovery.forgetOlderThanTheWindow(history.newestTime());
    if (!recovery.hasEnough()) {
      return;
    }
    OptionalDouble turn = recovery.headingCorrection();
    if (turn.isPresent()) {
      filter.turn(
          turn.getAsDouble(),
          square(config.lostPositionStdDevMeters()),
          square(Math.toRadians(config.recoveryHeadingStdDevDegrees())));
      headingInDoubt = false;
      startAfreshAfterRecovering();
      return;
    }
    Optional<Recovery.Move> move = recovery.positionCorrection();
    if (move.isEmpty()) {
      return;
    }
    Translation2d by = move.get().by();
    filter.shift(by.getX(), by.getY(), square(config.recoveryStdDevMeters()));
    startAfreshAfterRecovering();
    // The estimate is as recent as the images it now rests on. A recovery made later, once an
    // older image that held it back has aged out of the window, must not make them new.
    lastAcceptedNanos = Math.max(lastAcceptedNanos, move.get().newestTaken());
    placed = true;
  }

  /** After a recovery: the history starts again from it, and so do the doubts. */
  private void startAfreshAfterRecovering() {
    startHistoryHere();
    wheelDoubt.forgetSuspectTravel();
    recoveries++;
    consecutiveRejections = 0;
    recovery.clear();
  }

  /**
   * Sets each doubted camera's turn and shift variances from its doubt, at the newest step, and
   * keeps them there for a replay to start from. A replay from before restores the variances as
   * they were then; each update sets them again.
   *
   * <p>A doubt of some meters adds its square to the camera's shift variance. The same doubt, seen
   * from as far as its tags were, is a turn of the doubt over that distance, and adds that turn's
   * square to the camera's turn variance. Until the distance has been judged, it adds no turn.
   */
  private void applyMountDoubts() {
    if (!started || mountDoubts.isEmpty()) {
      return;
    }
    for (Map.Entry<String, MountDoubt> told : mountDoubts.entrySet()) {
      int camera = places.cameraPlace(told.getKey());
      if (!ConsideredErrors.hasPlace(camera)) {
        continue;
      }
      MountDoubt doubt = told.getValue();
      double meters = doubt.meters();
      double turnRadians = doubt.tagDistanceMeters() > 0 ? meters / doubt.tagDistanceMeters() : 0;
      filter.setCameraVariances(
          camera,
          usualCameraTurnVariance + turnRadians * turnRadians,
          usualCameraShiftVariance + meters * meters);
    }
    mountDoubts.clear();
    history.store(history.newestStep(), filter);
  }

  // ---------------------------------------------------------------------------------------------
  // What the estimator says.
  // ---------------------------------------------------------------------------------------------

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
      return Optional.of(history.poseAt(timestampNanos, filter.headingOffset()));
    }
    // At the newest step or a little after it, the robot is taken to be where it is now.
    boolean tooFarAhead = timestampNanos > newestTime + futureToleranceNanos;
    if (tooFarAhead) {
      return Optional.empty();
    }
    return Optional.of(pose());
  }

  /** The current pose: cheap, for driving every loop. */
  public Pose2d pose() {
    return filter.pose(yaw);
  }

  /** The current estimate, with its uncertainty, velocity, mode, and health. */
  public LocalizationEstimate estimate() {
    long now = 0;
    OdometryHistory.Motion motion = OdometryHistory.Motion.NONE;
    double wheelResidualMeters = 0;
    if (started) {
      now = history.newestTime();
      motion = history.motionOver(VELOCITY_WINDOW_NANOS, filter.headingOffset());
      wheelResidualMeters = history.wheelResidual(history.newestStep());
    }
    // Before odometry starts, nothing has said the gyro is lost.
    boolean gyroConnected = !started || lastSource != YawSource.WHEELS;
    boolean everAccepted = lastAcceptedNanos != Long.MIN_VALUE;
    double visionAgeSeconds = Double.POSITIVE_INFINITY;
    if (everAccepted) {
      visionAgeSeconds = Timestamps.toSeconds(now - lastAcceptedNanos);
    }
    boolean camerasKeepDisagreeing = consecutiveRejections >= config.lostAfterRejections();
    boolean visionRecent = everAccepted && now - lastAcceptedNanos <= visionTimeoutNanos;
    return new LocalizationEstimate(
        now,
        pose(),
        filter.covariance(),
        motion.fieldVelocity(),
        motion.yawRateRadiansPerSecond(),
        mode(gyroConnected, placed, camerasKeepDisagreeing, visionRecent),
        new LocalizationEstimate.Health(
            visionAgeSeconds,
            acceptedCount,
            rejectedCount,
            lastInnovationMeters,
            largestRecentDisagreement(now),
            wheelResidualMeters,
            wheelDoubt.suspectTravelMeters(),
            wheelDoubt.conditions().tiltRadians(),
            started && wheelDoubt.inCollisionAt(now),
            recoveries,
            gyroConnected));
  }

  /**
   * The farthest any measurement accepted within the disagreement window was from the estimate, in
   * meters; NaN if none was accepted in it.
   */
  private double largestRecentDisagreement(long now) {
    long windowStart = now - disagreementWindowNanos;
    double largest = Double.NaN;
    for (Agreement agreement : agreements) {
      // Written so the first one replaces the NaN that "none yet" starts as.
      boolean largerThanAnyYet = !(agreement.meters() <= largest);
      if (agreement.time() >= windowStart && largerThanAnyYet) {
        largest = agreement.meters();
      }
    }
    return largest;
  }

  /**
   * What the estimate rests on, in order: a lost gyro, then not placed yet, then cameras that keep
   * disagreeing, then recent camera measurements, else odometry alone.
   *
   * @param lost whether the cameras have kept disagreeing, measurement after measurement
   * @param visionRecent whether a camera measurement was accepted recently
   */
  static LocalizationEstimate.Mode mode(
      boolean gyroConnected, boolean placed, boolean lost, boolean visionRecent) {
    if (!gyroConnected) {
      return LocalizationEstimate.Mode.IMU_FAULT;
    }
    if (!placed) {
      return LocalizationEstimate.Mode.INITIALIZING;
    }
    if (lost) {
      return LocalizationEstimate.Mode.LOST;
    }
    if (visionRecent) {
      return LocalizationEstimate.Mode.TRACKING;
    }
    return LocalizationEstimate.Mode.DEAD_RECKONING;
  }

  private static double square(double value) {
    return value * value;
  }
}
