package com.michaelgrundvig.frc.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Predicate;
import org.wpilib.math.geometry.Translation2d;

/**
 * The estimator's camera side: each loop's measurements, refused if the history can't place them,
 * replayed in at the moments their images were taken ({@link VisionReplay}), gated ({@link
 * VisionGate}), and counted ({@link MeasurementBooks}). When the rejected ones agree, it recovers
 * to them ({@link Recovery}).
 *
 * <p>It also keeps what the cameras are judged against: whether the robot has been placed, and
 * whether its heading is a person's word no camera has backed yet ({@link HeadingDoubt}).
 */
final class VisionUpdate {
  private final EstimatorConfig config;
  private final Predicate<Translation2d> onField;
  private final long disagreementWindowNanos;
  private final PoseFilter filter;
  private final OdometryHistory history;
  private final ErrorPlaces places;
  private final VisionGate gate;
  private final CameraMissCorrection corrections;
  private final CameraMounts mounts;
  private final VisionReplay replay;
  private final WheelDoubt wheelDoubt;
  private final MotionEvidence motionEvidence;
  private final Recovery recovery;
  private final MeasurementBooks books = new MeasurementBooks();

  /** The camera measurements added since the last update. */
  private final List<VisionMeasurement> pending = new ArrayList<>();

  /** What the latest recovery did, and on what evidence. */
  private Optional<Recovered> lastRecovery = Optional.empty();

  /** Whether the robot has been placed: told where it is, or found by a camera. */
  private boolean placed;

  /**
   * Whether the heading is a person's word that no camera has backed yet: a placement's, or a
   * driver's reset. A camera backs it when its own heading agrees with it, or when the cameras'
   * headings turn the estimate. Until then a position solved with that heading cannot vote to move
   * the estimate: a wrong heading puts every such position off alike, and they agree.
   */
  private HeadingDoubt headingDoubt = HeadingDoubt.NONE;

  /** The estimator's parts the camera side works with. */
  record Parts(
      PoseFilter filter,
      OdometryHistory history,
      ErrorPlaces places,
      CameraMissCorrection corrections,
      CameraMounts mounts,
      VisionReplay replay,
      WheelDoubt wheelDoubt,
      MotionEvidence motionEvidence) {}

  VisionUpdate(EstimatorConfig config, Predicate<Translation2d> onField, Parts parts) {
    this.config = config;
    this.onField = onField;
    disagreementWindowNanos = Timestamps.fromSeconds(config.disagreementWindowSeconds());
    filter = parts.filter();
    history = parts.history();
    places = parts.places();
    corrections = parts.corrections();
    mounts = parts.mounts();
    replay = parts.replay();
    wheelDoubt = parts.wheelDoubt();
    motionEvidence = parts.motionEvidence();
    gate = new VisionGate(config, filter, places);
    recovery = new Recovery(config);
  }

  /** A camera measurement, to apply at the next update. */
  void add(VisionMeasurement measurement) {
    pending.add(measurement);
  }

  /** Refuses every measurement added since the last update, for one reason: odometry hasn't run. */
  List<MeasurementVerdict> refuseAll(Verdict verdict) {
    List<MeasurementVerdict> verdicts = new ArrayList<>();
    for (VisionMeasurement measurement : pending) {
      books.countRefusal();
      verdicts.add(
          new MeasurementVerdict(measurement, verdict, Double.NaN, Double.NaN, Double.NaN));
    }
    pending.clear();
    return verdicts;
  }

  /**
   * Applies the measurements added since the last update, each at its capture time, and recovers if
   * the cameras agree odometry is wrong. Call it once odometry has started.
   *
   * @return what happened to each measurement, for the log
   */
  List<MeasurementVerdict> update() {
    List<MeasurementVerdict> verdicts = new ArrayList<>();
    List<VisionMeasurement> usable = new ArrayList<>();
    for (VisionMeasurement measurement : pending) {
      Optional<Verdict> refusal = refusalOf(measurement);
      if (refusal.isPresent()) {
        books.countRefusal();
        verdicts.add(
            new MeasurementVerdict(measurement, refusal.get(), Double.NaN, Double.NaN, Double.NaN));
      } else {
        usable.add(measurement);
      }
    }
    pending.clear();
    if (!usable.isEmpty()) {
      replay.replay(usable, this::apply, verdicts);
    }
    mounts.reopenKnockedTurns();
    recoverToAgreeingCameras();
    mounts.applyDoubts();
    books.forgetAgreementsBefore(history.newestTime() - disagreementWindowNanos);
    return verdicts;
  }

  /**
   * Why a measurement cannot be applied at all, before it is ever compared with the estimate: its
   * image was taken at a moment the history does not reach, or it puts the robot off the field.
   * Empty for a measurement that can be applied.
   */
  private Optional<Verdict> refusalOf(VisionMeasurement measurement) {
    Optional<Verdict> unreachable = replay.unreachable(measurement);
    if (unreachable.isPresent()) {
      return unreachable;
    }
    if (!onField.test(measurement.pose().getTranslation())) {
      return Optional.of(Verdict.OFF_FIELD);
    }
    return Optional.empty();
  }

  /**
   * Compares a camera measurement with the estimate as it was when its image was taken, gates it,
   * keeps the books on what the gate said, and if it passes applies it as a Kalman update.
   *
   * @param then what odometry says of the moment its image was taken
   * @return whether it was applied
   */
  private boolean apply(
      VisionReplay.Replayed replayed, OdometryAtImage then, List<MeasurementVerdict> verdicts) {
    VisionMeasurement measurement = replayed.applied().measurement();
    int camera = places.cameraPlace(measurement.camera());
    filter.cameraCount(places.placedCameraCount());
    int[] tagPlaces = places.tagPlaces(measurement.tagIds());
    double headingThen = then.yaw() + filter.headingOffset();
    // A camera the watch has in doubt counts for less by that doubt, as more noise in each of its
    // measurements: what is taken off them lags the camera's real miss as its tags change.
    double doubt = corrections.doubtMeters(camera);
    double doubtVariance = doubt * doubt;
    Innovation asItCame =
        filter.innovation(measurement, then.yaw(), camera, tagPlaces, 0, 0, doubtVariance);
    CameraMissCorrection.Corrected corrected =
        corrections.correct(
            filter, measurement, camera, tagPlaces, then, headingThen, asItCame, doubtVariance);
    Innovation innovation = corrected.innovation();
    if (innovation == null) {
      // It can't be compared with the estimate at all, so it is left out, with a verdict of its
      // own. It is no camera disagreeing with the estimate, so it is no rejection in a row.
      books.countRefusal();
      verdicts.add(
          new MeasurementVerdict(
              measurement, Verdict.NOT_COMPARABLE, Double.NaN, Double.NaN, Double.NaN));
      return false;
    }
    VisionGate.Judgment judged = gate.judge(innovation, camera, headingDoubt);
    boolean withinGate = judged.verdict() != Verdict.GATED;
    if (withinGate && !replayed.newThisUpdate()) {
      // Applied before and passing again: it was counted when it first passed.
      filter.correct(innovation);
      return true;
    }
    keepBooks(replayed, innovation, judged, camera, verdicts);
    if (judged.verdict() != Verdict.ACCEPTED) {
      return false;
    }
    if (replayed.newThisUpdate()) {
      corrections.tellWatch(
          filter, camera, replayed.time(), measurement, innovation, corrected, then, headingThen);
    }
    filter.correct(innovation);
    return true;
  }

  /**
   * Counts what the gate said of a measurement. A camera's own heading that agrees with a person's
   * word backs it. One applied before that now fails is taken back, and counted as a rejection like
   * a new one.
   */
  private void keepBooks(
      VisionReplay.Replayed replayed,
      Innovation innovation,
      VisionGate.Judgment judged,
      int camera,
      List<MeasurementVerdict> verdicts) {
    books.countCompared(innovation.distanceMeters());
    if (judged.ownHeading() == VisionGate.OwnHeading.AGREES) {
      headingDoubt = HeadingDoubt.NONE;
    }
    if (judged.verdict() == Verdict.ACCEPTED) {
      mounts.checkForKnock(camera, innovation);
      countAcceptance(replayed, innovation);
    } else {
      countRejection(replayed, innovation);
    }
    verdicts.add(
        new MeasurementVerdict(
            replayed.applied().measurement(),
            judged.verdict(),
            innovation.distanceMeters(),
            judged.chiSquared(),
            judged.threshold()));
  }

  /** Counts a rejection, and hands recovery what it needs to judge whether the cameras agree. */
  private void countRejection(VisionReplay.Replayed replayed, Innovation innovation) {
    books.countRejection();
    VisionMeasurement measurement = replayed.applied().measurement();
    boolean headingInDoubt = headingDoubt != HeadingDoubt.NONE;
    recovery.rejected(
        new Recovery.Rejected(
            replayed.time(),
            measurement.camera(),
            measurement.tagIds(),
            innovation.xMeters(),
            innovation.yMeters(),
            innovation.headingRadians(),
            gate.positionSays(innovation, measurement, headingInDoubt)));
  }

  /** Counts an acceptance: the estimate rests on a camera again, and nothing needs recovering. */
  private void countAcceptance(VisionReplay.Replayed replayed, Innovation innovation) {
    books.countAcceptance(replayed.time(), innovation.distanceMeters());
    recovery.clear();
    motionEvidence.restedOnCamera(replayed.time());
    placed = true;
  }

  /**
   * When enough recent rejected measurements agree with each other, for independent reasons, the
   * cameras are right and odometry is wrong. The estimate turns to the heading they agree on, if
   * they agree its heading is wrong. Otherwise it jumps to where they put the robot. Either way its
   * uncertainty starts afresh.
   */
  private void recoverToAgreeingCameras() {
    recovery.forgetOlderThanTheWindow(history.newestTime());
    if (!recovery.hasEnough()) {
      return;
    }
    MotionEvidence.Sign sign = motionEvidence.sinceTheCameras();
    OptionalDouble turn = recovery.headingCorrection(sign, gate.widestHeadingAllowance());
    if (turn.isPresent()) {
      turnToTheCameras(turn.getAsDouble(), sign);
      return;
    }
    Optional<Recovery.Move> move = recovery.positionCorrection(sign);
    if (move.isPresent()) {
      moveToTheCameras(move.get(), sign);
    }
  }

  /**
   * Turns the estimate to the heading the cameras agree on. The books stay as they were, where a
   * position recovery keeps them anew. The turn leaves the position as unsure as a lost one, for
   * the next images to settle, so the estimate does not yet rest on a camera: its mode stays. What
   * besides the cameras said the robot may have moved stays too, for a position recovery that may
   * follow on one camera's word. A robot never placed is not placed by a heading alone.
   */
  private void turnToTheCameras(double turnRadians, MotionEvidence.Sign sign) {
    lastRecovery =
        Optional.of(
            new Recovered(
                Recovered.Correction.TURNED,
                Math.toDegrees(turnRadians),
                recovery.evidence(sign, false),
                recovery.judged()));
    filter.turn(
        turnRadians,
        square(config.lostPositionStdDevMeters()),
        square(Math.toRadians(config.recoveryHeadingStdDevDegrees())));
    headingDoubt = HeadingDoubt.NONE;
    restartAfterRecovery();
  }

  /**
   * Moves the estimate to where the cameras agree it is. The estimate is as recent as the images it
   * now rests on: a recovery made later, once an older image that held it back has aged out of the
   * window, must not make them new.
   */
  private void moveToTheCameras(Recovery.Move move, MotionEvidence.Sign sign) {
    Translation2d by = move.by();
    lastRecovery =
        Optional.of(
            new Recovered(
                Recovered.Correction.MOVED,
                by.getNorm(),
                recovery.evidence(sign, true),
                recovery.judged()));
    filter.shift(by.getX(), by.getY(), square(config.recoveryStdDevMeters()));
    restartAfterRecovery();
    books.restOnCamera(move.newestTaken());
    motionEvidence.restedOnCamera(move.newestTaken());
    placed = true;
  }

  /** After a recovery: the history starts again from it, and so do the doubts. */
  private void restartAfterRecovery() {
    restartHistory();
    wheelDoubt.forgetSuspectTravel();
    books.countRecovery();
    recovery.clear();
  }

  /** Drops the history: the newest step, with the filter as it is now, becomes its start. */
  void restartHistory() {
    replay.restart();
    books.forgetAgreements();
  }

  /**
   * The robot was placed, so whatever the cameras disagreed on before is forgotten. A placement
   * with a heading puts the heading in doubt until a camera backs it.
   */
  void placed(PoseReset reset) {
    placed = true;
    motionEvidence.placed();
    if (reset.covariance().hasHeading()) {
      headingDoubt = HeadingDoubt.PLACEMENT;
    }
    recovery.clear();
    books.forgetRejectionsInARow();
  }

  /** A driver reset the heading: it is in doubt until a camera backs it. */
  void headingReset() {
    headingDoubt = HeadingDoubt.RESET;
    motionEvidence.placed();
    recovery.clear();
    books.forgetRejectionsInARow();
  }

  /** Whose word the heading rests on while no camera has backed it; {@code NONE} once one has. */
  HeadingDoubt headingDoubt() {
    return headingDoubt;
  }

  /** Whether the robot has been placed: told where it is, or found by a camera. */
  boolean isPlaced() {
    return placed;
  }

  /** What the latest recovery did, and on what evidence; empty before the first. */
  Optional<Recovered> lastRecovery() {
    return lastRecovery;
  }

  MeasurementBooks books() {
    return books;
  }

  private static double square(double value) {
    return value * value;
  }
}
