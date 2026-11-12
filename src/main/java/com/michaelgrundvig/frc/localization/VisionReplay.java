package com.michaelgrundvig.frc.localization;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

/**
 * Applies camera measurements at the moments their images were taken, however late they arrive, so
 * they give the same result in any order of arrival. It rewinds to just before the earliest new
 * measurement and replays the history from there, applying the new measurements and the ones
 * applied before, in time order.
 *
 * <p>The filter is put back as it was after the last step before the earliest new measurement. Then
 * each later step is predicted through again. A measurement that falls inside a step splits it: the
 * filter predicts up to the measurement's moment, the measurement is applied, and the prediction
 * carries on from there. Each step's stored filter state is rewritten as the replay passes it.
 */
final class VisionReplay {
  /**
   * A camera's measurement and the moment the history applies it at.
   *
   * @param time when it is applied: its capture time, or the newest step's time if it was stamped a
   *     little after that
   */
  record Applied(long time, VisionMeasurement measurement) {}

  /**
   * A measurement to apply during a replay.
   *
   * @param newThisUpdate whether it arrived this update, or was applied before and is being applied
   *     again. Both are judged against the estimate as the replay has it. One applied before that
   *     an earlier image, new this update, now rules out is taken back, as it would have been
   *     rejected had the two come in order.
   */
  record Replayed(Applied applied, boolean newThisUpdate) {
    long time() {
      return applied.time();
    }
  }

  /** Judges and applies one measurement during a replay. */
  @FunctionalInterface
  interface Applier {
    /**
     * @param then what odometry says of the moment its image was taken
     * @param verdicts what became of each measurement, for the log
     * @return whether it was applied
     */
    boolean apply(Replayed replayed, OdometryAtImage then, List<MeasurementVerdict> verdicts);
  }

  private final OdometryHistory history;
  private final PoseFilter filter;
  private final OdometryPrediction prediction;
  private final CameraMounts mounts;

  /** How far after the newest step an image may be stamped and still be applied, at that step. */
  private final long futureToleranceNanos;

  /** The measurements the history has applied and still reaches back to, in time order. */
  private final List<Applied> applied = new ArrayList<>();

  /**
   * Measurements taken at or before this moment are from before the estimate was last reset. It is
   * set as odometry starts, and read only after.
   */
  private long lastResetNanos;

  VisionReplay(
      EstimatorConfig config,
      OdometryHistory history,
      PoseFilter filter,
      OdometryPrediction prediction,
      CameraMounts mounts) {
    futureToleranceNanos = Timestamps.fromSeconds(config.futureToleranceSeconds());
    this.history = history;
    this.filter = filter;
    this.prediction = prediction;
    this.mounts = mounts;
  }

  /**
   * Why a measurement's image was taken at a moment the history doesn't reach: before the estimate
   * was last reset, before the history's oldest step, or too far after its newest. Empty for one it
   * reaches. Call it once odometry has started.
   */
  Optional<Verdict> unreachable(VisionMeasurement measurement) {
    long taken = measurement.timestampNanos();
    if (taken <= lastResetNanos) {
      return Optional.of(Verdict.BEFORE_RESET);
    }
    if (taken <= history.oldestTime()) {
      return Optional.of(Verdict.STALE);
    }
    if (taken > history.newestTime() + futureToleranceNanos) {
      return Optional.of(Verdict.FROM_THE_FUTURE);
    }
    return Optional.empty();
  }

  /**
   * The history has just started, or started again from its newest step, with the filter as it is
   * now: nothing taken before it is applied.
   */
  void startedHere() {
    history.store(OdometryHistory.OLDEST_STEP, filter);
    lastResetNanos = history.oldestTime();
  }

  /** Drops the history: the newest step, with the filter as it is now, becomes its start. */
  void restart() {
    history.keepOnlyNewest();
    history.store(OdometryHistory.OLDEST_STEP, filter);
    applied.clear();
    lastResetNanos = history.oldestTime();
  }

  /**
   * Replays the history from just before the earliest of these measurements, applying them and the
   * ones applied since, in time order.
   */
  void replay(List<VisionMeasurement> usable, Applier applier, List<MeasurementVerdict> verdicts) {
    long newestTime = history.newestTime();
    long earliestNew = newestTime;
    for (VisionMeasurement measurement : usable) {
      earliestNew = Math.min(earliestNew, measurement.timestampNanos());
    }
    int startStep = history.lastStepBefore(earliestNew);
    List<Replayed> toApply = toApplyAfter(startStep, usable);

    history.restore(startStep, filter);
    int next = 0;
    for (int step = startStep + 1; step < history.stepCount(); step++) {
      long stepStart = history.endTime(step - 1);
      long stepEnd = history.endTime(step);
      double predictedUpTo = 0;
      while (next < toApply.size() && toApply.get(next).time() <= stepEnd) {
        Replayed replayed = toApply.get(next++);
        double fraction = (double) (replayed.time() - stepStart) / (stepEnd - stepStart);
        prediction.predict(step, predictedUpTo, fraction);
        predictedUpTo = fraction;
        OdometryAtImage then = OdometryAtImage.partWayThrough(history, step, fraction);
        applyOne(replayed, then, applier, verdicts);
      }
      prediction.predict(step, predictedUpTo, 1);
      prediction.learnDrift(step);
      mounts.reopenTurnsAt(step);
      history.store(step, filter);
    }
    // Anything left is at the starting step itself. That happens only when the history is a single
    // step, with no later step to split.
    while (next < toApply.size()) {
      OdometryAtImage then = OdometryAtImage.atTheOnlyStep(history, filter);
      applyOne(toApply.get(next++), then, applier, verdicts);
    }
    history.store(history.newestStep(), filter);
  }

  /**
   * Everything to apply after the starting step, in time order: what was applied there before, and
   * what is new. A measurement stamped a little after the newest step is applied at the newest
   * step. The replay adds back each one that is applied again.
   */
  private List<Replayed> toApplyAfter(int startStep, List<VisionMeasurement> usable) {
    long newestTime = history.newestTime();
    long startTime = history.endTime(startStep);
    List<Replayed> toApply = new ArrayList<>();
    for (Applied done : applied) {
      if (done.time() > startTime) {
        toApply.add(new Replayed(done, false));
      }
    }
    for (VisionMeasurement measurement : usable) {
      long appliedAt = Math.min(measurement.timestampNanos(), newestTime);
      toApply.add(new Replayed(new Applied(appliedAt, measurement), true));
    }
    toApply.sort(Comparator.comparingLong(Replayed::time));
    applied.removeIf(done -> done.time() > startTime);
    return toApply;
  }

  private void applyOne(
      Replayed replayed, OdometryAtImage then, Applier applier, List<MeasurementVerdict> verdicts) {
    if (applier.apply(replayed, then, verdicts)) {
      applied.add(replayed.applied());
    }
  }

  /** Forgets the measurements at or before this moment: the history no longer reaches them. */
  void forgetAtOrBefore(long timeNanos) {
    applied.removeIf(done -> done.time() <= timeNanos);
  }
}
