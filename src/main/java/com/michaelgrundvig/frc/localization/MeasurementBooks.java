package com.michaelgrundvig.frc.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.OptionalLong;

/**
 * The books the estimator keeps on its camera measurements, for its mode and health: how many it
 * accepted and rejected, how many in a row it rejected, when the estimate last rested on a camera,
 * and how far the measurements it compared were from the estimate.
 *
 * <p>"None yet" is an empty optional here. The estimate's health says it as its own fields
 * document: NaN, or an infinite age.
 */
final class MeasurementBooks {
  /** How far an accepted measurement was from the estimate, and when its image was taken. */
  private record Agreement(long time, double meters) {}

  /** How far each recently accepted measurement was from the estimate. */
  private final List<Agreement> agreements = new ArrayList<>();

  /** When the newest image the estimate rests on was taken. */
  private OptionalLong restedOnCamera = OptionalLong.empty();

  /** How far the latest measurement the gate judged was from the estimate. */
  private OptionalDouble lastInnovationMeters = OptionalDouble.empty();

  private int rejectionsInARow;
  private int accepted;
  private int rejected;
  private int recoveries;

  /** Counts a measurement that was never compared with the estimate, or couldn't be. */
  void countRefusal() {
    rejected++;
  }

  /** Notes how far a measurement the gate judged was from the estimate. */
  void countCompared(double innovationMeters) {
    lastInnovationMeters = OptionalDouble.of(innovationMeters);
  }

  /** Counts a measurement the gate rejected: a camera disagreeing with the estimate. */
  void countRejection() {
    rejected++;
    rejectionsInARow++;
  }

  /** Counts an accepted measurement: the estimate rests on a camera again. */
  void countAcceptance(long time, double innovationMeters) {
    accepted++;
    rejectionsInARow = 0;
    restOnCamera(time);
    agreements.add(new Agreement(time, innovationMeters));
  }

  /** The estimate rests on an image taken then, unless it already rests on a newer one. */
  void restOnCamera(long time) {
    long newest = time;
    if (restedOnCamera.isPresent()) {
      newest = Math.max(restedOnCamera.getAsLong(), time);
    }
    restedOnCamera = OptionalLong.of(newest);
  }

  /** Counts a recovery to what the cameras agree on. */
  void countRecovery() {
    recoveries++;
    rejectionsInARow = 0;
  }

  /** Forgets the rejections in a row: the estimate was placed again, or turned. */
  void forgetRejectionsInARow() {
    rejectionsInARow = 0;
  }

  /** Forgets how far the accepted measurements were: the history starts again. */
  void forgetAgreements() {
    agreements.clear();
  }

  /** Forgets the agreements of images taken before this moment. */
  void forgetAgreementsBefore(long windowStart) {
    agreements.removeIf(agreement -> agreement.time() < windowStart);
  }

  /** Whether the cameras have kept disagreeing, measurement after measurement. */
  boolean camerasKeepDisagreeing(int lostAfterRejections) {
    return rejectionsInARow >= lostAfterRejections;
  }

  /** Whether a camera measurement the estimate rests on was taken within the timeout of now. */
  boolean visionRecent(long nowNanos, long timeoutNanos) {
    if (restedOnCamera.isEmpty()) {
      return false;
    }
    return nowNanos - restedOnCamera.getAsLong() <= timeoutNanos;
  }

  /**
   * Since the newest image the estimate rests on was taken, in seconds: infinite if none yet, as
   * the health documents it.
   */
  double visionAgeSeconds(long nowNanos) {
    if (restedOnCamera.isEmpty()) {
      return Double.POSITIVE_INFINITY;
    }
    return Timestamps.toSeconds(nowNanos - restedOnCamera.getAsLong());
  }

  /** How far the latest judged measurement was from the estimate: NaN if none yet. */
  double lastInnovationMeters() {
    return lastInnovationMeters.orElse(Double.NaN);
  }

  /**
   * The farthest any measurement accepted at or after this moment was from the estimate, in meters:
   * NaN if none was.
   */
  double largestDisagreementSince(long windowStart) {
    OptionalDouble largest = OptionalDouble.empty();
    for (Agreement agreement : agreements) {
      if (agreement.time() < windowStart) {
        continue;
      }
      if (largest.isEmpty() || agreement.meters() > largest.getAsDouble()) {
        largest = OptionalDouble.of(agreement.meters());
      }
    }
    return largest.orElse(Double.NaN);
  }

  int accepted() {
    return accepted;
  }

  int rejected() {
    return rejected;
  }

  int recoveries() {
    return recoveries;
  }
}
