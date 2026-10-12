package com.michaelgrundvig.frc.localization;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * Judges whether the cameras, while they disagree with the estimate, agree with each other. When
 * they do, odometry is the one that is wrong (a collision, a spin, a bad heading reset), and the
 * estimate should go to the cameras. This class keeps the recent rejected measurements'
 * disagreements and says what the latest of them agree on. The estimator does the moving.
 *
 * <p>The heading is settled before the position. Positions solved with the estimate's own heading
 * agree with each other whether it is the position or the heading that is wrong: a heading off by
 * an angle turns every one of them about its tags alike. So while the heading is in doubt those
 * positions do not vote on the position ({@link Rejected#positionCounts}), and the cameras' own
 * headings settle the heading first.
 */
final class Recovery {
  private final int measurementsNeeded;
  private final long windowNanos;
  private final double agreementMeters;
  private final double agreementRadians;

  /** The rejections still inside the window, oldest first. */
  private final ArrayDeque<Rejected> recent = new ArrayDeque<>();

  /** The latest few rejections of one kind, newest first, while they are judged. */
  private final List<Rejected> latest = new ArrayList<>();

  /**
   * A rejected measurement's disagreement with the estimate.
   *
   * @param time when its image was taken
   * @param innovationX how far its position was from the estimate's along the field's x, in meters
   * @param innovationY and along y
   * @param innovationHeading how far its heading was from the estimate's, in radians; NaN when the
   *     measurement had no heading of its own ({@link #hasHeading})
   * @param positionAgreed whether its position alone passed the gate, so that only its heading
   *     disagreed
   * @param positionCounts whether its position can say the estimate's position is wrong. One solved
   *     with a heading in doubt cannot: it lands off by that heading's error however right the
   *     position is, and the same way in every image that shares it
   */
  record Rejected(
      long time,
      double innovationX,
      double innovationY,
      double innovationHeading,
      boolean positionAgreed,
      boolean positionCounts) {
    boolean hasHeading() {
      return !Double.isNaN(innovationHeading);
    }

    /** Whether it is evidence that the estimate's position is wrong. */
    boolean saysThePositionIsWrong() {
      return !positionAgreed && positionCounts;
    }
  }

  /**
   * A move of the estimate's position that the cameras agree on.
   *
   * @param by how far, on the field
   * @param newestTaken when the newest image that agreed on it was taken: the estimate then rests
   *     on images that old, however much later the move is made
   */
  record Move(Translation2d by, long newestTaken) {}

  Recovery(EstimatorConfig config) {
    measurementsNeeded = config.recoveryMeasurements();
    windowNanos = Timestamps.fromSeconds(config.recoveryWindowSeconds());
    agreementMeters = config.recoveryAgreementMeters();
    agreementRadians = Math.toRadians(config.recoveryAgreementDegrees());
  }

  void rejected(Rejected rejected) {
    recent.addLast(rejected);
  }

  /** Forgets every rejection: a measurement was accepted, or the estimate was reset. */
  void clear() {
    recent.clear();
  }

  /** Forgets the rejections that have fallen out of the window, as of now. */
  void forgetOlderThanTheWindow(long nowNanos) {
    long windowStart = nowNanos - windowNanos;
    while (!recent.isEmpty() && recent.getFirst().time() < windowStart) {
      recent.removeFirst();
    }
  }

  /** Whether enough rejections are left for them to agree on anything. */
  boolean hasEnough() {
    return recent.size() >= measurementsNeeded;
  }

  /**
   * The turn the estimate's heading needs, if the latest rejected measurements that have headings
   * agree on one: the newest few of them, as many as a recovery needs, every one within the
   * agreement angle of their mean, and that mean not the estimate's own heading. Their positions
   * were worked out with the estimate's heading, so the position is left to the next measurement.
   *
   * <p>Only the latest count. A robot carried by hand is seen from a new place in every image, and
   * the images taken on the way disagree with each other. What matters is whether the cameras agree
   * now.
   *
   * @return the turn in radians, or empty if they do not agree on one
   */
  OptionalDouble headingCorrection() {
    latest.clear();
    for (Iterator<Rejected> newestFirst = recent.descendingIterator();
        newestFirst.hasNext() && latest.size() < measurementsNeeded; ) {
      Rejected rejected = newestFirst.next();
      if (rejected.hasHeading()) {
        latest.add(rejected);
      }
    }
    if (latest.size() < measurementsNeeded) {
      return OptionalDouble.empty();
    }
    // The mean is taken of each heading's difference from the first one, wrapped to half a turn
    // either way, so that headings either side of half a turn average rightly.
    double first = latest.get(0).innovationHeading();
    double sumOfDifferencesFromFirst = 0;
    for (Rejected rejected : latest) {
      sumOfDifferencesFromFirst += MathUtil.angleModulus(rejected.innovationHeading() - first);
    }
    double mean = first + sumOfDifferencesFromFirst / latest.size();
    for (Rejected rejected : latest) {
      double fromMean = Math.abs(MathUtil.angleModulus(rejected.innovationHeading() - mean));
      if (fromMean > agreementRadians) {
        return OptionalDouble.empty();
      }
    }
    // Headings that agree with the estimate's ask for no turn: what was rejected was their
    // positions. Written so that a mean that is not a number asks for none either.
    boolean asksForATurn = Math.abs(MathUtil.angleModulus(mean)) > agreementRadians;
    if (!asksForATurn) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(mean);
  }

  /**
   * The move the estimate's position needs, if the latest rejected measurements agree on one: the
   * newest few that say the position is wrong, as many as a recovery needs, every one within the
   * agreement distance of their mean. Older ones that disagree do not hold it back: they were taken
   * while the robot was still being moved.
   */
  Optional<Move> positionCorrection() {
    latest.clear();
    for (Iterator<Rejected> newestFirst = recent.descendingIterator();
        newestFirst.hasNext() && latest.size() < measurementsNeeded; ) {
      Rejected rejected = newestFirst.next();
      if (rejected.saysThePositionIsWrong()) {
        latest.add(rejected);
      }
    }
    if (latest.size() < measurementsNeeded) {
      return Optional.empty();
    }
    double meanX = 0;
    double meanY = 0;
    long newestTaken = Long.MIN_VALUE;
    for (Rejected rejected : latest) {
      meanX += rejected.innovationX();
      meanY += rejected.innovationY();
      newestTaken = Math.max(newestTaken, rejected.time());
    }
    meanX /= latest.size();
    meanY /= latest.size();
    for (Rejected rejected : latest) {
      double fromMean = Math.hypot(rejected.innovationX() - meanX, rejected.innovationY() - meanY);
      if (fromMean > agreementMeters) {
        return Optional.empty();
      }
    }
    return Optional.of(new Move(new Translation2d(meanX, meanY), newestTaken));
  }
}
