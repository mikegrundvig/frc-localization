package com.michaelgrundvig.frc.localization;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A robot pose worked out from a camera's view of AprilTags, ready for the estimator: where the
 * robot was when the image was taken, how uncertain that is, and where the pose came from, so every
 * accepted or rejected measurement can be traced to its camera, tags, and solver.
 *
 * @param timestampNanos when the image was captured; the estimator applies the measurement at that
 *     moment, not when it arrived
 * @param pose the robot's pose on the field
 * @param covariance its uncertainty. A solve that took its heading from the gyro says nothing new
 *     about heading, so its heading variance is infinite.
 * @param lever how its position leans on the gyro's heading: a solve that took its heading from the
 *     gyro placed the robot by turning its view of the tags by that heading, so a heading off by an
 *     angle puts the position off by that angle times the distance to the tags, square to them, the
 *     same way in every image. The estimator carries that in the measurement's uncertainty. {@link
 *     Lever#NONE} for a solve that found its own heading (multi-tag, single tag, external).
 * @param camera which camera
 * @param tagIds the tags it used
 * @param tagShares how much each of them counts in its position, in the same order, together 1: a
 *     tag off the field's layout puts the position off by its share of how far it's off, the same
 *     in every image any camera takes of it, and the estimator keeps that apart for each tag
 * @param solver how it was solved
 */
public record VisionMeasurement(
    long timestampNanos,
    Pose2d pose,
    PoseCovariance covariance,
    Lever lever,
    String camera,
    List<Integer> tagIds,
    List<Double> tagShares,
    Solver solver) {
  /** How a pose was solved from the tags. */
  public enum Solver {
    /** Position only, with the heading taken from the gyro: the default, and the most robust. */
    HEADING_CONSTRAINED,
    /** Every visible tag solved at once, on the coprocessor. */
    MULTI_TAG,
    /** One tag alone: ambiguous unless the heading settles it. */
    SINGLE_TAG,
    /** A pose the vision system solved its own way, such as a Limelight's. */
    EXTERNAL
  }

  /**
   * How a position solved with the gyro's heading leans on it.
   *
   * @param heading the heading it was solved with: the estimate's, as of when it was solved
   * @param robotToTags from the robot's center to the tags it was solved from, on the field: their
   *     middle, weighted as the solve weighed them
   */
  public record Lever(Rotation2d heading, Translation2d robotToTags) {
    /** A solve that found its own heading: its position doesn't lean on the gyro's. */
    public static final Lever NONE = new Lever(Rotation2d.ZERO, Translation2d.ZERO);

    public Lever {
      if (!Double.isFinite(robotToTags.getX())
          || !Double.isFinite(robotToTags.getY())
          || !Double.isFinite(heading.getRadians())) {
        throw new IllegalArgumentException(
            "a lever's heading and offset to its tags: " + heading + ", " + robotToTags);
      }
    }
  }

  public VisionMeasurement {
    if (timestampNanos < 0) {
      throw new IllegalArgumentException("a time on the robot's clock: " + timestampNanos);
    }
    if (camera.isBlank()) {
      throw new IllegalArgumentException("a vision measurement needs its camera's name");
    }
    if (tagIds.isEmpty()) {
      throw new IllegalArgumentException("a vision measurement comes from at least one tag");
    }
    if (tagShares.size() != tagIds.size()) {
      throw new IllegalArgumentException(
          "a share for each of a measurement's tags: " + tagShares + " for " + tagIds);
    }
    double total = 0;
    for (double share : tagShares) {
      if (!(share >= 0) || Double.isInfinite(share)) {
        throw new IllegalArgumentException("a tag's share is zero or more: " + tagShares);
      }
      total += share;
    }
    if (!(total > 0)) {
      throw new IllegalArgumentException("a measurement's tags share its position: " + tagShares);
    }

    tagIds = List.copyOf(tagIds);
    List<Double> shares = new ArrayList<>(tagShares.size());
    for (double share : tagShares) {
      shares.add(share / total);
    }
    tagShares = List.copyOf(shares);
  }

  /** A measurement whose tags count equally in its position. */
  public VisionMeasurement(
      long timestampNanos,
      Pose2d pose,
      PoseCovariance covariance,
      Lever lever,
      String camera,
      List<Integer> tagIds,
      Solver solver) {
    this(
        timestampNanos,
        pose,
        covariance,
        lever,
        camera,
        tagIds,
        Collections.nCopies(tagIds.size(), 1.0),
        solver);
  }
}
