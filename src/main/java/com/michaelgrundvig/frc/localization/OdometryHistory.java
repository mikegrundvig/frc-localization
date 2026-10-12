package com.michaelgrundvig.frc.localization;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The pose estimator's memory of how the robot drove: its recent odometry steps, oldest first, each
 * kept with the filter's state as it was once that step had been applied.
 *
 * <p>The estimator needs it because a camera's image arrives late. To apply a measurement at the
 * moment its image was taken, the estimator puts the filter back as it was after the last step
 * before that moment ({@link #restore}), applies the measurement, and replays every step since.
 *
 * <p><b>Steps and slots.</b> Two kinds of number index this class, and they are easy to confuse. A
 * <em>step number</em> is what callers use: {@link #OLDEST_STEP} is the oldest step kept and {@link
 * #newestStep()} the latest, so a step's number goes down by one whenever an older step is dropped.
 * A <em>slot</em> is where a step's values sit in the arrays, which are used as a ring so that
 * dropping the oldest step moves nothing. Only {@link #slotOf} turns a step number into a slot, and
 * no slot ever leaves this class.
 *
 * <p>A step covers the stretch of time from the step before it to its own time. The oldest step
 * kept is only a starting point: its travel belongs to a stretch the history no longer holds, and
 * nothing reads it.
 *
 * <p>The values are kept in one array for each kind of value, not in an object for each step,
 * because a step is added at the odometry rate (250 Hz on a robot) and nothing may be allocated
 * there.
 */
final class OdometryHistory {
  /** The number of the oldest step kept. */
  static final int OLDEST_STEP = 0;

  /**
   * How the robot moved lately.
   *
   * @param fieldVelocity its velocity over the field, in meters a second
   * @param yawRateRadiansPerSecond how fast it turned, counterclockwise
   */
  record Motion(Translation2d fieldVelocity, double yawRateRadiansPerSecond) {
    /** A robot that has not moved, or a history too short to say how it did. */
    static final Motion NONE = new Motion(Translation2d.ZERO, 0);
  }

  private final int capacity;

  /** The slot of the oldest step kept. */
  private int oldestSlot;

  /** How many steps are kept: at least one once {@link #start} has been called. */
  private int stepCount;

  // What each step was, fixed when it was added. Every array is indexed by slot.

  /** When each step ended, on the robot's clock. */
  private final long[] endTimeNanos;

  /**
   * How far the wheels carried the robot over each step, forward and to the left in the robot's
   * frame as the step began, with whatever the estimator discounted for a tilt already taken off.
   */
  private final double[] travelForwardMeters;

  private final double[] travelLeftMeters;

  /** How far the wheels' disagreement with each other may have put each step's travel off. */
  private final double[] wheelResidualMeters;

  /**
   * The position variance each step adds beyond what its distance and turning add, for slip, tilt
   * and collisions. It is worked out when the step arrives and kept, so a replay weighs the step
   * exactly as it was first weighed.
   */
  private final double[] addedPositionVariance;

  /** What turned the robot over each step: an IMU, or the wheels standing in for one. */
  private final YawSource[] turnedBy;

  /** The unwrapped yaw as each step ended, in radians. */
  private final double[] yawAfterRadians;

  // The filter's state once each step had been applied. A replay rewrites these.

  private final double[] filterXAfter;
  private final double[] filterYAfter;
  private final double[] filterHeadingOffsetAfter;

  /**
   * The filter's uncertainty after each step, one step after another: for each slot, the {@link
   * #uncertaintyLength} numbers that {@link PoseFilter#copyUncertaintyTo} writes.
   */
  private final double[] filterUncertaintyAfter;

  private final int uncertaintyLength;

  /**
   * @param capacity how many steps it keeps before each new one drops the oldest
   * @param uncertaintyLength how many numbers the filter's uncertainty takes to store ({@link
   *     PoseFilter#uncertaintyLength()})
   */
  OdometryHistory(int capacity, int uncertaintyLength) {
    this.capacity = capacity;
    this.uncertaintyLength = uncertaintyLength;
    endTimeNanos = new long[capacity];
    travelForwardMeters = new double[capacity];
    travelLeftMeters = new double[capacity];
    wheelResidualMeters = new double[capacity];
    addedPositionVariance = new double[capacity];
    turnedBy = new YawSource[capacity];
    yawAfterRadians = new double[capacity];
    filterXAfter = new double[capacity];
    filterYAfter = new double[capacity];
    filterHeadingOffsetAfter = new double[capacity];
    filterUncertaintyAfter = new double[capacity * uncertaintyLength];
  }

  /**
   * Starts the history over with a single step, the moment odometry starts from. That step has no
   * travel: it only says when the robot was first read and what its yaw was then.
   */
  void start(long timestampNanos, YawSource source, double yawRadians) {
    oldestSlot = 0;
    stepCount = 1;
    endTimeNanos[oldestSlot] = timestampNanos;
    turnedBy[oldestSlot] = source;
    yawAfterRadians[oldestSlot] = yawRadians;
  }

  /**
   * Adds the newest step. When the history is full, the oldest step is dropped to make room, and
   * every remaining step's number goes down by one.
   *
   * @param timestampNanos when the step ended
   * @param forwardMeters how far the wheels carried the robot forward, as much of it as counts
   * @param leftMeters and to the left
   * @param residualMeters how far the wheels' disagreement may have put that travel off
   * @param positionVariance the position variance the step adds for slip, tilt and collisions
   * @param source what turned the robot over this step: an IMU, or the wheels standing in
   * @param yawRadians the unwrapped yaw as the step ended
   * @return the new step's number, which is the newest
   */
  int add(
      long timestampNanos,
      double forwardMeters,
      double leftMeters,
      double residualMeters,
      double positionVariance,
      YawSource source,
      double yawRadians) {
    if (stepCount == capacity) {
      dropOldestStep();
    }
    stepCount++;
    int slot = slotOf(newestStep());
    endTimeNanos[slot] = timestampNanos;
    travelForwardMeters[slot] = forwardMeters;
    travelLeftMeters[slot] = leftMeters;
    wheelResidualMeters[slot] = residualMeters;
    addedPositionVariance[slot] = positionVariance;
    turnedBy[slot] = source;
    yawAfterRadians[slot] = yawRadians;
    return newestStep();
  }

  /** How many steps are kept. Their numbers run from {@link #OLDEST_STEP} to this less one. */
  int stepCount() {
    return stepCount;
  }

  /** The number of the latest step. */
  int newestStep() {
    return stepCount - 1;
  }

  /** When a step ended. */
  long endTime(int step) {
    return endTimeNanos[slotOf(step)];
  }

  long oldestTime() {
    return endTime(OLDEST_STEP);
  }

  long newestTime() {
    return endTime(newestStep());
  }

  /** How long a step lasted: from the end of the step before it to its own end. */
  long durationNanos(int step) {
    return endTime(step) - endTime(step - 1);
  }

  double travelForward(int step) {
    return travelForwardMeters[slotOf(step)];
  }

  double travelLeft(int step) {
    return travelLeftMeters[slotOf(step)];
  }

  double wheelResidual(int step) {
    return wheelResidualMeters[slotOf(step)];
  }

  double addedPositionVariance(int step) {
    return addedPositionVariance[slotOf(step)];
  }

  YawSource turnedBy(int step) {
    return turnedBy[slotOf(step)];
  }

  /** The unwrapped yaw as a step ended. */
  double yawAfter(int step) {
    return yawAfterRadians[slotOf(step)];
  }

  /** How far the robot turned over a step. */
  double turnOver(int step) {
    return yawAfter(step) - yawAfter(step - 1);
  }

  /**
   * The yaw part of the way through a step, taking the robot to have turned evenly over it.
   *
   * @param fraction how far through the step: 0 as it begins, 1 as it ends
   */
  double yawPartWayThrough(int step, double fraction) {
    double yawBefore = yawAfter(step - 1);
    return yawBefore + fraction * (yawAfter(step) - yawBefore);
  }

  /** The last step that ended before a moment, or the oldest step if none did. */
  int lastStepBefore(long timestampNanos) {
    int step = newestStep();
    while (step > OLDEST_STEP && endTime(step) >= timestampNanos) {
      step--;
    }
    return step;
  }

  /** Keeps the filter's state as it is now, as its state after a step. */
  void store(int step, PoseFilter filter) {
    int slot = slotOf(step);
    filterXAfter[slot] = filter.x();
    filterYAfter[slot] = filter.y();
    filterHeadingOffsetAfter[slot] = filter.headingOffset();
    filter.copyUncertaintyTo(filterUncertaintyAfter, slot * uncertaintyLength);
  }

  /** Puts the filter back in the state it was in after a step. */
  void restore(int step, PoseFilter filter) {
    int slot = slotOf(step);
    filter.restore(
        filterXAfter[slot],
        filterYAfter[slot],
        filterHeadingOffsetAfter[slot],
        filterUncertaintyAfter,
        slot * uncertaintyLength);
  }

  /** Drops every step but the newest, which becomes the oldest: the history starts from it. */
  void keepOnlyNewest() {
    oldestSlot = slotOf(newestStep());
    stepCount = 1;
  }

  /**
   * Drops the steps that are no longer needed to reach back to a moment. The oldest step is the
   * starting point for everything after it, so it can go only once the step after it ended at or
   * before that moment and can be the starting point instead. The newest step is always kept.
   */
  void dropStepsNotNeededSince(long cutoffNanos) {
    int secondOldest = OLDEST_STEP + 1;
    while (stepCount > 1 && endTime(secondOldest) <= cutoffNanos) {
      dropOldestStep();
    }
  }

  /**
   * Where the robot was at a moment between the oldest step and the newest, as the stored filter
   * states have it. The position is taken along a straight line between the states either side of
   * the moment, and the yaw likewise.
   *
   * @param headingOffsetRadians the filter's heading offset now, which is added to the yaw then:
   *     the heading is given as the estimate understands it today
   */
  Pose2d poseAt(long timestampNanos, double headingOffsetRadians) {
    // The step the moment falls in is the earliest one whose predecessor ended before the moment.
    int step = newestStep();
    while (step > OLDEST_STEP && endTime(step - 1) >= timestampNanos) {
      step--;
    }
    if (step == OLDEST_STEP) {
      int slot = slotOf(OLDEST_STEP);
      return new Pose2d(
          filterXAfter[slot],
          filterYAfter[slot],
          new Rotation2d(yawAfterRadians[slot] + headingOffsetRadians));
    }
    int before = slotOf(step - 1);
    int after = slotOf(step);
    double fraction =
        (double) (timestampNanos - endTimeNanos[before])
            / (endTimeNanos[after] - endTimeNanos[before]);
    return new Pose2d(
        filterXAfter[before] + fraction * (filterXAfter[after] - filterXAfter[before]),
        filterYAfter[before] + fraction * (filterYAfter[after] - filterYAfter[before]),
        new Rotation2d(yawPartWayThrough(step, fraction) + headingOffsetRadians));
  }

  /**
   * How the robot moved over the latest steps: those that ended within a window of time before the
   * newest. Each step's travel is turned onto the field by the heading the robot had as the step
   * began, and the total is divided by the time those steps took.
   *
   * @param headingOffsetRadians the filter's heading offset, which makes a yaw a field heading
   */
  Motion motionOver(long windowNanos, double headingOffsetRadians) {
    long newestTime = newestTime();
    long windowStart = newestTime - windowNanos;
    double movedXMeters = 0;
    double movedYMeters = 0;
    double turnedRadians = 0;
    long earliestTime = newestTime;
    for (int step = newestStep(); step > OLDEST_STEP && endTime(step) > windowStart; step--) {
      int slot = slotOf(step);
      int previous = slotOf(step - 1);
      double heading = yawAfterRadians[previous] + headingOffsetRadians;
      double forward = travelForwardMeters[slot];
      double left = travelLeftMeters[slot];
      movedXMeters += Math.cos(heading) * forward - Math.sin(heading) * left;
      movedYMeters += Math.sin(heading) * forward + Math.cos(heading) * left;
      turnedRadians += yawAfterRadians[slot] - yawAfterRadians[previous];
      earliestTime = endTimeNanos[previous];
    }
    double seconds = Timestamps.toSeconds(newestTime - earliestTime);
    boolean anyTimePassed = seconds > 0;
    if (!anyTimePassed) {
      return Motion.NONE;
    }
    return new Motion(
        new Translation2d(movedXMeters / seconds, movedYMeters / seconds), turnedRadians / seconds);
  }

  private void dropOldestStep() {
    oldestSlot = slotOf(OLDEST_STEP + 1);
    stepCount--;
  }

  /** Where a step's values sit in the arrays. */
  private int slotOf(int step) {
    return (oldestSlot + step) % capacity;
  }
}
