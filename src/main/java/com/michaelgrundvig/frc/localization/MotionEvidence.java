package com.michaelgrundvig.frc.localization;

/**
 * Whether anything other than the cameras says the robot may have been moved where its wheels could
 * not see, since the estimate last rested on a camera: what lets a single camera's word move the
 * estimate in a recovery ({@link Recovery}).
 *
 * <p>A camera that disagrees with the estimate is either right, and odometry was wrong, or wrong
 * itself. One camera repeating one answer cannot say which: a tag misread as another puts every
 * image off by the same distance, in the same direction, however the robot drives, exactly as a
 * shove the wheels never felt would. What can say which is a sensor that has nothing to do with the
 * camera. So this keeps the newest sign from one:
 *
 * <ul>
 *   <li>the robot was placed, or its heading reset, and no camera has backed that since: where it
 *       stands rests on a person's word;
 *   <li>the IMU felt a jolt the wheels' grip cannot explain (a collision, or a share of one);
 *   <li>the IMU saw the wheels slide over the floor;
 *   <li>the robot tilted past where its wheels' travel counts in full, so they may have left the
 *       floor;
 *   <li>the robot was disabled, when people pick it up and carry it.
 * </ul>
 *
 * <p>A sign counts until a camera's image taken after it agrees with the estimate: then whatever it
 * was did not move the robot past what the estimate allows. An image taken before it does not count
 * against it, however late it arrives.
 *
 * <p>It holds plain numbers and allocates nothing.
 */
final class MotionEvidence {
  /** What says the robot may have moved unseen, if anything. */
  enum Sign {
    /** Nothing: only the cameras could say it moved. */
    NONE,
    /** Placed, or its heading reset, and no camera has backed it since. */
    PLACED,
    /** The IMU felt a jolt the wheels' grip cannot explain. */
    HIT,
    /** The IMU saw the wheels slide over the floor. */
    SLID,
    /** Tilted past where the wheels' travel counts in full. */
    LIFTED,
    /** Disabled, when people may carry it. */
    DISABLED
  }

  /** The newest sign felt, and the step it came after: it happened later than that. */
  private Sign newest = Sign.NONE;

  private long newestNanos = Long.MIN_VALUE;

  /** Whether the robot's placement, or its heading reset, has not been backed by a camera yet. */
  private boolean placementUnbacked;

  /** When the newest image the estimate rests on was taken. */
  private long restedNanos = Long.MIN_VALUE;

  /**
   * Takes in what the robot's other sensors say about the odometry steps to come.
   *
   * @param nanos when the newest odometry step ended: what they say happened after it
   */
  void conditions(DriveConditions conditions, double shareOfTravelThatCounts, long nanos) {
    Sign sign = signIn(conditions, shareOfTravelThatCounts);
    if (sign != Sign.NONE) {
      newest = sign;
      newestNanos = nanos;
    }
  }

  /**
   * The strongest sign a loop's conditions give, if any. A collision says most, a slide less, and a
   * robot standing disabled least, since it says only that it could have been carried.
   */
  private static Sign signIn(DriveConditions conditions, double shareOfTravelThatCounts) {
    if (conditions.collisionShare() > 0) {
      return Sign.HIT;
    }
    if (conditions.slipMetersPerSecond() > 0) {
      return Sign.SLID;
    }
    if (shareOfTravelThatCounts < 1) {
      return Sign.LIFTED;
    }
    if (conditions.disabled()) {
      return Sign.DISABLED;
    }
    return Sign.NONE;
  }

  /** The robot was placed, or its heading reset: until a camera backs it, it is a person's word. */
  void placed() {
    placementUnbacked = true;
  }

  /** The estimate rests on an image taken at this moment: one accepted, or one recovered to. */
  void restedOnCamera(long takenNanos) {
    restedNanos = Math.max(restedNanos, takenNanos);
    placementUnbacked = false;
  }

  /** What says the robot may have moved unseen since the estimate last rested on a camera. */
  Sign sinceTheCameras() {
    if (placementUnbacked) {
      return Sign.PLACED;
    }
    // An image taken as the step ended was taken before what came after the step.
    boolean feltSince = newest != Sign.NONE && newestNanos >= restedNanos;
    if (feltSince) {
      return newest;
    }
    return Sign.NONE;
  }
}
