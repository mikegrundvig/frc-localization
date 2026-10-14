package com.michaelgrundvig.frc.localization;

/**
 * Whether one camera's measurements keep landing to one side of where the other cameras' land: the
 * estimator's own, quick watch on each camera's mount.
 *
 * <p>Each measurement the estimator accepts is set against where odometry alone puts the robot: the
 * estimate as it stood a second or so ago, carried on by the wheels and the gyro. It misses that by
 * a little. A camera knocked on its mount misses the same way every time, in the robot's frame: its
 * mount moved with the robot, so its error turns with the robot.
 *
 * <p>What the wheels or the gyro get wrong puts every camera's measurements off alike, and by an
 * amount that changes from one moment to the next while the wheels slip. So each miss is judged
 * against the misses the other cameras made at about the same moment: the camera's own miss, less
 * the middle of the voices heard just then. The voices are every camera's newest miss, this one
 * among them, and odometry's, which misses by nothing, each counting by how sure it claims to be.
 * With two cameras seeing tags, one of them knocked, odometry is what tells them apart: the sound
 * one still agrees with where the wheels carried the robot, and the knocked one does not. Wheels
 * that slip count for little, by the estimator's own doubt of them, and are outvoted, since every
 * camera then misses alike.
 *
 * <p>The watch keeps, for each camera, a running average of how far apart its misses were, over its
 * latest measurements, the older ones counting for less. For a camera that sits where its
 * configuration says, they point every which way and average out. What is left of the average,
 * beyond what noise alone would leave in an average of that many, is how far the camera is in doubt
 * ({@link #doubtMeters}).
 *
 * <p>The estimator takes that much off the camera's measurements before it weighs them ({@link
 * #offX}), and adds as much doubt to what the camera repeats in every image. So within a second a
 * knocked camera's measurements land about where the others' do, and count for less, while the
 * others hold the estimate. The misses the watch is given are the measurements' own, before
 * anything is taken off, so the average holds while the camera stays knocked. If its misses stop
 * (it was pushed back), the average falls, and the doubt and what is taken off go with it. A camera
 * that sees no tag for a while, or sees them alone, keeps the doubt it had, until its own
 * measurements beside another camera's say otherwise. A robot with one camera has nothing to judge
 * it against, and it is never doubted here.
 *
 * <p><b>A camera's clock.</b> A camera whose images are stamped a little late misses another way:
 * by how far the robot moved and turned in that time. An image stamped late by a time t is set
 * against where the robot was t after it was taken, so it misses by the robot's velocity times t,
 * backward; and it was solved with the heading of t later, which swings it about its tags by the
 * turn rate times t. Both grow with how the robot moves, and are nothing at rest. So beside the
 * average, the watch fits how a camera's misses follow the robot's motion: the time that,
 * multiplied by how far a second of lateness would put each image off (its late arm), best explains
 * them. That time is the camera's lateness ({@link #lateSeconds}), and what it explains is taken
 * off the camera's measurements with the rest.
 *
 * <p>A place is a camera's number among the errors the filter keeps apart ({@link
 * ConsideredErrors}). The sums are kept in one array for each kind, by place, so that noting a
 * measurement makes no object.
 */
final class CameraWatch {
  /**
   * How many standard deviations a camera's average must stand clear of zero before any of it
   * counts as doubt: of its noise, and of what a calibrated camera's mount is allowed. The length
   * of an average that is only noise, in two dimensions, is past three of its per-axis standard
   * deviations about once in a hundred: e^(-3²/2) = 0.011.
   */
  private static final double CLEAR_OF_NOISE = 3;

  /**
   * A camera is judged once its average rests on this many measurements, in effect. Its noise is
   * judged by its own scatter, and the scatter of fewer than about ten says little: a variance from
   * nine degrees of freedom is within a factor of two of the truth only nineteen times in twenty.
   */
  private static final double FEWEST_TO_JUDGE = 10;

  /**
   * Two cameras' misses are of the same moment when their images were taken within this of each
   * other: a tenth of a second, two or three images of a camera at 30 a second. In that time even
   * wheels sliding a meter a second move odometry's place by centimeters, not by the tens of
   * centimeters a slide adds up to.
   */
  private static final long SAME_MOMENT_NANOS = 100_000_000L;

  /**
   * A voice that claims to be surer than this, as a variance in square meters, counts as if it
   * claimed this: a tenth of a millimeter, so that a claim of no doubt at all does not make a
   * weight that is not a number.
   */
  private static final double LEAST_VARIANCE = 1e-8;

  /**
   * How unsure a camera's lateness is before its misses have said anything of it, as a variance: 50
   * ms squared, a frame or two, which is what a coprocessor whose time sync failed is off by. It
   * only keeps a fit made from very little motion from saying much.
   */
  private static final double LATE_VARIANCE_BEFORE = 0.05 * 0.05;

  /** What each older measurement keeps of its weight when a new one comes: 1 - 1/memory. */
  private final double kept;

  /**
   * What a camera may still be off by that the filter already allows for: its turn on its mount, in
   * radians, by its place, and its shift, in meters, each as a variance. The filter estimates each
   * camera's turn, and tells the watch how sure of it it is now ({@link #allowTurn}).
   */
  private final double[] allowedTurnVariance = new double[ConsideredErrors.MAX_CAMERAS];

  private final double allowedShiftVariance;

  /**
   * How late or early a camera's stamps may be while its time sync works, as a variance, in seconds
   * squared: the lateness's allowance.
   */
  private final double allowedLateVariance;

  // For each camera, by place: its newest miss, and sums over how far apart its misses were, each
  // weighted by how recent it is. Everything is in the robot's frame, forward and left, in meters.

  /** When each camera's newest measurement was taken: before any, long before any clock's time. */
  private final long[] heardNanos = new long[ConsideredErrors.MAX_CAMERAS];

  private final double[] newestMissForward = new double[ConsideredErrors.MAX_CAMERAS];
  private final double[] newestMissLeft = new double[ConsideredErrors.MAX_CAMERAS];

  /** How much each camera's newest miss counts among the voices: one over its claimed variance. */
  private final double[] newestMissWeight = new double[ConsideredErrors.MAX_CAMERAS];

  /** The sum of the weights: how many measurements the average rests on, in effect. */
  private final double[] weight = new double[ConsideredErrors.MAX_CAMERAS];

  private final double[] apartForwardSum = new double[ConsideredErrors.MAX_CAMERAS];
  private final double[] apartLeftSum = new double[ConsideredErrors.MAX_CAMERAS];

  /** The squared lengths of how far apart its misses were, in square meters. */
  private final double[] apartSquaredSum = new double[ConsideredErrors.MAX_CAMERAS];

  /** How far the measurement's tags were from the robot, in meters. */
  private final double[] distanceSum = new double[ConsideredErrors.MAX_CAMERAS];

  // The late arms of its measurements: how far a second of lateness would have put each one off,
  // forward and left, in meters a second.
  private final double[] lateArmForwardSum = new double[ConsideredErrors.MAX_CAMERAS];
  private final double[] lateArmLeftSum = new double[ConsideredErrors.MAX_CAMERAS];

  /** The late arms' squared lengths. */
  private final double[] lateArmSquaredSum = new double[ConsideredErrors.MAX_CAMERAS];

  /** Each late arm's dot product with how far apart its miss was. */
  private final double[] lateArmDotApartSum = new double[ConsideredErrors.MAX_CAMERAS];

  /**
   * The lateness each camera's own misses fit, in seconds, and its variance, as last judged: before
   * the middle of all the cameras' is taken off it.
   */
  private final double[] fittedLate = new double[ConsideredErrors.MAX_CAMERAS];

  private final double[] fittedLateVariance = new double[ConsideredErrors.MAX_CAMERAS];

  /** How late each camera's images are taken to be stamped, in seconds, as last judged. */
  private final double[] late = new double[ConsideredErrors.MAX_CAMERAS];

  /**
   * How far each camera is taken to put its measurements off, as last judged: along the robot's
   * forward and along its left, in meters. Its length is the camera's doubt.
   */
  private final double[] offForward = new double[ConsideredErrors.MAX_CAMERAS];

  private final double[] offLeft = new double[ConsideredErrors.MAX_CAMERAS];

  /**
   * The voices' misses along one axis, in order, and how much each counts, while their middle is
   * found: every camera's and odometry's. This object's own.
   */
  private final double[] inOrder = new double[ConsideredErrors.MAX_CAMERAS + 1];

  private final double[] inOrderWeight = new double[ConsideredErrors.MAX_CAMERAS + 1];

  /**
   * @param memory how many of a camera's latest measurements it is judged by, in effect: each new
   *     one takes 1/memory off the weight of all before it
   * @param allowedTurnStdDevRadians how far a calibrated camera may be turned on its mount, which
   *     the filter already allows for: the watch looks only for more than that could be. It holds
   *     for each camera until the filter says otherwise ({@link #allowTurn})
   * @param allowedShiftStdDevMeters and how far it may be shifted
   * @param allowedLateStdDevSeconds how late or early a camera's stamps may be while its time sync
   *     works: the watch looks only for more than that could be
   */
  CameraWatch(
      int memory,
      double allowedTurnStdDevRadians,
      double allowedShiftStdDevMeters,
      double allowedLateStdDevSeconds) {
    if (memory < 2 * FEWEST_TO_JUDGE) {
      throw new IllegalArgumentException(
          "judge a camera by at least " + (int) (2 * FEWEST_TO_JUDGE) + " measurements: " + memory);
    }
    kept = 1 - 1.0 / memory;
    java.util.Arrays.fill(allowedTurnVariance, allowedTurnStdDevRadians * allowedTurnStdDevRadians);
    allowedShiftVariance = allowedShiftStdDevMeters * allowedShiftStdDevMeters;
    allowedLateVariance = allowedLateStdDevSeconds * allowedLateStdDevSeconds;
    java.util.Arrays.fill(heardNanos, Long.MIN_VALUE / 2);
  }

  /**
   * Notes one accepted measurement's miss, and judges its camera anew if another camera's miss of
   * the same moment is there to judge it against.
   *
   * @param camera the camera's place
   * @param takenNanos when its image was taken
   * @param missX how far the measurement was from where odometry alone puts the robot, along the
   *     field's x, in meters, before anything was taken off it
   * @param missY and along y
   * @param headingRadians the robot's heading when the image was taken, which turns the miss into
   *     the robot's frame
   * @param tagDistanceMeters how far its tags were from the robot
   * @param claimedVariance how unsure the measurement claims its position is, as a variance along
   *     each axis, in square meters
   * @param odometryVariance how unsure odometry's place is, the same way
   * @param lateArmX how far a second of lateness in its stamp would have put it off along the
   *     field's x, in meters a second ({@link #lateArmX})
   * @param lateArmY and along y
   */
  void accepted(
      int camera,
      long takenNanos,
      double missX,
      double missY,
      double headingRadians,
      double tagDistanceMeters,
      double claimedVariance,
      double odometryVariance,
      double lateArmX,
      double lateArmY) {
    if (!ConsideredErrors.hasPlace(camera)) {
      return;
    }
    double cos = Math.cos(headingRadians);
    double sin = Math.sin(headingRadians);
    // The field's miss, turned back by the heading, is the miss in the robot's frame.
    newestMissForward[camera] = cos * missX + sin * missY;
    newestMissLeft[camera] = -sin * missX + cos * missY;
    newestMissWeight[camera] = 1 / Math.max(claimedVariance, LEAST_VARIANCE);
    heardNanos[camera] = takenNanos;

    int voices = voicesOfTheMoment(takenNanos);
    boolean anotherCameraToJudgeBy = voices >= 2;
    if (!anotherCameraToJudgeBy) {
      return;
    }
    double odometryWeight = 1 / Math.max(odometryVariance, LEAST_VARIANCE);
    double apartForward =
        newestMissForward[camera]
            - middleOfTheMoment(newestMissForward, takenNanos, odometryWeight);
    double apartLeft =
        newestMissLeft[camera] - middleOfTheMoment(newestMissLeft, takenNanos, odometryWeight);
    weight[camera] = kept * weight[camera] + 1;
    apartForwardSum[camera] = kept * apartForwardSum[camera] + apartForward;
    apartLeftSum[camera] = kept * apartLeftSum[camera] + apartLeft;
    apartSquaredSum[camera] =
        kept * apartSquaredSum[camera] + apartForward * apartForward + apartLeft * apartLeft;
    distanceSum[camera] = kept * distanceSum[camera] + tagDistanceMeters;
    double lateArmForward = cos * lateArmX + sin * lateArmY;
    double lateArmLeft = -sin * lateArmX + cos * lateArmY;
    lateArmForwardSum[camera] = kept * lateArmForwardSum[camera] + lateArmForward;
    lateArmLeftSum[camera] = kept * lateArmLeftSum[camera] + lateArmLeft;
    lateArmSquaredSum[camera] =
        kept * lateArmSquaredSum[camera]
            + lateArmForward * lateArmForward
            + lateArmLeft * lateArmLeft;
    lateArmDotApartSum[camera] =
        kept * lateArmDotApartSum[camera] + lateArmForward * apartForward + lateArmLeft * apartLeft;
    judge(camera);
  }

  /**
   * How far a camera may be turned on its mount that the filter already allows for, from now on, as
   * a variance in radians squared: the doubt of the filter's own estimate of its turn. The misses
   * the watch is given have that estimate taken off, so what is left is either within this doubt,
   * which is the filter's to learn, or the watch's to find.
   */
  void allowTurn(int camera, double variance) {
    if (ConsideredErrors.hasPlace(camera)) {
      allowedTurnVariance[camera] = variance;
    }
  }

  /**
   * Forgets what the watch has judged of a camera's mount, and the misses it judged it by: its turn
   * has just been opened up again, and its misses from before say nothing of its mount now. Its
   * lateness stays as judged until its next misses say otherwise.
   */
  void forgetMount(int camera) {
    if (!ConsideredErrors.hasPlace(camera)) {
      return;
    }
    weight[camera] = 0;
    apartForwardSum[camera] = 0;
    apartLeftSum[camera] = 0;
    apartSquaredSum[camera] = 0;
    distanceSum[camera] = 0;
    lateArmForwardSum[camera] = 0;
    lateArmLeftSum[camera] = 0;
    lateArmSquaredSum[camera] = 0;
    lateArmDotApartSum[camera] = 0;
    offForward[camera] = 0;
    offLeft[camera] = 0;
  }

  /** Whether a camera's newest miss is of the same moment as an image taken at a time. */
  private boolean isOfTheMoment(int camera, long takenNanos) {
    return Math.abs(takenNanos - heardNanos[camera]) <= SAME_MOMENT_NANOS;
  }

  /** How many cameras have a miss of the same moment as an image taken at a time. */
  private int voicesOfTheMoment(long takenNanos) {
    int voices = 0;
    for (int camera = 0; camera < ConsideredErrors.MAX_CAMERAS; camera++) {
      if (isOfTheMoment(camera, takenNanos)) {
        voices++;
      }
    }
    return voices;
  }

  /**
   * The middle of the voices' misses along one axis: the newest miss of every camera heard at this
   * moment, and odometry's, which is zero. Each voice counts by how sure it claims to be, one over
   * its variance, and the middle is their weighted median: the miss with half of all the weight at
   * or below it and half at or above it, or the mean of the two that share that.
   *
   * <p>So one camera far off does not move it. Neither does odometry while the wheels are in doubt,
   * when it counts for little, nor while surer cameras agree it is off. And two cameras far from
   * their tags, each claiming little and each off its own way, do not outvote odometry.
   */
  private double middleOfTheMoment(double[] newestMiss, long takenNanos, double odometryWeight) {
    // Odometry's voice goes in first.
    inOrder[0] = 0;
    inOrderWeight[0] = odometryWeight;
    int count = 1;
    double allTheWeight = odometryWeight;
    for (int camera = 0; camera < ConsideredErrors.MAX_CAMERAS; camera++) {
      if (isOfTheMoment(camera, takenNanos)) {
        // Each miss goes in after the larger ones before it, which keeps them in order.
        int at = count++;
        while (at > 0 && inOrder[at - 1] > newestMiss[camera]) {
          inOrder[at] = inOrder[at - 1];
          inOrderWeight[at] = inOrderWeight[at - 1];
          at--;
        }
        inOrder[at] = newestMiss[camera];
        inOrderWeight[at] = newestMissWeight[camera];
        allTheWeight += newestMissWeight[camera];
      }
    }
    double half = allTheWeight / 2;
    // From the bottom, the first miss with half the weight at or below it; from the top, the
    // first with half at or above it. They are the same miss unless two share the middle.
    int lower = 0;
    double below = inOrderWeight[0];
    while (below < half) {
      below += inOrderWeight[++lower];
    }
    int upper = count - 1;
    double above = inOrderWeight[upper];
    while (above < half) {
      above += inOrderWeight[--upper];
    }
    return (inOrder[lower] + inOrder[upper]) / 2;
  }

  /**
   * How far a second of lateness in an image's stamp puts its measurement off along the field's x,
   * in meters a second: its late arm's x. {@link #lateArmY} gives its y.
   *
   * <p>An image stamped t late is set against where the robot was t after it was taken, so it
   * misses by the velocity times t, backward. And it was solved with the heading the robot had t
   * later, which is the turn rate times t further round: a position solved with a heading that much
   * too far lands that angle times its lever, a quarter turn back from it. With the lever (lx, ly),
   * the velocity (vx, vy) and the turn rate w, a second of lateness puts it off by
   *
   * <pre>(w ly - vx, -w lx - vy)</pre>
   *
   * @param leverY from the robot to the measurement's tags along the field's y, in meters
   * @param velocityX the robot's velocity along the field's x when the image was taken
   * @param yawRate how fast the robot was turning then, counterclockwise, in radians a second
   */
  static double lateArmX(double leverY, double velocityX, double yawRate) {
    return yawRate * leverY - velocityX;
  }

  /** The late arm's y: see {@link #lateArmX}. */
  static double lateArmY(double leverX, double velocityY, double yawRate) {
    return -yawRate * leverX - velocityY;
  }

  /**
   * Judges a camera by how far apart its misses have been: how late its images are stamped, and how
   * far its measurements are off the same way every time.
   *
   * <p>The fit of {@code apart = off + late × late arm} is the line through the weighted means
   * whose slope is the late arms' covariance with the misses over the late arms' own spread:
   *
   * <pre>
   * spread = mean(|arm|²) - |mean(arm)|²
   * late   = (mean(arm · apart) - mean(arm) · mean(apart)) / spread
   * off    = mean(apart) - late × mean(arm)
   * </pre>
   *
   * <p>What that fit leaves of the misses is their noise, over both axes together, so half of it is
   * each axis's. The misses tell of the lateness with an information of the summed spread over one
   * measurement's noise, and what was known before any miss adds its own, one over its variance. So
   * the lateness fitted is the line's, weighted by the misses' share of the information, and its
   * variance is one over the two informations together. A robot that barely moves leaves it near
   * zero and as unsure as before.
   *
   * <p><b>Its lateness</b> is the lateness its misses fit, less the middle of all the cameras'
   * ({@link #judgeLateness}).
   *
   * <p><b>Its off</b> is kept only as far as it stands clear of what a sound camera could show:
   * times {@code sqrt(1 - 3² variance / its own square)}, or nothing where that is negative. The
   * variance is, along each axis,
   *
   * <pre>
   * noise²    = one measurement's noise / how many
   * allowed²  = its shift's allowance² + (its turn's allowance × its tags' distance)²
   * </pre>
   *
   * <p>and, for the part of the off that lies along the mean late arm, the fitted lateness's
   * variance times the mean arm's square besides: a lateness not yet known could be what puts a
   * robot that drives steadily one way off along its travel, and could not put it off to one side.
   *
   * <p>The noise is judged by the camera's own scatter, not by the uncertainty each image claimed:
   * an image's claim is a cautious one, several times its real scatter at a distance, and judged by
   * it a knocked camera would go unnoticed for many seconds.
   *
   * <p>The allowance is what the filter already takes a calibrated camera's mount to be off by. A
   * camera within three of its standard deviations is left to the filter. So is a small error of
   * the heading, which puts each camera's measurements a little off another way, as each being a
   * little turned would: the filter corrects the heading from just that, and a watch that took it
   * for the cameras' own would take it away.
   */
  private void judge(int camera) {
    double n = weight[camera];
    if (n < FEWEST_TO_JUDGE) {
      return;
    }
    double meanForward = apartForwardSum[camera] / n;
    double meanLeft = apartLeftSum[camera] / n;
    double armForward = lateArmForwardSum[camera] / n;
    double armLeft = lateArmLeftSum[camera] / n;
    double meanArmSquared = armForward * armForward + armLeft * armLeft;
    double spread = Math.max(0, lateArmSquaredSum[camera] / n - meanArmSquared);
    double armWithApart =
        lateArmDotApartSum[camera] / n - armForward * meanForward - armLeft * meanLeft;
    double lineLate = spread > 0 ? armWithApart / spread : 0;
    double leftOverBothAxes =
        apartSquaredSum[camera] / n
            - meanForward * meanForward
            - meanLeft * meanLeft
            - lineLate * lineLate * spread;
    double noiseEachAxis = Math.max(0, leftOverBothAxes) / 2;

    double informationBefore = 1 / LATE_VARIANCE_BEFORE;
    double informationFromMisses = informationOfLateness(n * spread, noiseEachAxis);
    double lateSeconds = 0;
    if (informationFromMisses > 0) {
      // Written as one over one plus the ratio, so that misses with no noise at all, whose
      // information is infinite, keep the whole of the line's lateness.
      lateSeconds = lineLate / (1 + informationBefore / informationFromMisses);
    }
    double lateVariance = 1 / (informationBefore + informationFromMisses);
    fittedLate[camera] = lateSeconds;
    fittedLateVariance[camera] = lateVariance;
    judgeLateness();

    double forward = meanForward - lateSeconds * armForward;
    double left = meanLeft - lateSeconds * armLeft;
    double tagDistance = distanceSum[camera] / n;
    double allowedSquared =
        allowedShiftVariance + allowedTurnVariance[camera] * tagDistance * tagDistance;
    double soundVariance = noiseEachAxis / n + allowedSquared;
    // What is unsure of the lateness makes the off unsure only along the mean late arm, the way
    // a lateness moves a measurement. So the off is kept in two parts: the part along the mean
    // arm, which has that doubt as well, and the part across it, which does not.
    double armLength = Math.sqrt(meanArmSquared);
    double alongForward = armLength > 0 ? armForward / armLength : 1;
    double alongLeft = armLength > 0 ? armLeft / armLength : 0;
    double along = forward * alongForward + left * alongLeft;
    double acrossForward = forward - along * alongForward;
    double acrossLeft = left - along * alongLeft;
    double alongKept =
        along * shareClearOf(along * along, soundVariance + lateVariance * meanArmSquared);
    double acrossShare =
        shareClearOf(acrossForward * acrossForward + acrossLeft * acrossLeft, soundVariance);
    offForward[camera] = alongKept * alongForward + acrossShare * acrossForward;
    offLeft[camera] = alongKept * alongLeft + acrossShare * acrossLeft;
  }

  /**
   * Sets every judged camera's lateness: the lateness its own misses fit, less the middle of all
   * the judged cameras', as far as that stands clear of noise and of what a camera's stamps may be
   * off by while its time sync works.
   *
   * <p>The misses are taken from odometry's place, and what the wheels and the gyro get wrong by a
   * share of the robot's motion, a worn tread or a gyro's scale, puts that place off just as a
   * lateness would put a camera's. That shows as the same lateness in every camera, so the middle
   * of them, their median, is odometry's doing and is taken off. With one camera judged there is
   * nothing to tell its lateness from odometry's error, and it has none. With two that differ, each
   * is half the difference from their middle: nothing says which of them is late.
   */
  private void judgeLateness() {
    int count = 0;
    for (int camera = 0; camera < ConsideredErrors.MAX_CAMERAS; camera++) {
      if (weight[camera] >= FEWEST_TO_JUDGE) {
        // Each goes in after the larger ones before it, which keeps them in order.
        int at = count++;
        while (at > 0 && inOrder[at - 1] > fittedLate[camera]) {
          inOrder[at] = inOrder[at - 1];
          at--;
        }
        inOrder[at] = fittedLate[camera];
      }
    }
    int middle = count / 2;
    boolean oddCount = count % 2 == 1;
    double middleLate = oddCount ? inOrder[middle] : (inOrder[middle - 1] + inOrder[middle]) / 2;
    for (int camera = 0; camera < ConsideredErrors.MAX_CAMERAS; camera++) {
      if (weight[camera] >= FEWEST_TO_JUDGE) {
        double own = fittedLate[camera] - middleLate;
        late[camera] =
            own * shareClearOf(own * own, fittedLateVariance[camera] + allowedLateVariance);
      }
    }
  }

  /**
   * How much the misses tell of a camera's lateness: the late arms' summed spread over one
   * measurement's noise. None where the arms do not spread at all, and without limit where they do
   * and there is no noise.
   */
  private static double informationOfLateness(double summedSpread, double noiseEachAxis) {
    if (summedSpread <= 0) {
      return 0;
    }
    return noiseEachAxis > 0 ? summedSpread / noiseEachAxis : Double.POSITIVE_INFINITY;
  }

  /**
   * How much of a fitted value to keep, for its square and the variance of what a sound camera
   * could show: all of it but what three standard deviations of that could be, as a share from 0 to
   * 1.
   */
  private static double shareClearOf(double valueSquared, double variance) {
    double soundSquared = CLEAR_OF_NOISE * CLEAR_OF_NOISE * variance;
    // Written so a value that is not a number is not kept at all.
    boolean clear = valueSquared > soundSquared;
    return clear ? Math.sqrt(1 - soundSquared / valueSquared) : 0;
  }

  /**
   * How late a camera's images are taken to be stamped, in seconds, as last judged: negative for
   * images stamped early. Zero for a camera that has never been judged.
   */
  double lateSeconds(int camera) {
    return ConsideredErrors.hasPlace(camera) ? late[camera] : 0;
  }

  /**
   * How far a camera's measurements may be off from where its mount moved, in meters, as last
   * judged. Zero for a camera that has never been judged.
   */
  double doubtMeters(int camera) {
    if (!ConsideredErrors.hasPlace(camera)) {
      return 0;
    }
    return Math.hypot(offForward[camera], offLeft[camera]);
  }

  /**
   * How far a camera is taken to put a measurement off along the field's x, in meters: what the
   * estimator takes off its measurements before it weighs them. It is what the camera's mount puts
   * every measurement off by, turned onto the field by the robot's heading, and what its lateness
   * puts this one off by, which is the lateness times the measurement's late arm.
   *
   * @param headingRadians the robot's heading when the image was taken
   * @param lateArmX how far a second of lateness would put this measurement off along the field's x
   *     ({@link #lateArmX})
   */
  double offX(int camera, double headingRadians, double lateArmX) {
    if (!ConsideredErrors.hasPlace(camera)) {
      return 0;
    }
    return Math.cos(headingRadians) * offForward[camera]
        - Math.sin(headingRadians) * offLeft[camera]
        + late[camera] * lateArmX;
  }

  /** And along y. */
  double offY(int camera, double headingRadians, double lateArmY) {
    if (!ConsideredErrors.hasPlace(camera)) {
      return 0;
    }
    return Math.sin(headingRadians) * offForward[camera]
        + Math.cos(headingRadians) * offLeft[camera]
        + late[camera] * lateArmY;
  }

  /**
   * How far a camera's tags were from the robot, on average, over the measurements it is judged by:
   * a mount turned by an angle puts its measurements that angle times this off.
   *
   * @return NaN for a camera with no measurements to judge it by
   */
  double distanceMeters(int camera) {
    if (!ConsideredErrors.hasPlace(camera) || weight[camera] == 0) {
      return Double.NaN;
    }
    return distanceSum[camera] / weight[camera];
  }
}
