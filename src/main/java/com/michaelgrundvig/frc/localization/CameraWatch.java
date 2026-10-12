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

  /** What each older measurement keeps of its weight when a new one comes: 1 - 1/memory. */
  private final double kept;

  /**
   * What a camera that sits where it was calibrated may still be off by, as the filter already
   * allows for: its turn on its mount, in radians, and its shift, in meters, each as a variance.
   */
  private final double allowedTurnVariance;

  private final double allowedShiftVariance;

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
   *     the filter already allows for: the watch looks only for more than that could be
   * @param allowedShiftStdDevMeters and how far it may be shifted
   */
  CameraWatch(int memory, double allowedTurnStdDevRadians, double allowedShiftStdDevMeters) {
    if (memory < 2 * FEWEST_TO_JUDGE) {
      throw new IllegalArgumentException(
          "judge a camera by at least " + (int) (2 * FEWEST_TO_JUDGE) + " measurements: " + memory);
    }
    kept = 1 - 1.0 / memory;
    allowedTurnVariance = allowedTurnStdDevRadians * allowedTurnStdDevRadians;
    allowedShiftVariance = allowedShiftStdDevMeters * allowedShiftStdDevMeters;
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
   */
  void accepted(
      int camera,
      long takenNanos,
      double missX,
      double missY,
      double headingRadians,
      double tagDistanceMeters,
      double claimedVariance,
      double odometryVariance) {
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
    judge(camera);
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
   * Judges a camera by its average: how far its measurements are off, which is the average of how
   * far apart its misses were, as far as that stands clear of what a sound camera could show.
   *
   * <pre>
   * average   = the weighted mean of how far apart its misses were
   * noise²    = their variance about that mean, along each axis / how many
   * allowed²  = its shift's allowance² + (its turn's allowance × its tags' distance)²
   * off       = average × sqrt(1 - 3² (noise² + allowed²) / |average|²), or nothing where that is
   *             negative
   * </pre>
   *
   * <p>The noise is judged by the camera's own scatter, not by the uncertainty each image claimed:
   * an image's claim is a cautious one, several times its real scatter at a distance, and judged by
   * it a knocked camera would go unnoticed for many seconds. Their variance about the mean is the
   * mean of their squares less the square of the mean, over both axes together, so half of it is
   * each axis's.
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
    double meanSquared = meanForward * meanForward + meanLeft * meanLeft;
    double scatterBothAxes = Math.max(0, apartSquaredSum[camera] / n - meanSquared);
    double varianceOfMeanEachAxis = scatterBothAxes / 2 / n;
    double tagDistance = distanceSum[camera] / n;
    double allowedSquared = allowedShiftVariance + allowedTurnVariance * tagDistance * tagDistance;
    double soundSquared =
        CLEAR_OF_NOISE * CLEAR_OF_NOISE * (varianceOfMeanEachAxis + allowedSquared);
    // Written so an average that is not a number counts as no doubt.
    boolean clearOfASoundCamera = meanSquared > soundSquared;
    double share = clearOfASoundCamera ? Math.sqrt(1 - soundSquared / meanSquared) : 0;
    offForward[camera] = share * meanForward;
    offLeft[camera] = share * meanLeft;
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
   * How far a camera is taken to put a measurement off along the field's x, in meters, with the
   * robot facing a heading: what the estimator takes off its measurements before it weighs them.
   */
  double offX(int camera, double headingRadians) {
    if (!ConsideredErrors.hasPlace(camera)) {
      return 0;
    }
    return Math.cos(headingRadians) * offForward[camera]
        - Math.sin(headingRadians) * offLeft[camera];
  }

  /** And along y. */
  double offY(int camera, double headingRadians) {
    if (!ConsideredErrors.hasPlace(camera)) {
      return 0;
    }
    return Math.sin(headingRadians) * offForward[camera]
        + Math.cos(headingRadians) * offLeft[camera];
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
