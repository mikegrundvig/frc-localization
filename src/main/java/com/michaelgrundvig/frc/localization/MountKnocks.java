package com.michaelgrundvig.frc.localization;

/**
 * Whether a camera's mount may have been knocked since its turn was learned, so that what the
 * estimate learned of its turn must be opened up again and learned anew.
 *
 * <p>The estimate learns each camera's turn on its mount slowly, as a steady thing, and then holds
 * it tightly. A camera knocked mid-match is turned by a new amount at once, and the estimate, sure
 * of the old one, would follow its images only slowly, and let them pull the heading meanwhile. The
 * sign is in the camera's own headings: a knocked camera turns every heading it gives by the knock,
 * image after image, at once.
 *
 * <p>So each camera's heading misses (its heading less the estimate's, with its learned turn taken
 * off) are averaged, the older ones counting for less, and their scatter is kept beside the
 * average. While the camera sits where the estimate has it, the misses scatter about zero, by the
 * camera's noise and the estimate's own doubt of the heading the camera should see. Once their
 * average stands clear of both, by {@link #CLEAR_OF_NOISE} of their standard deviations, the camera
 * has turned on its mount since. The scatter is the misses' own, not what the images claim: an
 * image's claim is a cautious one, many times its real scatter, and judged by it a knocked camera
 * would go unnoticed for a long while.
 *
 * <p>A knock is only a sign, not a verdict: the camera's turn is opened up by what its mount allows
 * and by how far its headings turned, and its images then teach the filter its turn as at the
 * start. A shove, a wheel's slip or a tag off the layout moves where a camera puts the robot, not
 * the camera's own heading, and is no sign. A camera misreading a tag is refused by the gate, and a
 * refused image says nothing here.
 *
 * <p>A place is a camera's number among the errors the filter keeps apart ({@link
 * ConsideredErrors}). The sums are kept in one array for each kind, by place, so that noting a
 * heading makes no object.
 */
final class MountKnocks {
  /**
   * How many standard deviations a camera's average heading miss must stand clear of what the
   * estimate allows before it is a sign: past three is about one chance in four hundred for a miss
   * that is only noise.
   */
  private static final double CLEAR_OF_NOISE = 3;

  /**
   * A camera's heading misses are judged once their average rests on this many, in effect: the
   * scatter of fewer than about ten says little of their noise, as for the watch on the cameras.
   */
  private static final double FEWEST_TO_JUDGE = 10;

  /** What each older heading miss keeps of its weight when a new one comes: 1 - 1/memory. */
  private final double kept;

  // For each camera, by place: sums over its latest headings' misses, each weighted by how recent
  // it
  // is, in radians. The weights' own sum is how many misses the sums rest on, in effect.
  private final double[] missWeight = new double[ConsideredErrors.MAX_CAMERAS];
  private final double[] missSum = new double[ConsideredErrors.MAX_CAMERAS];
  private final double[] missSquaredSum = new double[ConsideredErrors.MAX_CAMERAS];

  /**
   * @param memory how many of a camera's latest headings its heading misses are judged by, in
   *     effect: each new one takes 1/memory off the weight of all before it. The watch on the
   *     cameras' own ({@link EstimatorConfig#cameraWatchMeasurements}).
   */
  MountKnocks(int memory) {
    if (memory < 2 * FEWEST_TO_JUDGE) {
      throw new IllegalArgumentException(
          "judge a camera by at least " + (int) (2 * FEWEST_TO_JUDGE) + " headings: " + memory);
    }
    kept = 1 - 1.0 / memory;
  }

  /**
   * A camera's image was accepted: how far its heading missed the estimate's, its turn as the
   * estimate has it taken off. An image without a heading of its own (a miss that is not a number)
   * says nothing here.
   *
   * @param missRadians how far its heading was from the estimate's
   * @param expectedVariance how far the estimate itself may be off the heading the camera should
   *     see, as a variance: the heading's doubt and the camera's turn's together
   * @return the variance to open its turn up by, beyond its mount's tolerance: the square of how
   *     far its headings have turned on average, or zero for no sign. After a sign the camera's
   *     misses are judged afresh
   */
  double headingMissed(int camera, double missRadians, double expectedVariance) {
    if (!ConsideredErrors.hasPlace(camera) || !Double.isFinite(missRadians)) {
      return 0;
    }
    missWeight[camera] = kept * missWeight[camera] + 1;
    missSum[camera] = kept * missSum[camera] + missRadians;
    missSquaredSum[camera] = kept * missSquaredSum[camera] + missRadians * missRadians;
    double weight = missWeight[camera];
    if (weight < FEWEST_TO_JUDGE) {
      return 0;
    }
    double average = missSum[camera] / weight;
    double scatter = Math.max(0, missSquaredSum[camera] / weight - average * average);
    double allowed = CLEAR_OF_NOISE * Math.sqrt(expectedVariance + scatter / weight);
    // Written so an average that is not a number is no sign.
    boolean aSign = Math.abs(average) > allowed;
    if (!aSign) {
      return 0;
    }
    missWeight[camera] = 0;
    missSum[camera] = 0;
    missSquaredSum[camera] = 0;
    return average * average;
  }
}
