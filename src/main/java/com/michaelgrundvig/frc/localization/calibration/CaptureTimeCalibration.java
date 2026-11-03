package com.michaelgrundvig.frc.localization.calibration;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.util.MathUtil;

/**
 * How late a camera's capture times are, from the robot spinning in place both ways where the
 * camera sees tags: the spin test.
 *
 * <p>Each tag an image sees says the robot's heading by itself: the tag's place on the field, the
 * camera's view of it, and the camera's mount. The robot's own heading at the image's stamp comes
 * from its history. If the stamp is late by {@code t}, the robot had already turned {@code w × t}
 * further by then, turning at {@code w}, so the tag's heading reads that much behind it:
 *
 * <pre>difference = tag's heading - robot's heading at the stamp = bias - w × t</pre>
 *
 * <p>The bias is whatever doesn't depend on time: the mount's error, the solve's error for that
 * view of the tag, the heading estimate's offset. It depends on where the robot points, so the
 * sightings are grouped by heading, and within each group only how the difference changes with the
 * turn rate counts. Spinning both ways over the same headings, the bias drops out, and {@code t} is
 * the least-squares slope: the sum over groups of the differences against the rates, each taken
 * from its group's mean.
 *
 * <p>A stamp off by a loop (20 ms) at 1.5 rad/s reads 1.7° off, against a single tag's heading
 * noise of a few tenths of a degree at 2 m: a few hundred sightings put {@code t} within a
 * millisecond.
 */
public final class CaptureTimeCalibration {
  /** The headings sightings are grouped by, in radians: the bias barely changes within one. */
  static final double GROUP_RADIANS = Math.toRadians(3);

  /**
   * A sighting this far from the robot's heading is a bad solve (the tag turned over, a misread),
   * never a late stamp: a stamp 100 ms late at 1.5 rad/s is 8.6° off.
   */
  static final double FARTHEST_RADIANS = Math.toRadians(15);

  /** How many robust standard deviations from the fit a sighting may be and still count. */
  static final double OUTLIER_SPREADS = 4;

  /** The fewest sightings each way that can say anything. */
  public static final int MIN_EACH_WAY = 20;

  /** Slower than this, a sighting tells little about time, and isn't counted as either way. */
  static final double SLOWEST_RADIANS_PER_SECOND = 0.2;

  /** One sighting: where the robot pointed, how far the tag's heading was off it, how it turned. */
  private record Sighting(int group, double differenceRadians, double yawRateRadiansPerSecond) {}

  private final List<Sighting> sightings = new ArrayList<>();
  private int left;
  private int right;

  /**
   * How late the stamps are, and how sure that is.
   *
   * @param lateSeconds how much later than the image was taken its stamp says (negative: earlier)
   * @param stdDevSeconds one standard deviation of that
   * @param sightings the sightings the fit counted
   */
  public record Result(double lateSeconds, double stdDevSeconds, int sightings) {}

  /**
   * One tag in one image.
   *
   * @param fieldToTag the tag's place on the field
   * @param robotToCamera the camera's mount
   * @param best the camera's view of the tag, as its solve preferred it
   * @param alternate the solve's other view (the tag turned over), taken when it fits better
   * @param robotHeading the robot's heading at the image's stamp, from its history
   * @param yawRateRadiansPerSecond how fast it was turning then, counterclockwise
   */
  public void add(
      Pose3d fieldToTag,
      Transform3d robotToCamera,
      Transform3d best,
      Transform3d alternate,
      Rotation2d robotHeading,
      double yawRateRadiansPerSecond) {
    double fromBest = difference(fieldToTag, robotToCamera, best, robotHeading);
    double fromAlternate = difference(fieldToTag, robotToCamera, alternate, robotHeading);
    double difference = Math.abs(fromAlternate) < Math.abs(fromBest) ? fromAlternate : fromBest;
    if (Math.abs(difference) > FARTHEST_RADIANS
        || Math.abs(yawRateRadiansPerSecond) < SLOWEST_RADIANS_PER_SECOND) {
      return;
    }
    int group =
        (int)
            Math.floor(
                MathUtil.inputModulus(robotHeading.getRadians(), 0, 2 * Math.PI) / GROUP_RADIANS);
    sightings.add(new Sighting(group, difference, yawRateRadiansPerSecond));
    if (yawRateRadiansPerSecond > 0) {
      left++;
    } else {
      right++;
    }
  }

  /** The robot's heading by a tag, less its heading at the stamp, the short way around. */
  private static double difference(
      Pose3d fieldToTag, Transform3d robotToCamera, Transform3d view, Rotation2d robotHeading) {
    Pose3d robot = fieldToTag.transformBy(view.inverse()).transformBy(robotToCamera.inverse());
    return MathUtil.angleModulus(robot.getRotation().getZ() - robotHeading.getRadians());
  }

  /** The sightings taken, spinning left (counterclockwise). */
  public int left() {
    return left;
  }

  /** The sightings taken, spinning right. */
  public int right() {
    return right;
  }

  /**
   * How late the stamps are. Empty with too few sightings either way, or none spinning both ways
   * over the same headings.
   */
  public Optional<Result> solve() {
    if (left < MIN_EACH_WAY || right < MIN_EACH_WAY) {
      return Optional.empty();
    }
    List<Sighting> counted = new ArrayList<>(sightings);
    Optional<Fit> fit = fit(counted);
    if (fit.isEmpty()) {
      return Optional.empty();
    }
    // Once more without the sightings far off the fit: a misread tag, a blurred image.
    double spread = fit.get().robustSpread();
    if (spread > 0) {
      Fit first = fit.get();
      counted.removeIf(s -> Math.abs(first.residual(s)) > OUTLIER_SPREADS * spread);
      fit = fit(counted);
    }
    return fit.map(f -> new Result(f.late, f.stdDev(), f.counted));
  }

  /** The fit to some sightings: the slope, and each group's means it was taken about. */
  private record Fit(
      double late,
      Map<Integer, double[]> means,
      double sumSquaredRates,
      double sumSquaredResiduals,
      int counted,
      List<Double> residuals) {
    double residual(Sighting s) {
      double[] mean = Objects.requireNonNull(means.get(s.group()));
      return (s.differenceRadians() - mean[0]) + (s.yawRateRadiansPerSecond() - mean[1]) * late;
    }

    /** The residuals' spread, from their median size, so that a few wild ones don't widen it. */
    double robustSpread() {
      List<Double> sizes = residuals.stream().map(Math::abs).sorted().toList();
      return sizes.isEmpty() ? 0 : 1.4826 * sizes.get(sizes.size() / 2);
    }

    /** One standard deviation of the slope: the residuals' spread over the rates' reach. */
    double stdDev() {
      int freedom = Math.max(1, counted - means.size() - 1);
      return Math.sqrt(sumSquaredResiduals / freedom / sumSquaredRates);
    }
  }

  private static Optional<Fit> fit(List<Sighting> sightings) {
    Map<Integer, double[]> sums = new HashMap<>();
    for (Sighting s : sightings) {
      double[] sum = sums.computeIfAbsent(s.group(), g -> new double[3]);
      sum[0] += s.differenceRadians();
      sum[1] += s.yawRateRadiansPerSecond();
      sum[2]++;
    }
    Map<Integer, double[]> means = new HashMap<>();
    sums.forEach((group, sum) -> means.put(group, new double[] {sum[0] / sum[2], sum[1] / sum[2]}));
    double crossed = 0;
    double squaredRates = 0;
    for (Sighting s : sightings) {
      double[] mean = Objects.requireNonNull(means.get(s.group()));
      double rate = s.yawRateRadiansPerSecond() - mean[1];
      crossed += (s.differenceRadians() - mean[0]) * rate;
      squaredRates += rate * rate;
    }
    // Every group spun one way only: nothing tells time from bias.
    if (squaredRates < 1e-9) {
      return Optional.empty();
    }
    double late = -crossed / squaredRates;
    List<Double> residuals = new ArrayList<>(sightings.size());
    double squaredResiduals = 0;
    for (Sighting s : sightings) {
      double[] mean = Objects.requireNonNull(means.get(s.group()));
      double residual =
          (s.differenceRadians() - mean[0]) + (s.yawRateRadiansPerSecond() - mean[1]) * late;
      residuals.add(residual);
      squaredResiduals += residual * residual;
    }
    return Optional.of(
        new Fit(late, means, squaredRates, squaredResiduals, sightings.size(), residuals));
  }
}
