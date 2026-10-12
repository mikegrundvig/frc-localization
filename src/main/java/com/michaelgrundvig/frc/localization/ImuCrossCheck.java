package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayDeque;
import org.wpilib.math.util.MathUtil;

/**
 * Compares two IMUs' yaw, to catch one failing while both still report: a gyro that froze, lost its
 * mount, or saturated in a hit.
 *
 * <p>Over a sliding window of time, the two should turn the same amount. When they drift apart by
 * more than the tolerance, one of them is wrong. The tolerance grows with the turning, since two
 * gyros' scales differ a little:
 *
 * <pre>disagrees when |primary's turn - backup's turn| > tolerance + scale tolerance × |turn|</pre>
 *
 * <p>Which of the two is wrong, this can't say. It raises the question rather than silently picking
 * an answer.
 */
public final class ImuCrossCheck {
  private final long windowNanos;
  private final double toleranceRadians;
  private final double scaleTolerance;

  /** The readings within the window, oldest first. At least two are kept once there are two. */
  private final ArrayDeque<Reading> readings = new ArrayDeque<>();

  /** How far each IMU has turned since the comparison started, in radians, signed. */
  private double primaryTurned;

  private double backupTurned;

  /** Each IMU's yaw at the reading before, which the next reading's step is taken from. */
  private double lastPrimary;

  private double lastBackup;

  /**
   * One loop's reading.
   *
   * @param time when it was taken, in nanoseconds
   * @param primary how far the primary had turned by then since the comparison started, in radians
   * @param backup the same for the backup
   */
  private record Reading(long time, double primary, double backup) {}

  /**
   * How closely two IMUs must agree.
   *
   * @param windowSeconds how far back the comparison looks
   * @param toleranceDegrees how far apart the two may turn over the window, however little it
   *     turned
   * @param scaleTolerance and this share of the turning on top: two gyros whose scales differ by 1%
   *     disagree by 1% of every turn
   */
  @ConfigRecord
  public record Settings(double windowSeconds, double toleranceDegrees, double scaleTolerance) {
    /**
     * Over a second, within 3° and 5% of the turn: a gyro and an IMU whose scales differ a little.
     */
    public static final Settings DEFAULTS =
        builder().windowSeconds(1.0).toleranceDegrees(3).scaleTolerance(0.05).build();

    public Settings {
      Checks.on(Settings.class)
          .field("windowSeconds", windowSeconds, positive())
          .field("toleranceDegrees", toleranceDegrees, positive())
          .field("scaleTolerance", scaleTolerance, nonNegative())
          .orThrow();
    }

    /** Starts building the settings, one named value at a time. */
    public static ImuCrossCheckSettingsBuilder.WindowSecondsStage builder() {
      return ImuCrossCheckSettingsBuilder.builder();
    }
  }

  public ImuCrossCheck(Settings settings) {
    windowNanos = Timestamps.fromSeconds(settings.windowSeconds());
    toleranceRadians = Math.toRadians(settings.toleranceDegrees());
    scaleTolerance = settings.scaleTolerance();
  }

  /**
   * Both IMUs' yaw, once a loop while both report. Each yaw's step from the reading before is taken
   * the short way around, so a yaw that wraps past a half turn still adds up right.
   */
  public void update(long timestampNanos, double primaryYawRadians, double backupYawRadians) {
    boolean first = readings.isEmpty();
    if (!first) {
      primaryTurned += MathUtil.angleModulus(primaryYawRadians - lastPrimary);
      backupTurned += MathUtil.angleModulus(backupYawRadians - lastBackup);
    }
    lastPrimary = primaryYawRadians;
    lastBackup = backupYawRadians;
    readings.addLast(new Reading(timestampNanos, primaryTurned, backupTurned));
    forgetReadingsBefore(timestampNanos - windowNanos);
  }

  /** Lets go of the readings older than the window, always keeping two to compare. */
  private void forgetReadingsBefore(long windowStartNanos) {
    while (readings.size() > 2 && readings.getFirst().time() < windowStartNanos) {
      readings.removeFirst();
    }
  }

  /** Starts over, because one of them stopped reporting. */
  public void reset() {
    readings.clear();
    primaryTurned = 0;
    backupTurned = 0;
  }

  /**
   * How far apart the two turned over the window, in radians: the primary's turn less the backup's.
   * It is zero until there are two readings to compare.
   */
  public double disagreementRadians() {
    if (readings.size() < 2) {
      return 0;
    }
    Reading first = readings.getFirst();
    Reading last = readings.getLast();
    return (last.primary() - first.primary()) - (last.backup() - first.backup());
  }

  /** How far the primary turned over the window, in radians, either way. */
  public double turnedRadians() {
    if (readings.size() < 2) {
      return 0;
    }
    return Math.abs(readings.getLast().primary() - readings.getFirst().primary());
  }

  /** Whether the two disagree by more than the tolerance allows. */
  public boolean disagrees() {
    return Math.abs(disagreementRadians()) > toleranceRadians + scaleTolerance * turnedRadians();
  }
}
