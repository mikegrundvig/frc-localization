package com.michaelgrundvig.frc.localization;

import java.util.ArrayDeque;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The accelerometer's bias, as the {@link SlipCheck} learns it while the robot stands still.
 *
 * <p>The robot is standing still when its wheels and its gyro both say it is not moving or turning.
 * Standing still, the accelerometer feels either its bias or a push the wheels do not see. The two
 * are told apart by whether the reading holds steady, not by its size, because a bias can be large
 * (gravity's share, from a case tilted on its mount, is meters per second squared) and a push
 * cannot keep the robot speeding up steadily for seconds: it sets the robot going and then holds it
 * at a speed, which feels like standing, or it builds, or comes and goes.
 *
 * <p>So the bias is learned from a reading that holds steady over the last {@link
 * SlipCheck.Settings#steadySeconds} of standing: the speed the readings add up to strays no further
 * than {@link SlipCheck.Settings#steadyMetersPerSecond} from the straight line their average would
 * draw. The bias is then the average over the older half of that stretch, which a push that has
 * only just begun, too small yet to tell, has not reached.
 */
final class AccelerometerBias {
  /** One loop's accelerometer reading while standing still, along the robot's axes, in m/s². */
  private record Reading(double seconds, double forward, double left) {}

  private final double stillMetersPerSecond;
  private final double steadySeconds;
  private final double steadyMetersPerSecond;
  private final double wheelReachMeters;

  /** The readings since the robot came to stand still, the last few seconds of them. */
  private final ArrayDeque<Reading> standing = new ArrayDeque<>();

  private double standingSeconds;

  private double forwardMetersPerSecSq;
  private double leftMetersPerSecSq;
  private boolean learned;

  AccelerometerBias(SlipCheck.Settings settings, double wheelReachMeters) {
    stillMetersPerSecond = settings.stillMetersPerSecond();
    steadySeconds = settings.steadySeconds();
    steadyMetersPerSecond = settings.steadyMetersPerSecond();
    this.wheelReachMeters = wheelReachMeters;
  }

  /** Forgets the bias and the standing readings: another IMU may read next, with its own bias. */
  void forget() {
    forwardMetersPerSecSq = 0;
    leftMetersPerSecSq = 0;
    learned = false;
    standing.clear();
    standingSeconds = 0;
  }

  /** The bias along the robot's forward axis: zero until it has been learned. */
  double forward() {
    return forwardMetersPerSecSq;
  }

  double left() {
    return leftMetersPerSecSq;
  }

  /** Whether the bias has been learned since it was last forgotten. */
  boolean learned() {
    return learned;
  }

  /** Takes one loop's reading, and learns the bias from it if the reading has held steady. */
  void learnFrom(SlipCheck.Sample sample) {
    double forward = sample.forwardAccelerationMetersPerSecSq();
    double left = sample.leftAccelerationMetersPerSecSq();
    if (!standingStill(sample) || !Double.isFinite(forward) || !Double.isFinite(left)) {
      standing.clear();
      standingSeconds = 0;
      return;
    }
    standing.addLast(new Reading(sample.seconds(), forward, left));
    standingSeconds += sample.seconds();
    // Keep the fewest loops that cover the stretch: drop the oldest while the rest still do.
    while (standing.size() > 1
        && standingSeconds - standing.getFirst().seconds() >= steadySeconds) {
      standingSeconds -= standing.removeFirst().seconds();
    }
    boolean notStoodLongEnough = standingSeconds < steadySeconds;
    if (notStoodLongEnough) {
      return;
    }

    // A reading times its loop's length is the speed it adds. Add those up over the whole stretch
    // and over its older half.
    double halfTheStretch = standingSeconds / 2;
    double seconds = 0;
    double olderSeconds = 0;
    double olderSpeedForward = 0;
    double olderSpeedLeft = 0;
    double speedForward = 0;
    double speedLeft = 0;
    for (Reading reading : standing) {
      boolean inOlderHalf = seconds < halfTheStretch;
      if (inOlderHalf) {
        olderSeconds += reading.seconds();
        olderSpeedForward += reading.forward() * reading.seconds();
        olderSpeedLeft += reading.left() * reading.seconds();
      }
      speedForward += reading.forward() * reading.seconds();
      speedLeft += reading.left() * reading.seconds();
      seconds += reading.seconds();
    }
    double averageForward = speedForward / seconds;
    double averageLeft = speedLeft / seconds;
    if (!heldSteady(averageForward, averageLeft)) {
      return;
    }
    forwardMetersPerSecSq = olderSpeedForward / olderSeconds;
    leftMetersPerSecSq = olderSpeedLeft / olderSeconds;
    learned = true;
  }

  /**
   * Whether the wheels and the gyro say the robot is neither moving nor turning: its wheels roll
   * slower than the still speed, and both the wheels and the gyro turn it slower than that at the
   * wheels' reach. A gyro that cannot say how fast it turns is taken as not turning.
   */
  private boolean standingStill(SlipCheck.Sample sample) {
    ChassisVelocities wheels = sample.wheels();
    double yawRate =
        Double.isFinite(sample.yawRateRadiansPerSecond()) ? sample.yawRateRadiansPerSecond() : 0;
    return Math.hypot(wheels.vx, wheels.vy) < stillMetersPerSecond
        && Math.abs(wheels.omega) * wheelReachMeters < stillMetersPerSecond
        && Math.abs(yawRate) * wheelReachMeters < stillMetersPerSecond;
  }

  /**
   * Whether the standing readings held steady. If they did, the speed they add up to, loop by loop,
   * keeps to the straight line their average draws. Where they rose or fell, it strays from it.
   */
  private boolean heldSteady(double averageForward, double averageLeft) {
    double speedForward = 0;
    double speedLeft = 0;
    double seconds = 0;
    for (Reading reading : standing) {
      speedForward += reading.forward() * reading.seconds();
      speedLeft += reading.left() * reading.seconds();
      seconds += reading.seconds();
      double strayed =
          Math.hypot(speedForward - averageForward * seconds, speedLeft - averageLeft * seconds);
      if (strayed >= steadyMetersPerSecond) {
        return false;
      }
    }
    return true;
  }
}
