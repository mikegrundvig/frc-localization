package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import java.util.function.UnaryOperator;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The wheels against the IMU, loop by loop: do they slide over the floor together, and was that a
 * hit? Each test drives a robot whose truth it knows (how it moves on the field, and how much
 * faster than that its wheels roll) and checks what the slip check says against that truth.
 */
class SlipCheckTest {
  private static final double LOOP = 0.02;
  private static final double REACH = 0.4;
  private static final SlipCheck.Settings SETTINGS = SlipCheck.Settings.DEFAULTS;
  private static final double TOLERANCE = SETTINGS.speedToleranceMetersPerSecond();

  /** The tread's grip, its friction of 1.2 times g: what the floor can explain. */
  private static final double GRIP = 1.2 * ImuReadings.STANDARD_GRAVITY;

  /** Loops enough to stand still for the accelerometer's bias to be learned. */
  private static final int STEADY = (int) Math.ceil(SETTINGS.steadySeconds() / LOOP) + 1;

  /**
   * A robot moving on the field, and what its wheels and IMU read of it, loop by loop. The truth is
   * its velocity on the field and its heading; its wheels can roll faster than it moves, and turn
   * it further than it turns. Its IMU sits where it's placed, and is swung around the center as the
   * robot turns.
   */
  private static final class Robot {
    final SlipCheck check;
    final Translation2d imu;
    final boolean checked;
    double heading = 0.3;
    double rate;
    double fieldVx;
    double fieldVy;

    /** How much faster than the robot the wheels roll, in the field's frame. */
    double slideX;

    double slideY;

    /** What the accelerometer reads, turned this far from the robot's axes. */
    double imuTurned;

    /** And its bias along the robot's forward axis. */
    double bias;

    /** And along its left axis. */
    double biasLeft;

    /** What becomes of each loop's reading before the check gets it: nothing, unless spoiled. */
    UnaryOperator<SlipCheck.Sample> reads = UnaryOperator.identity();

    Robot() {
      this(Translation2d.ZERO, true);
    }

    Robot(Translation2d imu, boolean checked) {
      this(imu, checked, GRIP);
    }

    /** On tread said to grip this hard, in m/s². */
    Robot(Translation2d imu, boolean checked, double grip) {
      check = new SlipCheck(SETTINGS, REACH, grip);
      this.imu = imu;
      this.checked = checked;
    }

    /** The IMU's velocity on the field: the robot's, and its swing around the center. */
    Translation2d imuVelocity() {
      Translation2d swing = new Translation2d(-rate * imu.getY(), rate * imu.getX());
      return new Translation2d(fieldVx, fieldVy).plus(swing.rotateBy(new Rotation2d(heading)));
    }

    /**
     * One loop: the robot speeds up by this much on the field and turns at this rate by its end;
     * the wheels turn it further by this much.
     */
    Robot loop(double speedUpX, double speedUpY, double newRate, double wheelsTurnMore) {
      Translation2d before = imuVelocity();
      double turn = (rate + newRate) / 2 * LOOP;
      heading += turn;
      rate = newRate;
      fieldVx += speedUpX;
      fieldVy += speedUpY;
      // What the IMU felt over the loop, along the robot's axes.
      Translation2d felt =
          imuVelocity().minus(before).div(LOOP).rotateBy(new Rotation2d(-heading - imuTurned));
      Translation2d wheels =
          new Translation2d(fieldVx + slideX, fieldVy + slideY).rotateBy(new Rotation2d(-heading));
      check.update(
          reads.apply(
              new SlipCheck.Sample(
                  LOOP,
                  turn + wheelsTurnMore,
                  turn,
                  heading,
                  rate,
                  // Its height swings it no differently: the robot only turns level.
                  new Translation3d(imu.getX(), imu.getY(), 0.1),
                  checked,
                  new ChassisVelocities(wheels.getX(), wheels.getY(), rate),
                  felt.getX() + bias,
                  felt.getY() + biasLeft,
                  0)));
      return this;
    }

    Robot loops(int count, double speedUpX, double speedUpY, double rate, double wheelsTurnMore) {
      for (int i = 0; i < count; i++) {
        loop(speedUpX, speedUpY, rate, wheelsTurnMore);
      }
      return this;
    }

    /** Stands still long enough for the accelerometer's bias to be learned. */
    Robot stand() {
      return loops(STEADY, 0, 0, 0, 0);
    }

    Robot slide(double x, double y) {
      slideX = x;
      slideY = y;
      return this;
    }
  }

  @Test
  void wheelsThatAgreeWithTheImuArentSliding() {
    // Speeding up hard down the field while turning: 4.4 m/s², a turn a second.
    Robot robot = new Robot().loops(50, 4.4 * LOOP, 0, 2 * Math.PI, 0);

    assertThat(robot.check.slipMetersPerSecond()).isZero();
    assertThat(robot.check.slideMetersPerSecond()).isLessThan(1e-9);
    assertThat(robot.check.collisionShare()).isZero();
  }

  /**
   * An IMU 0.25 m from the center is swung around it as the robot turns, and feels that: the
   * wheels' velocity is compared where it sits, so a robot spinning up to a turn a second as it
   * drives is no slide.
   */
  @Test
  void anImuOffTheCenterFeelsTheTurnSwingItAndIsNoSlide() {
    Robot robot = new Robot(new Translation2d(0.2, 0.15), true);
    for (int i = 1; i <= 50; i++) {
      robot.loop(2.0 * LOOP, 0, 2 * Math.PI * Math.min(1, i / 25.0), 0);
    }

    assertThat(robot.check.slideMetersPerSecond()).isLessThan(1e-9);
    assertThat(robot.check.slipMetersPerSecond()).isZero();
  }

  /**
   * The wheels turn the robot further than the gyro does: they slid around its center. The check
   * says how fast, at the wheels' reach, less what its tolerance allows.
   */
  @Test
  void aTurnTheGyroDoesntSeeIsASlideAtTheWheels() {
    double rate = 0.1; // rad/s more than the gyro: 4 cm/s at the wheels
    Robot slow = new Robot().loops(20, 0, 0, 0, rate * LOOP);
    Robot fast = new Robot().loops(20, 0, 0, 0, 4 * rate * LOOP);

    // The tolerance, spread over the quarter second it looks back, at the wheels.
    double allowed = Math.toRadians(SETTINGS.turnToleranceDegrees()) * REACH / 0.25;
    assertThat(slow.check.turnSlipMetersPerSecond())
        .as("the slide at the wheels, less what the tolerance allows")
        .isBetween(rate * REACH - allowed, rate * REACH);
    assertThat(fast.check.turnSlipMetersPerSecond() - slow.check.turnSlipMetersPerSecond())
        .as("four times the slide, and its doubt grows with it")
        .isCloseTo(3 * rate * REACH, within(1e-9));
    assertThat(slow.check.speedSlipMetersPerSecond()).isZero();
  }

  @Test
  void aTurnWithinTheToleranceIsNoSlide() {
    double tolerance = Math.toRadians(SETTINGS.turnToleranceDegrees());
    // Wheels worn unevenly: they turn the robot's reckoning a little as it drives straight...
    Robot straight = new Robot().loops(20, 0, 0, 0, tolerance / 14);
    // ... and a little more than it turns.
    Robot turning = new Robot().loops(20, 0, 0, 5, 0.004);

    assertThat(straight.check.slipMetersPerSecond()).isZero();
    assertThat(turning.check.slipMetersPerSecond()).isZero();
  }

  /**
   * Half the wheels' acceleration takes: the robot speeds up at 2.2 m/s², its spinning wheels at
   * 4.4. They come to roll ever faster than it moves, and the check follows them.
   */
  @Test
  void wheelsSpinningInALaunchAreASlide() {
    Robot robot = new Robot();
    double[] slip = new double[3];
    for (int i = 1; i <= 30; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0);
      if (i % 10 == 0) {
        slip[i / 10 - 1] = robot.check.speedSlipMetersPerSecond();
      }
    }

    double sliding = robot.slideX; // 1.32 m/s faster than the robot moves
    assertThat(slip).as("growing as the slide does").isSorted();
    assertThat(robot.check.slideMetersPerSecond()).isBetween(0.7 * sliding, sliding);
    assertThat(robot.check.speedSlipMetersPerSecond())
        .as("past the tolerance, all of it")
        .isEqualTo(robot.check.slideMetersPerSecond());
    assertThat(robot.check.turnSlipMetersPerSecond()).isZero();
  }

  /** Once the robot catches up with its spinning wheels, the accelerometer feels that: no slide. */
  @Test
  void whenTheRobotCatchesUpTheSlideIsOver() {
    Robot robot = new Robot();
    for (int i = 1; i <= 30; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0);
    }
    assertThat(robot.check.speedSlipMetersPerSecond()).isPositive();

    // The wheels hold their speed; the robot speeds up to it, at 6 m/s², in a fifth of a second.
    for (int i = 1; i <= 11; i++) {
      double catchUp = Math.min(6 * LOOP, robot.slideX);
      robot.slide(robot.slideX - catchUp, 0).loop(catchUp, 0, 0, 0);
    }

    assertThat(robot.slideX).isCloseTo(0, within(1e-12));
    assertThat(robot.check.speedSlipMetersPerSecond()).isZero();
    assertThat(robot.check.slideMetersPerSecond())
        .as("what was let go while it slid, at the bias's rate, is under the tolerance")
        .isLessThan(TOLERANCE);
  }

  /** Pushed sideways while the wheels stand still: the IMU feels the push get going. */
  @Test
  void aPushTheWheelsDontSeeIsASlide() {
    Robot slow = new Robot().loops(5, 0, 0, 0, 0); // standing
    Robot fast = new Robot().loops(5, 0, 0, 0, 0);
    for (int i = 1; i <= 10; i++) {
      // 1.5 and 3 m/s² sideways for a fifth of a second; the wheels roll none of it.
      slow.slide(0, -1.5 * LOOP * i).loop(0, 1.5 * LOOP, 0, 0);
      fast.slide(0, -3 * LOOP * i).loop(0, 3 * LOOP, 0, 0);
    }

    // 0.3 m/s in all, within the accelerometer's tolerance; 0.6 m/s, past it.
    assertThat(slow.check.speedSlipMetersPerSecond()).isZero();
    assertThat(fast.check.slideMetersPerSecond()).isBetween(0.8 * 0.6, 0.6);
    assertThat(fast.check.speedSlipMetersPerSecond()).isPositive();
  }

  /**
   * A push that holds at a steady speed: the accelerometer felt it get going, and nothing since,
   * but the slide stays seen, let go only as fast as a bias could have added it. When the push lets
   * go, the accelerometer feels the robot slow to its wheels' pace, and the slide is over.
   */
  @Test
  void aSteadySlideStaysSeenUntilItStops() {
    Robot robot = new Robot().loops(5, 0, 0, 0, 0);
    for (int i = 1; i <= 100; i++) {
      // To 1 m/s sideways at 3 m/s², then held there: two seconds in all. The wheels roll none of
      // it.
      double speedUp = Math.min(3 * LOOP, 1 - robot.fieldVy);
      robot.slide(0, -(robot.fieldVy + speedUp)).loop(0, speedUp, 0, 0);
    }
    double held = robot.check.slideMetersPerSecond();
    double slipping = robot.check.slipMetersPerSecond();
    for (int i = 1; i <= 25; i++) {
      double slowDown = Math.min(3 * LOOP, robot.fieldVy);
      robot.slide(0, -(robot.fieldVy - slowDown)).loop(0, -slowDown, 0, 0);
    }

    // Let go at the bias's rate: 0.5 m/s of the 1 m/s in two seconds.
    assertThat(held).as("still seen, two seconds on").isGreaterThan(0.45);
    assertThat(slipping).isPositive();
    assertThat(robot.check.slipMetersPerSecond()).as("stopped").isZero();
  }

  /**
   * An accelerometer reading 0.3 m/s² too high, twice what the speed comparison lets go of:
   * standing still for half a minute, it learns its bias once the reading has held for {@link
   * SlipCheck.Settings#steadySeconds}, before the slide it builds meanwhile reaches the tolerance;
   * then a hard launch is no slide either. A bias that size is a part's, not gravity's.
   */
  @Test
  void standingStillTheAccelerometersBiasIsLearned() {
    Robot robot = new Robot();
    robot.bias = 0.3;
    double most = 0;
    for (int i = 0; i < 1500; i++) {
      robot.loop(0, 0, 0, 0);
      most = Math.max(most, robot.check.slideMetersPerSecond());
    }
    double learned = robot.check.biasMetersPerSecSq();
    robot.loops(50, 4.4 * LOOP, 0, 0, 0);

    assertThat(most).as("standing").isLessThan(TOLERANCE);
    assertThat(learned).isCloseTo(0.3, within(1e-9));
    assertThat(robot.check.leaksGravity()).isFalse();
    assertThat(robot.check.slipMetersPerSecond()).as("launching").isZero();
  }

  /**
   * A bias it never stands still to learn (2 m/s², the robot driving at a steady 1 m/s throughout)
   * adds up as a slide, but none faster than the robot could go.
   */
  @Test
  void noSlideIsFasterThanTheRobotCouldGo() {
    Robot robot = new Robot();
    robot.fieldVx = 1;
    robot.bias = 2;
    robot.loops(1500, 0, 0, 0, 0);

    assertThat(robot.check.biasMetersPerSecSq()).as("never learned").isZero();
    assertThat(robot.check.slipMetersPerSecond()).isEqualTo(SETTINGS.maxSlideMetersPerSecond());
  }

  /**
   * A robot not yet checked whose IMU's case sits tilted on its mount, its definition left level:
   * gravity leaks into the accelerometer's level axes, 1.7 m/s² forward at 10°, and a full g to the
   * left on one mounted on a vertical plate, its y axis up. Standing, the reading holds, so it's
   * the bias, however large, and the size comparison, waiting for it, adds nothing up meanwhile.
   * Driving after, with it taken off, the wheels and the accelerometer agree: no slide. And a bias
   * that large is gravity, not a part's.
   */
  @Test
  void aSteadyReadingStandingIsTheBiasWhateverItsSize() {
    for (double[] leak : new double[][] {{1.7, 0}, {0, ImuReadings.STANDARD_GRAVITY}}) {
      Robot robot = new Robot(Translation2d.ZERO, false);
      robot.bias = leak[0];
      robot.biasLeft = leak[1];
      double most = 0;
      for (int i = 0; i < 150; i++) { // three seconds standing
        robot.loop(0, 0, 0, 0);
        most = Math.max(most, robot.check.slipMetersPerSecond());
      }
      double learned = robot.check.biasMetersPerSecSq();
      boolean leaks = robot.check.leaksGravity();
      // Then a launch at 4.4 m/s², turning a turn a second, and a stop.
      for (int i = 0; i < 25; i++) {
        robot.loop(4.4 * LOOP, 0, 2 * Math.PI, 0);
        most = Math.max(most, robot.check.slipMetersPerSecond());
      }
      for (int i = 0; i < 25; i++) {
        robot.loop(-4.4 * LOOP, 0, 0, 0);
        most = Math.max(most, robot.check.slipMetersPerSecond());
      }

      assertThat(learned).as("learned").isCloseTo(Math.hypot(leak[0], leak[1]), within(1e-9));
      assertThat(leaks).as("gravity").isTrue();
      assertThat(most).as("no slide, standing or driving").isZero();
    }
  }

  /**
   * Not yet checked, the accelerometer is compared with the wheels by size only once its bias is
   * learned, and again only once it's learned anew after the comparison starts over (another IMU,
   * or this one back): before, a launch with the wheels spinning goes unseen by it.
   */
  @Test
  void anUncheckedAccelerometerIsComparedOnlyOnceItsBiasIsLearned() {
    Robot robot = new Robot(Translation2d.ZERO, false);
    for (int i = 1; i <= 30; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0);
    }
    double unlearned = robot.check.speedSlipMetersPerSecond();

    Robot learned = new Robot(Translation2d.ZERO, false).stand();
    for (int i = 1; i <= 30; i++) {
      learned.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0);
    }
    double seen = learned.check.speedSlipMetersPerSecond();
    learned.check.startOver(Double.NaN, Double.NaN);
    learned.slide(0, 0).loop(0, 0, 0, 0);
    for (int i = 1; i <= 30; i++) {
      learned.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0);
    }

    assertThat(unlearned).as("before its bias is learned").isZero();
    assertThat(seen).as("once it is").isPositive();
    assertThat(learned.check.speedSlipMetersPerSecond()).as("started over").isZero();
  }

  /**
   * A push building slowly, at 1 m/s³, the wheels and the gyro still: the accelerometer's reading
   * never holds, so none of the push is taken for its bias, and the slide is seen in full, less
   * only what the comparison lets go of.
   */
  @Test
  void aPushBuildingSlowlyIsNoBias() {
    Robot robot = new Robot().stand();
    double felt = 0;
    for (int i = 1; i <= 100; i++) { // two seconds, to 2 m/s² and 2 m/s
      felt += 1.0 * LOOP;
      robot.slide(0, -(robot.fieldVy + felt * LOOP)).loop(0, felt * LOOP, 0, 0);
    }

    double letGo = SETTINGS.speedBiasMetersPerSecSq() * 2;
    assertThat(robot.check.biasMetersPerSecSq())
        .as("learned standing, not pushed")
        .isLessThan(1e-9);
    assertThat(robot.check.slipMetersPerSecond())
        .as("the push's %.2f m/s, less what's let go", robot.fieldVy)
        .isCloseTo(robot.fieldVy - letGo, within(0.02));
  }

  /** A slide that turns and one that speeds up, at once: the root of their squares. */
  @Test
  void theTwoAddAsTheRootOfTheirSquares() {
    Robot robot = new Robot();
    for (int i = 1; i <= 20; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0.002);
    }

    assertThat(robot.check.turnSlipMetersPerSecond()).isPositive();
    assertThat(robot.check.speedSlipMetersPerSecond()).isPositive();
    assertThat(robot.check.slipMetersPerSecond())
        .isCloseTo(
            Math.hypot(
                robot.check.turnSlipMetersPerSecond(), robot.check.speedSlipMetersPerSecond()),
            within(1e-12));
  }

  /** A turn out of the window no longer counts: the doubt it added stays, and no more is added. */
  @Test
  void aTurnThatEndedLeavesTheWindow() {
    Robot robot = new Robot().loops(5, 0, 0, 0, 0.01).loops(14, 0, 0, 0, 0);

    assertThat(robot.check.slipMetersPerSecond()).isZero();
  }

  /**
   * An accelerometer turned a quarter turn from the robot's axes reads every change of speed
   * sideways. Not yet checked, it's compared by size, which a turn doesn't change: a robot speeding
   * up hard while turning isn't taken for sliding; checked, the same reading would be.
   */
  @Test
  void anUncheckedAccelerometerTurnedOnItsMountSeesNoSlideWhereThereIsNone() {
    Robot unchecked = new Robot(Translation2d.ZERO, false);
    Robot checked = new Robot();
    unchecked.imuTurned = Math.PI / 2;
    checked.imuTurned = Math.PI / 2;
    unchecked.stand();
    checked.stand();

    unchecked.loops(30, 4.4 * LOOP, 0, 2, 0);
    checked.loops(30, 4.4 * LOOP, 0, 2, 0);

    assertThat(unchecked.check.slipMetersPerSecond()).isZero();
    assertThat(checked.check.speedSlipMetersPerSecond()).as("as it would be").isPositive();
  }

  /**
   * Not yet checked, and turned a quarter turn on its mount, the accelerometer still sees wheels
   * spinning in a launch, rolling ever faster than the robot moves, as one along the robot's axes
   * does: the wheels' change of velocity outgrows what it feels. And a push the wheels don't see,
   * and the push stopping.
   */
  @Test
  void anUncheckedAccelerometerSeesASlideBySize() {
    Robot unchecked = new Robot(Translation2d.ZERO, false);
    unchecked.imuTurned = Math.PI / 2;
    Robot checked = new Robot();
    Robot pushed = new Robot(Translation2d.ZERO, false);
    pushed.imuTurned = Math.PI / 2;
    unchecked.stand();
    checked.stand();
    pushed.stand();
    for (int i = 1; i <= 30; i++) {
      unchecked.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0);
      checked.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0);
    }
    for (int i = 1; i <= 50; i++) {
      double speedUp = Math.min(3 * LOOP, 1 - pushed.fieldVy);
      pushed.slide(0, -(pushed.fieldVy + speedUp)).loop(0, speedUp, 0, 0);
    }
    double whilePushed = pushed.check.slipMetersPerSecond();
    for (int i = 1; i <= 25; i++) {
      double slowDown = Math.min(3 * LOOP, pushed.fieldVy);
      pushed.slide(0, -(pushed.fieldVy - slowDown)).loop(0, -slowDown, 0, 0);
    }

    assertThat(unchecked.check.slipMetersPerSecond())
        .as("as the one along the robot's axes sees it")
        .isPositive()
        .isCloseTo(checked.check.slipMetersPerSecond(), within(1e-9));
    assertThat(whilePushed).as("pushed, less what it let go of").isGreaterThan(0.75);
    assertThat(pushed.check.slipMetersPerSecond()).as("stopped").isZero();
  }

  /**
   * A hit: the IMU feels 18 m/s² that the wheels don't. A robot stopping as hard is felt by both.
   * And one felt at the accelerometer's range is past what it can say: a jolt, whatever the wheels
   * did.
   */
  @Test
  void aJoltTheWheelsDontExplainIsACollision() {
    Robot hit = new Robot().loops(10, 0, 0, 0, 0);
    Robot stopping = new Robot();
    stopping.fieldVx = 2;
    stopping.loops(10, 0, 0, 0, 0);
    Robot slammed = new Robot();
    slammed.fieldVx = 2;
    slammed.loops(10, 0, 0, 0, 0);

    hit.slide(-0.36, 0).loop(0.36, 0, 0, 0); // shoved to 0.36 m/s in a loop: 18 m/s²
    stopping.loop(-0.36, 0, 0, 0); // slows as fast, its wheels with it
    slammed.loop(-0.8, 0, 0, 0); // 40 m/s², past the range, its wheels with it

    assertThat(hit.check.collisionShare()).isOne();
    assertThat(stopping.check.collisionShare()).isCloseTo(0, within(1e-12));
    assertThat(slammed.check.collisionShare()).isOne();
    assertThat(hit.check.speedSlipMetersPerSecond()).as("the collision's doubt covers it").isZero();
  }

  /**
   * Hit from the side at 13.5 m/s²: past what the tread's grip could explain (11.8), short of a
   * whole collision (15), so a share of one, the same standing still and braking hard, when the IMU
   * feels 18 m/s² in all, the wheels explaining the 12 of the braking. Only a share, the speed
   * comparison goes on with it, so a slide it starts is still seen. And the robot catching up with
   * wheels that got ahead of it, at the grip, the wheels holding their speed, is no collision.
   */
  @Test
  void whatTheWheelsDontExplainPastTheGripIsAShareOfACollision() {
    Robot standing = new Robot().loops(10, 0, 0, 0, 0);
    Robot braking = new Robot();
    braking.fieldVx = 2;
    braking.loops(10, 0, 0, 0, 0);
    Robot catchingUp = new Robot();
    catchingUp.slide(0.5, 0).loops(10, 0, 0, 0, 0);

    standing.slide(0, -0.27).loop(0, 0.27, 0, 0);
    braking.slide(0, -0.27).loop(-0.24, 0.27, 0, 0);
    catchingUp.slide(0.5 - 0.23, 0).loop(0.23, 0, 0, 0); // 11.5 m/s², its wheels holding

    double share = (13.5 - GRIP) / (SETTINGS.joltMetersPerSecSq() - GRIP);
    assertThat(standing.check.collisionShare()).isCloseTo(share, within(1e-9));
    assertThat(braking.check.collisionShare()).isCloseTo(share, within(1e-9));
    assertThat(catchingUp.check.collisionShare()).isZero();
    assertThat(braking.check.slideMetersPerSecond())
        .as("the speed comparison keeps the sideways 0.27 m/s the wheels didn't see")
        .isCloseTo(0.27, within(0.01));
  }

  /**
   * Wheels spinning ahead of the robot, the floor gripping again: it drags the wheels back as it
   * pushes the robot on, the robot speeding up at 11.5 m/s² while its wheels slow at 4.4. The two
   * changes are 15.9 m/s² apart, but sliding, the wheels say nothing of the robot: what the IMU
   * felt, under the grip, is no collision.
   */
  @Test
  void whileTheWheelsSlideTheyExplainNothingOfAJolt() {
    Robot robot = new Robot();
    for (int i = 1; i <= 30; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0);
    }
    assertThat(robot.check.speedSlipMetersPerSecond()).as("sliding").isPositive();

    robot.slide(robot.slideX - 0.23 - 0.088, 0).loop(0.23, 0, 0, 0);

    assertThat(robot.check.collisionShare()).isZero();
  }

  /**
   * The same as the wheels and the robot meet: the slide over by the speed comparison, the IMU
   * still feels the robot catching up for a moment after (its filter lags a little). Just done
   * sliding, the wheels still say nothing of the robot: no collision.
   */
  @Test
  void justDoneSlidingTheWheelsStillExplainNothingOfAJolt() {
    Robot robot = new Robot();
    for (int i = 1; i <= 10; i++) {
      robot.slide(0.1 * i, 0).loop(0, 0, 0, 0); // wheels spinning up to 1 m/s, the robot still
    }
    assertThat(robot.check.speedSlipMetersPerSecond()).as("sliding").isPositive();
    for (int i = 0; i < 3; i++) {
      robot.slide(robot.slideX - 0.23 - 0.088, 0).loop(0.23, 0, 0, 0);
    }
    assertThat(robot.check.speedSlipMetersPerSecond()).as("met").isZero();

    robot.slide(robot.slideX - 0.23 - 0.088, 0).loop(0.23, 0, 0, 0);

    assertThat(robot.check.collisionShare()).isZero();
  }

  /**
   * Treads said to grip past the collision's jolt, 1.7 g against its 15 m/s²: the floor could
   * explain any jolt up to 16.7 m/s², so one the wheels don't explain is no collision up to it, and
   * a whole one past it, with no share between; by its size alone too, since driving could feel as
   * much. At the accelerometer's range it's still a whole one.
   */
  @Test
  void aGripPastTheJoltLeavesNoCollisionTheFloorCouldExplain() {
    double grip = 1.7 * ImuReadings.STANDARD_GRAVITY;
    Robot nudged = new Robot(Translation2d.ZERO, true, grip).loops(10, 0, 0, 0, 0);
    Robot hit = new Robot(Translation2d.ZERO, true, grip).loops(10, 0, 0, 0, 0);
    Robot stopping = new Robot(Translation2d.ZERO, false, grip);
    stopping.fieldVx = 2;
    stopping.loops(10, 0, 0, 0, 0);
    Robot slammed = new Robot(Translation2d.ZERO, false, grip);
    slammed.fieldVx = 2;
    slammed.loops(10, 0, 0, 0, 0);

    nudged.slide(-0.32, 0).loop(0.32, 0, 0, 0); // 16 m/s², the wheels not seeing it
    hit.slide(-0.36, 0).loop(0.36, 0, 0, 0); // 18 m/s²
    stopping.loop(-0.32, 0, 0, 0); // 16 m/s² by its size alone
    slammed.loop(-0.8, 0, 0, 0); // 40 m/s², past the range

    assertThat(nudged.check.collisionShare()).isZero();
    assertThat(hit.check.collisionShare()).isOne();
    assertThat(stopping.check.collisionShare()).isZero();
    assertThat(slammed.check.collisionShare()).isOne();
  }

  /** Without a checked accelerometer, a jolt's size alone decides. */
  @Test
  void anUncheckedAccelerometersJoltIsJudgedByItsSize() {
    Robot stopping = new Robot(Translation2d.ZERO, false);
    stopping.fieldVx = 2;
    stopping.loops(10, 0, 0, 0, 0);

    stopping.loop(-0.36, 0, 0, 0);

    assertThat(stopping.check.collisionShare()).isOne();
  }

  @Test
  void aMissingReadingStartsTheComparisonOver() {
    Robot robot = new Robot().loops(20, 0, 0, 0, 0.004);
    for (int i = 1; i <= 20; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0.004);
    }
    assertThat(robot.check.turnSlipMetersPerSecond()).isPositive();
    assertThat(robot.check.speedSlipMetersPerSecond()).isPositive();

    robot.check.startOver(Double.NaN, Double.NaN);
    assertThat(robot.check.slipMetersPerSecond()).isZero();
    assertThat(robot.check.slideMetersPerSecond()).isZero();
    assertThat(robot.check.collisionShare()).isZero();

    // Starting over, one loop is only the comparison's start.
    robot.loop(2.2 * LOOP, 0, 0, 0.004);
    assertThat(robot.check.speedSlipMetersPerSecond()).isZero();
    robot.check.startOver(40, 0);
    assertThat(robot.check.collisionShare()).as("a jolt, by its size").isOne();
  }

  /**
   * A wheel speed or a yaw that isn't a number (a module's velocity lost, an IMU's bad frame) is no
   * slide and no collision: the comparison starts over, and the loops after it are judged as a
   * robot's that never read it, a launch after it seen as on that robot.
   */
  @Test
  void aWheelSpeedOrYawThatIsntANumberIsNoSlideAndTheCheckCarriesOn() {
    List<UnaryOperator<SlipCheck.Sample>> spoiled =
        List.of(
            // One module's velocity not a number: the kinematics' every velocity isn't.
            read ->
                new SlipCheck.Sample(
                    read.seconds(),
                    read.wheelTurnRadians(),
                    read.imuTurnRadians(),
                    read.yawRadians(),
                    read.yawRateRadiansPerSecond(),
                    read.imuPosition(),
                    read.accelerometerChecked(),
                    new ChassisVelocities(Double.NaN, Double.NaN, Double.NaN),
                    read.forwardAccelerationMetersPerSecSq(),
                    read.leftAccelerationMetersPerSecSq(),
                    read.tiltRadians()),
            // The IMU's yaw not a number, and so its turn over the loop.
            read ->
                new SlipCheck.Sample(
                    read.seconds(),
                    read.wheelTurnRadians(),
                    Double.NaN,
                    Double.NaN,
                    read.yawRateRadiansPerSecond(),
                    read.imuPosition(),
                    read.accelerometerChecked(),
                    read.wheels(),
                    read.forwardAccelerationMetersPerSecSq(),
                    read.leftAccelerationMetersPerSecSq(),
                    read.tiltRadians()));
    for (UnaryOperator<SlipCheck.Sample> spoil : spoiled) {
      // Speeding up at 2 m/s², turning a radian a second.
      Robot robot = new Robot().loops(25, 2 * LOOP, 0, 1, 0);
      Robot never = new Robot().loops(25, 2 * LOOP, 0, 1, 0);

      robot.reads = spoil;
      robot.loop(2 * LOOP, 0, 1, 0);
      robot.reads = UnaryOperator.identity();
      never.loop(2 * LOOP, 0, 1, 0);

      assertThat(robot.check.slipMetersPerSecond()).isZero();
      assertThat(robot.check.slideMetersPerSecond()).isZero();
      assertThat(robot.check.collisionShare()).isZero();
      robot.loops(25, 2 * LOOP, 0, 1, 0);
      never.loops(25, 2 * LOOP, 0, 1, 0);
      assertThat(robot.check.slipMetersPerSecond()).isZero();
      for (int i = 1; i <= 30; i++) {
        robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0.004);
        never.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0.004);
      }
      assertThat(robot.check.speedSlipMetersPerSecond()).isPositive();
      assertThat(robot.check.speedSlipMetersPerSecond())
          .isCloseTo(never.check.speedSlipMetersPerSecond(), within(1e-9));
      assertThat(robot.check.turnSlipMetersPerSecond())
          .isPositive()
          .isCloseTo(never.check.turnSlipMetersPerSecond(), within(1e-9));
    }
  }

  @Test
  void settingsMustMakeSense() {
    assertThatThrownBy(() -> SlipCheckSettingsBuilder.builder(SETTINGS).windowSeconds(0).build())
        .hasMessage("Settings.windowSeconds: 0 must be positive");
    assertThatThrownBy(
            () ->
                SlipCheckSettingsBuilder.builder(SETTINGS)
                    .speedToleranceMetersPerSecond(-1)
                    .build())
        .hasMessage("Settings.speedToleranceMetersPerSecond: -1 must be zero or more");
    assertThatThrownBy(() -> new SlipCheck(SETTINGS, 0, GRIP)).hasMessageContaining("reach");
    assertThatThrownBy(() -> new SlipCheck(SETTINGS, REACH, 0)).hasMessageContaining("grip");
    assertThatThrownBy(
            () ->
                new SlipCheck.Sample(
                    0, 0, 0, 0, 0, Translation3d.ZERO, true, new ChassisVelocities(), 0, 0, 0))
        .hasMessageContaining("takes time");
  }
}
