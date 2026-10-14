package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The estimator learning each camera's turn on its mount while the robot drives, in a world of
 * plane geometry ({@link MountWorld}): four cameras at the corners facing 45° out, a field ringed
 * with tags, and the robot driving loops around the middle, turning slowly as it goes.
 *
 * <p>Every scenario is held to the same promises, and some to one more of their own:
 *
 * <ul>
 *   <li>Each camera's turn, as learned, is within three of its standard deviations of its real one,
 *       both taken from the cameras' average: the cameras' turns are learned relative to one
 *       another, and what they share is carried at what their mounts allow.
 *   <li>The heading is within three of its standard deviations, so its doubt includes what is left
 *       of the cameras' turns.
 *   <li>The position is within a few centimeters.
 * </ul>
 */
class MountTurnTest {
  /** What one scenario holds the estimator to beyond the promises every scenario is held to. */
  enum Promise {
    /** Nothing more. */
    THE_USUAL,
    /** The first camera's turn is surer than its mount's tolerance: it was learned. */
    THE_FIRST_CAMERAS_TURN_IS_LEARNED,
    /** No camera's turn is much surer than its mount's tolerance: it was carried. */
    EVERY_TURN_IS_CARRIED
  }

  /**
   * One scenario.
   *
   * @param name what happens
   * @param run drives the world through it
   * @param mostOffMeters the farthest the estimate may be from the robot at the end
   */
  record Scenario(String name, Consumer<MountWorld> run, Promise promise, double mostOffMeters) {
    @Override
    public String toString() {
      return name;
    }
  }

  /** Each camera's mount as far off as a mount measured from CAD and checked may be. */
  private static final CameraConfig.MountTolerance MEASURED = CameraConfig.MountTolerance.MEASURED;

  private static final double PRIOR_DEGREES = Math.toDegrees(MEASURED.turnStdDevRadians());

  static Stream<Scenario> scenarios() {
    return Stream.of(
        new Scenario(
            "every camera where its configuration says: the turns stay near none",
            world -> world.driveLoops(3),
            Promise.THE_USUAL,
            0.02),
        new Scenario(
            "the first camera turned 1°, learned while driving",
            world -> {
              world.turn[0] = Math.toRadians(1);
              world.driveLoops(3);
            },
            Promise.THE_FIRST_CAMERAS_TURN_IS_LEARNED,
            0.02),
        new Scenario(
            "the first camera turned 6°, far past its tolerance, learned while driving",
            world -> {
              world.turn[0] = Math.toRadians(6);
              world.driveLoops(3);
            },
            Promise.THE_FIRST_CAMERAS_TURN_IS_LEARNED,
            0.03),
        new Scenario(
            "the first camera knocked 3° after a loop, learned anew",
            world -> {
              world.driveLoops(1);
              world.turn[0] = Math.toRadians(3);
              world.driveLoops(3);
            },
            Promise.THE_FIRST_CAMERAS_TURN_IS_LEARNED,
            0.03),
        new Scenario(
            "standing still: nothing tells a turn from the heading, so every turn is carried",
            world -> world.drive(20, 0, 0, 0),
            Promise.EVERY_TURN_IS_CARRIED,
            0.02),
        new Scenario(
            "shoved 20 cm forward and 10 cm left mid-loop: not taken for a turn",
            world -> {
              world.driveLoops(1);
              world.shove(0.2, 0.1);
              world.driveLoops(1);
            },
            Promise.THE_USUAL,
            0.02),
        new Scenario(
            "the wheels miss 30% of the travel for 2 s: not taken for a turn",
            world -> {
              world.driveLoops(1);
              world.wheelsMiss = 0.3;
              world.drive(2, 1.5, 0, 0.3);
              world.wheelsMiss = 0;
              world.driveLoops(1);
            },
            Promise.THE_USUAL,
            0.02),
        new Scenario(
            "the first camera reads one tag as another for a loop: not taken for a turn",
            world -> {
              world.driveLoops(1);
              world.misreadCamera = 0;
              world.misreadTag = 3;
              world.misreadAs = 5;
              world.driveLoops(1);
            },
            Promise.THE_USUAL,
            0.02),
        new Scenario(
            "a tag surveyed 5 cm off the layout: not taken for a turn",
            world -> {
              world.realTags.set(4, world.realTags.get(4).plus(new Translation2d(0.05, 0)));
              world.driveLoops(2);
            },
            Promise.THE_USUAL,
            0.02));
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("scenarios")
  void eachCamerasTurnIsLearnedOrCarriedAndNothingElseIsTakenForOne(Scenario scenario) {
    MountWorld world =
        new MountWorld(
            EstimatorConfig.DEFAULTS, MEASURED, new Pose2d(6, 3, new Rotation2d(0.2)), 0.1, 7);
    scenario.run().accept(world);

    PoseEstimator estimator = world.estimator;
    int cameras = MountWorld.CAMERAS.size();
    double realAverage = 0;
    double learnedAverage = 0;
    for (int camera = 0; camera < cameras; camera++) {
      realAverage += Math.toDegrees(world.turn[camera]) / cameras;
      learnedAverage += estimator.cameraTurnDegrees(MountWorld.CAMERAS.get(camera)) / cameras;
    }
    for (int camera = 0; camera < cameras; camera++) {
      String name = MountWorld.CAMERAS.get(camera);
      double real = Math.toDegrees(world.turn[camera]) - realAverage;
      double learned = estimator.cameraTurnDegrees(name) - learnedAverage;
      double stdDev = estimator.cameraTurnStdDevDegrees(name);
      assertThat(learned)
          .as("%s's turn from the cameras' average, as learned (±%.2f°)", name, stdDev)
          .isCloseTo(real, org.assertj.core.api.Assertions.within(3 * stdDev));
      switch (scenario.promise()) {
        case THE_FIRST_CAMERAS_TURN_IS_LEARNED -> {
          if (camera == 0) {
            assertThat(stdDev).as("%s's turn, learned", name).isLessThan(0.75 * PRIOR_DEGREES);
          }
        }
        case EVERY_TURN_IS_CARRIED ->
            assertThat(stdDev).as("%s's turn, carried", name).isGreaterThan(0.6 * PRIOR_DEGREES);
        case THE_USUAL -> {}
      }
    }
    double headingStdDev = Math.toDegrees(estimator.estimate().covariance().headingStdDev());
    assertThat(Math.toDegrees(world.headingError()))
        .as("the heading's error, within three of its standard deviations (%.2f°)", headingStdDev)
        .isBetween(-3 * headingStdDev, 3 * headingStdDev);
    assertThat(world.positionError())
        .as("the position's error")
        .isLessThan(scenario.mostOffMeters());
  }
}
