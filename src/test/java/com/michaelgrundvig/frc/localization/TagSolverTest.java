package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The robot's heading by one tag alone: of the tag's two views, the one nearer a heading the robot
 * is known to face roughly. A tag 5 m ahead of a camera at the robot's center, facing back at it.
 */
class TagSolverTest {
  private static final Pose3d TAG = new Pose3d(5, 0, 0.5, new Rotation3d(0, 0, Math.PI));

  /** The camera's view of the tag from a robot at the origin facing this way. */
  private static Transform3d seenFacing(double degrees) {
    return new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(degrees))), TAG);
  }

  /**
   * One tag's two views, and what the robot is known to face.
   *
   * @param bestDegrees the heading the best view puts the robot at
   * @param alternate the other view; {@link Transform3d#ZERO} for none
   * @param nearDegrees the heading the robot is known to face roughly
   * @param expectedDegrees the heading the tag gives
   */
  record Row(
      String name,
      double bestDegrees,
      Transform3d alternate,
      double nearDegrees,
      double expectedDegrees) {
    @Override
    public String toString() {
      return name;
    }
  }

  static Stream<Row> rows() {
    return Stream.of(
        new Row("the best view, nearer", 2, seenFacing(-25), 0, 2),
        new Row("the other view, nearer: the tag turned over", 25, seenFacing(-1), 0, -1),
        new Row("no other view: the best, however far", 40, Transform3d.ZERO, 0, 40),
        new Row("the nearer the short way round", 178, seenFacing(150), -179, 178));
  }

  @ParameterizedTest
  @MethodSource("rows")
  void aTagGivesTheHeadingOfItsViewNearerTheOneKnown(Row row) {
    Rotation2d heading =
        TagSolver.robotHeadingByTag(
            TAG,
            seenFacing(row.bestDegrees()),
            row.alternate(),
            Transform3d.ZERO,
            Rotation2d.fromDegrees(row.nearDegrees()));

    assertThat(heading.minus(Rotation2d.fromDegrees(row.expectedDegrees())).getDegrees())
        .isCloseTo(0, within(1e-9));
  }
}
