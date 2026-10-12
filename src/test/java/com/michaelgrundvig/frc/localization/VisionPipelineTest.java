package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The vision pipeline, on 2026's field: images made exactly from a known robot pose, so every solve
 * can be checked against the truth, and then spoiled the ways real ones are.
 */
class VisionPipelineTest {
  private static final FieldModel FIELD =
      new FieldModel(
          Fields.FRC_2026_REBUILT_WELDED.loadField(),
          Symmetry.ROTATED,
          List.of(),
          List.of(),
          List.of());

  /** In front of the blue hub's near face (tags 25 and 26), facing it. */
  private static final Pose2d ROBOT = new Pose2d(2.5, 4.1, Rotation2d.fromDegrees(10));

  private final VisionPipeline pipeline = new VisionPipeline(FIELD);
  private final CameraConfig camera = camera(0.3, 1.0);

  /** A forward camera at the front of the robot, tilted up 15 degrees. */
  private static CameraConfig camera(double up, double noiseScale) {
    return CameraConfig.builder()
        .name("front")
        .forward(Inches.of(12))
        .left(Inches.of(0))
        .up(Inches.of(up / 0.0254))
        .roll(Degrees.of(0))
        .tiltUp(Degrees.of(15))
        .yaw(Degrees.of(0))
        .sensor(
            CameraConfig.Sensor.builder()
                .widthPixels(1280)
                .heightPixels(800)
                .fieldOfView(Degrees.of(80))
                .fieldOfViewAcross(CameraConfig.Across.DIAGONAL)
                .frameRate(Hertz.of(30))
                .latency(Milliseconds.of(30))
                .sightRange(Meters.of(7))
                .build())
        .noiseScale(noiseScale)
        .build();
  }

  /** The camera's exact view of some tags, with the robot where it is. */
  private static TagDetections seeing(
      Pose2d robot, CameraConfig mount, boolean multiTag, int... ids) {
    Pose3d cameraPose = new Pose3d(robot).transformBy(mount.robotToCamera());
    List<TagDetections.Tag> tags = new ArrayList<>();
    for (int id : ids) {
      Transform3d view = new Transform3d(cameraPose, FIELD.tag(id).orElseThrow());
      tags.add(new TagDetections.Tag(id, corners(), view, flipped(view), 0.05));
    }
    TagDetections.MultiTag solved =
        multiTag
            ? new TagDetections.MultiTag(
                new Transform3d(new Pose3d(), cameraPose),
                0.3,
                tags.stream().map(TagDetections.Tag::id).toList())
            : null;
    return new TagDetections(1_000_000_000L, mount.name(), tags, solved);
  }

  /**
   * The other pose a single tag allows: turned to face another way, and a little off to the side,
   * so choosing it would put the robot in the wrong place.
   */
  private static Transform3d flipped(Transform3d view) {
    Rotation3d aside = new Rotation3d(0, 0, Math.toRadians(8));
    return new Transform3d(
        view.getTranslation().rotateBy(aside),
        view.getRotation().rotateBy(new Rotation3d(0, 0, Math.toRadians(35))));
  }

  private static List<Translation2d> corners() {
    return List.of(
        new Translation2d(600, 380),
        new Translation2d(680, 380),
        new Translation2d(680, 420),
        new Translation2d(600, 420));
  }

  private static CaptureContext steady(@Nullable Rotation2d heading) {
    return new CaptureContext(heading, Math.toRadians(1), 0, 0, 0, 0);
  }

  /** The result's measurement, which the test expects there to be. */
  private static VisionMeasurement measured(VisionResult result) {
    return Objects.requireNonNull(result.measurement(), result.outcome());
  }

  private VisionResult process(TagDetections seen, CaptureContext robot) {
    return pipeline.process(seen, camera, robot, VisionConfig.DEFAULTS, VisionPipeline.Trust.FULL);
  }

  @Test
  void withTheGyrosHeadingEachTagFixesThePosition() {
    VisionResult result =
        process(seeing(ROBOT, camera, false, 25, 26), steady(ROBOT.getRotation()));

    VisionMeasurement measured = measured(result);
    assertThat(measured.solver()).isEqualTo(VisionMeasurement.Solver.HEADING_CONSTRAINED);
    assertThat(measured.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-9);
    assertThat(measured.covariance().hasHeading()).as("the gyro keeps heading").isFalse();
    assertThat(result.heightErrorMeters()).isCloseTo(0.0, within(1e-9));
    assertThat(result.outcome()).startsWith("heading-constrained: 2 tags at ");
  }

  /**
   * A position solved with the gyro's heading leans on it: solved with the heading 1° off, it lands
   * that angle times the distance to the tags off, square to them. The measurement says how far
   * away its tags were, so the estimator can carry that.
   */
  @Test
  void aPositionSolvedWithTheGyrosHeadingLeansOnIt() {
    TagDetections seen = seeing(ROBOT, camera, false, 25, 26);
    double angle = Math.toRadians(1);

    VisionMeasurement right = measured(process(seen, steady(ROBOT.getRotation())));
    VisionMeasurement turned =
        measured(process(seen, steady(ROBOT.getRotation().plus(new Rotation2d(angle)))));

    Translation2d toTags = right.lever().robotToTags();
    assertThat(right.lever().heading()).as("solved with the gyro's").isEqualTo(ROBOT.getRotation());
    Translation2d tags =
        FIELD
            .tag(25)
            .orElseThrow()
            .toPose2d()
            .getTranslation()
            .plus(FIELD.tag(26).orElseThrow().toPose2d().getTranslation())
            .div(2);
    assertThat(toTags.getDistance(tags.minus(ROBOT.getTranslation())))
        .as("toward the tags, as far as they are")
        .isLessThan(0.2);
    // Turning the view by the angle moves the position by the angle times the lever, square to it.
    Translation2d moved = turned.pose().getTranslation().minus(right.pose().getTranslation());
    Translation2d predicted = new Translation2d(-toTags.getY(), toTags.getX()).times(-angle);
    assertThat(moved.getDistance(predicted)).isLessThan(0.002);
    assertThat(moved.getNorm()).isGreaterThan(0.02);
  }

  /**
   * Each tag counts in the position as the solve weighs it, by the inverse square of its distance:
   * a tag off the layout puts the position off by its share, which the measurement says.
   */
  @Test
  void eachTagsShareOfThePositionIsAsTheSolveWeighsIt() {
    VisionMeasurement measured =
        measured(process(seeing(ROBOT, camera, false, 25, 26), steady(ROBOT.getRotation())));

    double[] weights = new double[2];
    for (int i = 0; i < 2; i++) {
      double distance =
          FIELD
              .tag(measured.tagIds().get(i))
              .orElseThrow()
              .getTranslation()
              .getDistance(new Pose3d(ROBOT).transformBy(camera.robotToCamera()).getTranslation());
      weights[i] = 1 / (distance * distance);
    }
    assertThat(measured.tagShares().get(0))
        .isCloseTo(weights[0] / (weights[0] + weights[1]), within(1e-6));
    assertThat(measured.tagShares().get(1))
        .isCloseTo(weights[1] / (weights[0] + weights[1]), within(1e-6));
  }

  /** A solve that finds its own heading doesn't lean on the gyro's. */
  @Test
  void aFullSolveDoesntLeanOnTheGyrosHeading() {
    CaptureContext lost = new CaptureContext(null, Math.PI, 0, 0, 0, 0);

    VisionMeasurement multi = measured(process(seeing(ROBOT, camera, true, 25, 26), lost));

    assertThat(multi.lever()).isEqualTo(VisionMeasurement.Lever.NONE);
  }

  /**
   * A single tag's two poses: the one that agrees with the gyro wins, even when it's the second.
   */
  @Test
  void theGyroSettlesWhichOfATagsTwoPosesIsRight() {
    TagDetections seen = seeing(ROBOT, camera, false, 26);
    TagDetections.Tag tag = seen.tags().get(0);
    TagDetections swapped =
        new TagDetections(
            seen.timestampNanos(),
            seen.camera(),
            List.of(
                new TagDetections.Tag(
                    26, corners(), tag.altCameraToTag(), tag.bestCameraToTag(), 0.4)),
            null);

    VisionResult result = process(swapped, steady(ROBOT.getRotation()));

    assertThat(measured(result).pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-9);
  }

  /** 2026's cameras were measured from three different origins: a wrong height shows up here. */
  @Test
  void aCameraMountedOtherThanItsConfigurationSaysFailsTheHeightCheck() {
    TagDetections seen = seeing(ROBOT, camera(0.6, 1.0), false, 25, 26); // really 30 cm higher

    VisionResult result = process(seen, steady(ROBOT.getRotation()));

    assertThat(result.measurement()).isNull();
    assertThat(result.outcome()).startsWith("a tag's height is off by");
    assertThat(result.heightErrorMeters()).isCloseTo(0.3, within(0.05));
  }

  @Test
  void headingIsMeasuredOnlyFromSeveralTagsWithTheRobotSteady() {
    Pose2d truth = ROBOT;
    Rotation2d driftedGyro = truth.getRotation().plus(Rotation2d.fromDegrees(1.5));

    VisionResult steady = process(seeing(truth, camera, true, 25, 26), steady(driftedGyro));
    VisionResult turning =
        process(
            seeing(truth, camera, true, 25, 26),
            new CaptureContext(driftedGyro, Math.toRadians(1), 0, 0, 0, 2.0));

    assertThat(measured(steady).covariance().hasHeading()).isTrue();
    assertThat(measured(steady).pose().getRotation().minus(truth.getRotation()).getDegrees())
        .as("the tags' heading, not the drifted gyro's")
        .isCloseTo(0.0, within(1e-6));
    assertThat(measured(turning).covariance().hasHeading()).as("turning: gyro only").isFalse();
  }

  /** Before the robot is placed, its heading is a guess: full solves find it instead. */
  @Test
  void withoutATrustedHeadingItSolvesTheWholePose() {
    CaptureContext lost = new CaptureContext(null, Math.PI, 0, 0, 0, 0);

    VisionResult multi = process(seeing(ROBOT, camera, true, 25, 26), lost);
    VisionResult single = process(seeing(ROBOT, camera, false, 26), lost);

    assertThat(measured(multi).solver()).isEqualTo(VisionMeasurement.Solver.MULTI_TAG);
    assertThat(measured(multi).pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
    assertThat(measured(multi).covariance().hasHeading()).isTrue();
    assertThat(measured(single).solver()).isEqualTo(VisionMeasurement.Solver.SINGLE_TAG);
  }

  @Test
  void anAmbiguousSingleTagIsThrownOutWhenNothingSettlesIt() {
    TagDetections seen = seeing(ROBOT, camera, false, 26);
    TagDetections.Tag tag = seen.tags().get(0);
    TagDetections ambiguous =
        new TagDetections(
            seen.timestampNanos(),
            seen.camera(),
            List.of(
                new TagDetections.Tag(
                    26, corners(), tag.bestCameraToTag(), tag.altCameraToTag(), 0.35)),
            null);

    VisionResult result = process(ambiguous, new CaptureContext(null, Math.PI, 0, 0, 0, 0));

    assertThat(result.measurement()).isNull();
    assertThat(result.outcome()).isEqualTo("ambiguous single tag: 0.35 > 0.20");
  }

  @Test
  void imagesTakenWhileTurningFastOrFromFarAwayOrOfOtherTagsAreThrownOut() {
    TagDetections seen = seeing(ROBOT, camera, false, 25, 26);
    Pose2d farAway = new Pose2d(0.5, 4.1, Rotation2d.ZERO); // 3.5 m from the tags

    VisionResult spinning =
        process(seen, new CaptureContext(ROBOT.getRotation(), 0.01, 0, 0, 0, 6.0));
    VisionResult tooFar =
        pipeline.process(
            seeing(farAway, camera, false, 25, 26),
            camera,
            steady(farAway.getRotation()),
            VisionConfigBuilder.builder(VisionConfig.DEFAULTS).maxDistanceMeters(3.0).build(),
            VisionPipeline.Trust.FULL);
    VisionResult unknownTag =
        process(
            new TagDetections(
                1,
                "front",
                List.of(
                    new TagDetections.Tag(99, corners(), Transform3d.ZERO, Transform3d.ZERO, 0.1)),
                null),
            steady(ROBOT.getRotation()));

    assertThat(spinning.outcome()).isEqualTo("turning too fast: 6.0 rad/s");
    assertThat(tooFar.outcome()).startsWith("too far: 3.");
    assertThat(unknownTag.outcome()).isEqualTo("no tags from this field's layout");
    assertThat(List.of(spinning, tooFar, unknownTag))
        .allSatisfy(result -> assertThat(result.measurement()).isNull());
  }

  @Test
  void aFullSolveThatPutsTheRobotOffTheFloorIsThrownOut() {
    Pose3d lifted = new Pose3d(ROBOT).transformBy(new Transform3d(0, 0, 0.4, Rotation3d.ZERO));
    Pose3d cameraPose = lifted.transformBy(camera.robotToCamera());
    TagDetections.Tag tag =
        new TagDetections.Tag(
            26,
            corners(),
            new Transform3d(cameraPose, FIELD.tag(26).orElseThrow()),
            Transform3d.ZERO,
            0.05);
    TagDetections seen = new TagDetections(1, "front", List.of(tag), null);

    VisionResult result = process(seen, new CaptureContext(null, Math.PI, 0, 0, 0, 0));

    assertThat(result.outcome()).isEqualTo("the solve puts the robot 0.40 m off the floor");
  }

  @Test
  void uncertaintyGrowsWithDistanceNoiseAndMotionAndShrinksWithTags() {
    double oneTag = spread(seeing(ROBOT, camera, false, 26), steady(ROBOT.getRotation()), camera);
    double twoTags =
        spread(seeing(ROBOT, camera, false, 25, 26), steady(ROBOT.getRotation()), camera);
    double noisyCamera =
        spread(seeing(ROBOT, camera, false, 26), steady(ROBOT.getRotation()), camera(0.3, 2.0));
    double moving =
        spread(
            seeing(ROBOT, camera, false, 26),
            new CaptureContext(ROBOT.getRotation(), 0.01, 0, 0, 4.0, 0),
            camera);
    double distance =
        seeing(ROBOT, camera, false, 26).tags().get(0).bestCameraToTag().getTranslation().getNorm();

    assertThat(oneTag).isCloseTo(0.02 * 0.5 * distance * distance, within(1e-9));
    assertThat(twoTags).isLessThan(oneTag * 0.6);
    assertThat(noisyCamera).isCloseTo(2 * oneTag, within(1e-9));
    assertThat(moving).as("4 m/s doubles it").isCloseTo(2 * oneTag, within(1e-9));
  }

  /**
   * A camera whose mount is in doubt adds the doubt to every measurement's spread, as independent
   * errors add, and a turn of the doubt over the tags' distance to its heading's.
   */
  @Test
  void aCamerasMountDoubtWidensItsMeasurements() {
    TagDetections seen = seeing(ROBOT, camera, true, 25, 26);
    CaptureContext lost = new CaptureContext(null, Math.PI, 0, 0, 0, 0);
    double doubt = 0.08;

    VisionMeasurement constrained =
        measured(
            pipeline.process(
                seen,
                camera,
                steady(ROBOT.getRotation()),
                VisionConfig.DEFAULTS,
                new VisionPipeline.Trust(doubt, null)));
    VisionMeasurement calibrated = measured(process(seen, steady(ROBOT.getRotation())));
    VisionMeasurement full =
        measured(
            pipeline.process(
                seen, camera, lost, VisionConfig.DEFAULTS, new VisionPipeline.Trust(doubt, null)));
    VisionMeasurement fullCalibrated = measured(process(seen, lost));
    double distance = process(seen, lost).distanceMeters();

    assertThat(constrained.covariance().xStdDev())
        .isCloseTo(Math.hypot(calibrated.covariance().xStdDev(), doubt), within(1e-12));
    assertThat(constrained.covariance().headingStdDev())
        .isCloseTo(
            Math.hypot(calibrated.covariance().headingStdDev(), doubt / distance), within(1e-12));
    assertThat(full.covariance().yStdDev())
        .isCloseTo(Math.hypot(fullCalibrated.covariance().yStdDev(), doubt), within(1e-12));
    assertThat(full.covariance().headingStdDev())
        .isCloseTo(
            Math.hypot(fullCalibrated.covariance().headingStdDev(), doubt / distance),
            within(1e-12));
    assertThat(constrained.pose())
        .as("where it puts the robot is the same")
        .isEqualTo(calibrated.pose());
  }

  /**
   * A task that trusts only some tags gets measurements from those alone; an image without any is
   * thrown out, saying which it wanted.
   */
  @Test
  void aTaskTrustsOnlyItsOwnTags() {
    TagDetections seen = seeing(ROBOT, camera, true, 25, 26);
    CaptureContext steady = steady(ROBOT.getRotation());

    VisionResult onlyOne =
        pipeline.process(
            seen,
            camera,
            steady,
            VisionConfig.DEFAULTS,
            new VisionPipeline.Trust(0, TagAllowlist.of("Hub", List.of(26, 27))));
    VisionResult none =
        pipeline.process(
            seen,
            camera,
            steady,
            VisionConfig.DEFAULTS,
            new VisionPipeline.Trust(0, TagAllowlist.of("Hub", List.of(10))));

    assertThat(measured(onlyOne).tagIds()).containsExactly(26);
    assertThat(measured(onlyOne).pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
    assertThat(measured(onlyOne).covariance().hasHeading())
        .as("the multi-tag solution used tag 25 too")
        .isFalse();
    assertThat(none.measurement()).isNull();
    assertThat(none.outcome()).isEqualTo("no tags this task trusts (Hub: 10)");
  }

  /**
   * A multi-tag solution from trusted tags alone still corrects heading, and solves a lost robot.
   */
  @Test
  void aMultiTagSolutionOfTrustedTagsIsKept() {
    TagDetections seen = seeing(ROBOT, camera, true, 25, 26);
    VisionPipeline.Trust hub = new VisionPipeline.Trust(0, TagAllowlist.of("Hub", List.of(25, 26)));

    VisionMeasurement steady =
        measured(
            pipeline.process(
                seen, camera, steady(ROBOT.getRotation()), VisionConfig.DEFAULTS, hub));
    VisionMeasurement lost =
        measured(
            pipeline.process(
                seen,
                camera,
                new CaptureContext(null, Math.PI, 0, 0, 0, 0),
                VisionConfig.DEFAULTS,
                hub));

    assertThat(steady.covariance().hasHeading()).isTrue();
    assertThat(lost.solver()).isEqualTo(VisionMeasurement.Solver.MULTI_TAG);
  }

  /**
   * A view or a solve that isn't a number throws the image out with its verdict, rather than hand
   * the estimator a pose it can't weigh: a tag whose place from the camera isn't a number, a
   * multi-tag solution whose place or turn isn't, and a single tag whose turn isn't.
   */
  @Test
  void aSolveThatIsntANumberIsThrownOutSayingSo() {
    TagDetections seen = seeing(ROBOT, camera, true, 25, 26);
    Transform3d solved = Objects.requireNonNull(seen.multiTag()).fieldToCamera();
    Transform3d tag = seen.tags().get(0).bestCameraToTag();
    Rotation3d noTurn = new Rotation3d(Double.NaN, 0, 0);
    CaptureContext lost = new CaptureContext(null, Math.PI, 0, 0, 0, 0);

    VisionResult tagNowhere =
        process(
            withTag(seen, new Transform3d(new Translation3d(Double.NaN, 0, 0), tag.getRotation())),
            steady(ROBOT.getRotation()));
    VisionResult solvedNowhere =
        process(
            withMultiTag(
                seen, new Transform3d(new Translation3d(Double.NaN, 0, 0), solved.getRotation())),
            lost);
    VisionResult solvedUnturned =
        process(withMultiTag(seen, new Transform3d(solved.getTranslation(), noTurn)), lost);
    VisionResult singleUnturned =
        process(
            withTag(
                seeing(ROBOT, camera, false, 26), new Transform3d(tag.getTranslation(), noTurn)),
            lost);

    assertThat(tagNowhere.refused()).isEqualTo(Verdict.TAG_NOT_A_NUMBER);
    assertThat(solvedNowhere.refused()).isEqualTo(Verdict.SOLVE_NOT_A_NUMBER);
    assertThat(solvedUnturned.refused()).isEqualTo(Verdict.SOLVE_NOT_A_NUMBER);
    assertThat(singleUnturned.refused()).isEqualTo(Verdict.SOLVE_NOT_A_NUMBER);
    assertThat(solvedUnturned.outcome()).isEqualTo("the solve isn't a number");
  }

  /**
   * With the gyro's heading, a multi-tag solution whose turn isn't a number can't measure the
   * heading: the tags still fix the position, and the gyro keeps the heading to itself.
   */
  @Test
  void aMultiTagHeadingThatIsntANumberLeavesTheHeadingToTheGyro() {
    TagDetections seen = seeing(ROBOT, camera, true, 25, 26);
    Transform3d solved = Objects.requireNonNull(seen.multiTag()).fieldToCamera();

    VisionMeasurement measured =
        measured(
            process(
                withMultiTag(
                    seen,
                    new Transform3d(solved.getTranslation(), new Rotation3d(Double.NaN, 0, 0))),
                steady(ROBOT.getRotation())));

    assertThat(measured.covariance().hasHeading()).isFalse();
    assertThat(measured.pose().getRotation()).isEqualTo(ROBOT.getRotation());
    assertThat(measured.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-9);
  }

  /** The image, its first tag seen this way instead. */
  private static TagDetections withTag(TagDetections seen, Transform3d view) {
    List<TagDetections.Tag> tags = new ArrayList<>(seen.tags());
    TagDetections.Tag first = tags.get(0);
    tags.set(0, new TagDetections.Tag(first.id(), first.corners(), view, view, first.ambiguity()));
    return new TagDetections(seen.timestampNanos(), seen.camera(), tags, seen.multiTag());
  }

  /** The image, its multi-tag solution putting the camera here instead. */
  private static TagDetections withMultiTag(TagDetections seen, Transform3d fieldToCamera) {
    TagDetections.MultiTag solved = Objects.requireNonNull(seen.multiTag());
    return new TagDetections(
        seen.timestampNanos(),
        seen.camera(),
        seen.tags(),
        new TagDetections.MultiTag(
            fieldToCamera, solved.reprojectionErrorPixels(), solved.tagIds()));
  }

  @Test
  void aMountsDoubtIsZeroOrMore() {
    assertThatThrownBy(() -> new VisionPipeline.Trust(-0.1, null))
        .hasMessage("a mount's doubt is zero or more: -0.1");
    assertThatThrownBy(() -> new VisionPipeline.Trust(Double.NaN, null))
        .isInstanceOf(IllegalArgumentException.class);
  }

  private double spread(TagDetections seen, CaptureContext robot, CameraConfig mount) {
    return measured(
            pipeline.process(seen, mount, robot, VisionConfig.DEFAULTS, VisionPipeline.Trust.FULL))
        .covariance()
        .xStdDev();
  }
}
