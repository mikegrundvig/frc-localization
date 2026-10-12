package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.tuple;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryModuleTypeBuilder;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Consumer;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The localization service on its own: readings, images and placements in, the estimate and its
 * verdicts out. No simulator and no clock: each loop's time is the test's, and the robot stands in
 * front of the blue hub's near face (tags 25 and 26), its camera seeing it exactly.
 */
class LocalizationServiceTest {
  private static final FieldModel FIELD =
      new FieldModel(
          Fields.FRC_2026_REBUILT_WELDED.loadField(),
          Symmetry.ROTATED,
          List.of(),
          List.of(),
          List.of());

  /** In front of the blue hub's near face, facing it. */
  private static final Pose2d ROBOT = new Pose2d(2.5, 4.1, Rotation2d.fromDegrees(10));

  private static final long T0 = 1_000_000_000L;
  private static final long LOOP_NANOS = 20_000_000L;

  /** A forward camera at the front of the robot, tilted up 15 degrees. */
  private static final CameraConfig FRONT =
      CameraConfig.builder()
          .name("front")
          .forward(Inches.of(12))
          .left(Inches.of(0))
          .up(Inches.of(12))
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
          .noiseScale(1.0)
          .build();

  /** A 28 x 24 in frame, 60 kg. */
  private static final Body BODY =
      Body.builder()
          .frameLength(Inches.of(28))
          .frameWidth(Inches.of(24))
          .height(Inches.of(30))
          .bumperEnds(Inches.of(3.5))
          .bumperSides(Inches.of(3.5))
          .bumperBottom(Inches.of(2.5))
          .bumperTop(Inches.of(5.75))
          .weight(Body.Weight.weighed(Kilograms.of(60)))
          .inertia(
              Body.Turning.tensor(
                  Inertia.builder().ixx(6).iyy(6).izz(6).ixy(0).ixz(0).iyz(0).build()))
          .cg(new Translation3d(0, 0, 0.2))
          .build();

  /** A 28 x 24 in swerve robot with the front camera, its IMUs not yet measured. */
  private static final RobotDefinition DEFINITION =
      RobotDefinition.builder()
          .name("test")
          .body(BODY)
          .drivetrain(
              DrivetrainGeometry.inset(
                  DrivetrainGeometry.ModuleType.builder()
                      .driveMotor(Motor.KRAKEN_X60)
                      .steerMotor(Motor.KRAKEN_X44)
                      .wheelRadius(Inches.of(2))
                      .driveGearing(Gearing.builder().ratio(6.75).efficiency(1.0).build())
                      .steerGearing(Gearing.builder().ratio(12.8).efficiency(1.0).build())
                      .couplingRatio(0)
                      .wheelCoefficientOfFriction(1.2)
                      .driveInertia(KilogramSquareMeters.of(0.001))
                      .steerInertia(KilogramSquareMeters.of(0.004))
                      .build(),
                  BODY,
                  Inches.of(2.625)))
          .imus(Imus.UNCHECKED)
          .cameras(List.of(FRONT))
          .build();

  /** Localization with the robot placed in front of the hub, to 5 cm and half a degree. */
  private static LocalizationService placed() {
    return placed(DEFINITION);
  }

  /** The same, for another robot of the test's drivetrain. */
  private static LocalizationService placed(RobotDefinition robot) {
    return placed(new SwerveKinematicsModel(robot.moduleTranslations()), robot);
  }

  private static LocalizationService placed(
      DriveKinematicsModel kinematics, RobotDefinition robot) {
    return new LocalizationService(
        new PoseEstimator(
            EstimatorConfig.DEFAULTS,
            FIELD::contains,
            FIELD.tags().keySet(),
            new PoseReset(0, ROBOT, PoseCovariance.fromStdDevs(0.05, 0.05, 0.01), "placed")),
        kinematics,
        FIELD,
        FIELD.terrain(),
        robot,
        0.02);
  }

  /** A turret carrying a camera: it turns about the robot's up, 10 cm above its center. */
  private static final Joint.Revolute TURRET =
      Joint.Revolute.builder()
          .label("turret")
          .parent(Part.CHASSIS)
          .origin(new Transform3d(0, 0, 0.1, Rotation3d.ZERO))
          .axis(new Vector3(0, 0, 1))
          .link(Link.builder().label("turret").mass(MassProperties.NONE).build())
          .build();

  /**
   * A camera on a joint is solved from where its joint had carried it when the image was taken: a
   * turret turned 25° puts the camera elsewhere, and its image says where the robot is only through
   * that. Never told where the turret is, localization takes it to be at its reference, and solves
   * the same image as if the camera faced forward.
   */
  @Test
  void aCameraOnAJointIsSolvedFromWhereItsJointCarriedIt() {
    CameraConfig riding = CameraConfigBuilder.builder(FRONT).name("turret").joint(TURRET).build();
    RobotDefinition turreted =
        RobotDefinitionBuilder.builder(DEFINITION)
            .joints(List.of(TURRET))
            .cameras(List.of(FRONT, riding))
            .build();
    double turned = Math.toRadians(25);
    Pose3d mount = turreted.math().cameraPose(riding, Map.of(TURRET, turned));

    LocalizationService following = placed(turreted);
    long now = standStill(following, 10);
    following.joints(now - LOOP_NANOS, Map.of(TURRET, 0.0));
    following.joints(now + LOOP_NANOS, Map.of(TURRET, 2 * turned));
    TagDetections seen =
        hubFace(new Pose3d(ROBOT), new Transform3d(Pose3d.ZERO, mount), "turret", now);
    VisionResult[] result = new VisionResult[1];
    loop(following, now + LOOP_NANOS, () -> result[0] = following.camera(seen));

    LocalizationService notFollowing = placed(turreted);
    standStill(notFollowing, 10);
    VisionResult[] unfollowed = new VisionResult[1];
    loop(notFollowing, now + LOOP_NANOS, () -> unfollowed[0] = notFollowing.camera(seen));

    VisionMeasurement measured = Objects.requireNonNull(result[0].measurement());
    assertThat(measured.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .as("halfway between the readings either side of the image, the turret at 25°")
        .isLessThan(1e-6);
    VisionMeasurement wrong = unfollowed[0].measurement();
    if (wrong != null) {
      assertThat(wrong.pose().getTranslation().getDistance(ROBOT.getTranslation()))
          .isGreaterThan(0.1);
    }
  }

  /** The drivetrain standing still at a moment, its gyro tilted as given. */
  private static DrivetrainReadings still(long nanos, double pitch, double roll) {
    return DrivetrainReadings.builder()
        .wheels(List.of(new OdometrySample(nanos, new double[4], new double[4])))
        .gyro(
            ImuReadings.builder()
                .reporting(true)
                .yawRadians(0.3)
                .yawRateRadiansPerSecond(0)
                .pitchRadians(pitch)
                .rollRadians(roll)
                // Gravity, as an accelerometer tilted so feels it standing still.
                .forwardAccelerationMetersPerSecSq(-ImuReadings.STANDARD_GRAVITY * Math.sin(pitch))
                .leftAccelerationMetersPerSecSq(
                    ImuReadings.STANDARD_GRAVITY * Math.sin(roll) * Math.cos(pitch))
                .yaws(YawSamples.of(new long[] {nanos}, new double[] {0.3}))
                .build())
        .backupImu(ImuReadings.NONE)
        .velocities(new ChassisVelocities())
        .build();
  }

  /** Loops of the robot standing level, from {@link #T0}: the time of the last one. */
  private static long standStill(LocalizationService localization, int loops) {
    long now = T0;
    for (int k = 0; k < loops; k++) {
      now = T0 + k * LOOP_NANOS;
      LoopContext loop = new LoopContext(now, true);
      localization.drivetrain(still(now, 0, 0), loop);
      localization.update();
    }
    return now;
  }

  /** One loop with what the cameras send in between, as {@code during}. */
  private static LocalizationOutput loop(
      LocalizationService localization, long now, Runnable during) {
    LoopContext loop = new LoopContext(now, true);
    localization.drivetrain(still(now, 0, 0), loop);
    during.run();
    return localization.update();
  }

  /** The front camera's exact view of the hub's near face, from the robot, taken then. */
  private static TagDetections hubFace(long takenNanos) {
    Pose3d cameraPose = new Pose3d(ROBOT).transformBy(FRONT.robotToCamera());
    List<TagDetections.Tag> tags = new ArrayList<>();
    for (int id : new int[] {25, 26}) {
      Transform3d view = new Transform3d(cameraPose, FIELD.tag(id).orElseThrow());
      tags.add(
          new TagDetections.Tag(
              id,
              List.of(
                  new Translation2d(600, 380),
                  new Translation2d(680, 380),
                  new Translation2d(680, 420),
                  new Translation2d(600, 420)),
              view,
              Transform3d.ZERO,
              0.05));
    }
    return new TagDetections(takenNanos, "front", tags, null);
  }

  /** A measurement from the front camera of tags 25 and 26, put at a pose and claiming 3 cm. */
  private static VisionMeasurement measured(long takenNanos, Pose2d pose) {
    return new VisionMeasurement(
        takenNanos,
        pose,
        PoseCovariance.fromStdDevs(0.03, 0.03, Double.POSITIVE_INFINITY),
        VisionMeasurement.Lever.NONE,
        "front",
        List.of(25, 26),
        VisionMeasurement.Solver.HEADING_CONSTRAINED);
  }

  @Test
  void anImageBecomesAVerdictOnEachOfItsTags() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);

    VisionResult[] result = new VisionResult[1];
    LocalizationOutput output =
        loop(localization, now + LOOP_NANOS, () -> result[0] = localization.camera(hubFace(now)));

    assertThat(result[0].refused()).isNull();
    assertThat(result[0].outcome()).startsWith("heading-constrained: 2 tags at ");
    assertThat(output.frames()).hasSize(1);
    FrameVerdict frame = output.frames().get(0);
    assertThat(frame.camera()).isEqualTo("front");
    assertThat(frame.captureNanos()).isEqualTo(now);
    assertThat(frame.multiTag()).as("solved from each tag, not the multi-tag").isEmpty();
    FrameVerdict.Measured measured = frame.measured().orElseThrow();
    assertThat(frame.verdict()).isEqualTo(Verdict.ACCEPTED);
    assertThat(measured.solver()).isEqualTo(VisionMeasurement.Solver.HEADING_CONSTRAINED);
    assertThat(measured.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
    assertThat(frame.tags()).extracting(TagVerdict::tag).containsExactly(25, 26);
    assertThat(frame.tags())
        .allSatisfy(
            tag -> {
              assertThat(tag.verdict()).isEqualTo(Verdict.ACCEPTED);
              assertThat(tag.innovationMeters()).isLessThan(1e-6);
              assertThat(tag.chiSquared()).isLessThanOrEqualTo(tag.threshold());
              assertThat(tag.threshold())
                  .isEqualTo(EstimatorConfig.DEFAULTS.gateChiSquaredPosition());
            });
    assertThat(output.report().health().orElseThrow().accepted()).isEqualTo(1);
  }

  /**
   * A venue moves the tags images are solved against: surveyed 5 cm farther down the field, the
   * hub's face puts the robot 5 cm farther down too, from the same image.
   */
  @Test
  void aVenueMovesTheTagsImagesAreSolvedAgainst() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);
    List<FieldTag> surveyed = new ArrayList<>();
    for (FieldTag tag : FIELD.toField().getTags()) {
      Pose3d pose = tag.getPose();
      surveyed.add(
          tag.getID() == 25 || tag.getID() == 26
              ? new FieldTag(
                  tag.getID(),
                  new Pose3d(pose.getX() + 0.05, pose.getY(), pose.getZ(), pose.getRotation()))
              : tag);
    }
    localization.venue(
        new FieldModel(
            new Field("Surveyed", "", "", null, FIELD.length(), FIELD.width(), "frc", surveyed),
            Symmetry.ROTATED,
            List.of(),
            List.of(),
            List.of()));

    VisionResult[] result = new VisionResult[1];
    LocalizationOutput output =
        loop(localization, now + LOOP_NANOS, () -> result[0] = localization.camera(hubFace(now)));

    Pose2d solved = output.frames().get(0).measured().orElseThrow().pose();
    assertThat(solved.getX()).isCloseTo(ROBOT.getX() + 0.05, within(1e-6));
    assertThat(solved.getY()).isCloseTo(ROBOT.getY(), within(1e-6));
  }

  @Test
  void aVenueMustHaveTheFieldsTags() {
    LocalizationService localization = placed();
    List<FieldTag> fewer = new ArrayList<>(FIELD.toField().getTags());
    fewer.remove(0);
    FieldModel other =
        new FieldModel(
            new Field("Other", "", "", null, FIELD.length(), FIELD.width(), "frc", fewer),
            Symmetry.ROTATED,
            List.of(),
            List.of(),
            List.of());
    assertThatThrownBy(() -> localization.venue(other)).hasMessageContaining("Other has others");
  }

  /**
   * An image the robot holds back is solved and said, and never moves the estimate: the update says
   * it was held.
   */
  @Test
  void anImageSolvedButHeldBackIsNeverApplied() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);

    VisionResult[] result = new VisionResult[1];
    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () -> result[0] = localization.hold(hubFace(now), null, Verdict.HELD));

    assertThat(result[0].measurement()).isNotNull();
    assertThat(output.frames())
        .singleElement()
        .satisfies(
            held -> {
              assertThat(held.verdict()).isEqualTo(Verdict.HELD);
              assertThat(held.measured()).as("never weighed").isEmpty();
              assertThat(held.tags()).extracting(TagVerdict::verdict).containsOnly(Verdict.HELD);
            });
    assertThat(output.report().health().orElseThrow().accepted()).isZero();
    assertThatThrownBy(() -> localization.hold(hubFace(now), null, Verdict.GATED))
        .as("the estimator's verdicts are its own")
        .isInstanceOf(IllegalArgumentException.class);
  }

  /**
   * A driver's heading reset solves no image until a camera backs it, however sure it says it is,
   * here 2°: the front camera's view of the hub just after it is solved from a tag alone, its own
   * heading agreeing with the reset backs it, and the next view is solved with the heading again.
   * Until 2026-10-08 a reset as sure as the 5° a heading must be within to solve with was solved
   * with: so a reset by eye, 5°, was or wasn't on a hair of variance, and one solved with was never
   * backed.
   */
  @Test
  void aHeadingResetNoCameraHasBackedSolvesNoImage() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10) + LOOP_NANOS;
    loop(
        localization, now, () -> localization.resetHeading(ROBOT.getRotation(), Math.toRadians(2)));
    long taken = now + LOOP_NANOS;
    loop(localization, taken, () -> {});

    VisionResult[] results = new VisionResult[2];
    loop(localization, taken + LOOP_NANOS, () -> results[0] = localization.camera(hubFace(taken)));
    loop(
        localization,
        taken + 2 * LOOP_NANOS,
        () -> results[1] = localization.camera(hubFace(taken + LOOP_NANOS)));

    assertThat(Objects.requireNonNull(results[0].measurement()).solver())
        .as("solved without the reset's heading")
        .isEqualTo(VisionMeasurement.Solver.SINGLE_TAG);
    assertThat(Objects.requireNonNull(results[1].measurement()).solver())
        .as("backed, solved with it")
        .isEqualTo(VisionMeasurement.Solver.HEADING_CONSTRAINED);
  }

  /**
   * Asking how the robot stood, or solving an image held back, before the drive team places the
   * robot that loop changes nothing: the loop's images are solved as the robot stands once placed,
   * here with a heading too unsure to solve with, and the loop is recorded once.
   */
  @Test
  void askingBeforeAPlacementChangesNothing() {
    LocalizationService asked = placed();
    LocalizationService notAsked = placed();
    long now = standStill(asked, 10) + LOOP_NANOS;
    standStill(notAsked, 10);
    PoseReset roughly =
        new PoseReset(now, ROBOT, PoseCovariance.fromStdDevs(0.05, 0.05, 0.5), "facing roughly");

    VisionResult[] results = new VisionResult[2];
    loop(
        asked,
        now,
        () -> {
          for (int k = 0; k < 5; k++) {
            asked.capture(now);
            asked.hold(hubFace(now), null, Verdict.HELD);
          }
          asked.place(roughly);
          results[0] = asked.camera(hubFace(now));
        });
    loop(
        notAsked,
        now,
        () -> {
          notAsked.place(roughly);
          results[1] = notAsked.camera(hubFace(now));
        });

    assertThat(results[0]).isEqualTo(results[1]);
    assertThat(Objects.requireNonNull(results[1].measurement()).solver())
        .as("solved without the placement's unsure heading")
        .isEqualTo(VisionMeasurement.Solver.SINGLE_TAG);
    long later = now + LOOP_NANOS;
    loop(asked, later, () -> {});
    loop(notAsked, later, () -> {});
    assertThat(asked.capture(now)).isEqualTo(notAsked.capture(now));
  }

  /**
   * An image the pipeline refuses says why at once, typed, and never reaches the estimator; the
   * update says so too, among every image's verdicts, with its numbers and every tag it saw.
   */
  @Test
  void anImageThePipelineRefusesSaysWhyAtOnce() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);
    localization.vision(
        VisionConfigBuilder.builder(VisionConfig.DEFAULTS).maxDistanceMeters(0.5).build());

    VisionResult[] result = new VisionResult[1];
    LocalizationOutput output =
        loop(localization, now + LOOP_NANOS, () -> result[0] = localization.camera(hubFace(now)));

    assertThat(result[0].refused()).isEqualTo(Verdict.TOO_FAR);
    assertThat(result[0].outcome())
        .isEqualTo(Verdict.TOO_FAR.describe(result[0].distanceMeters(), 0.5))
        .startsWith("too far: ");
    FrameVerdict refused = output.frames().get(0);
    assertThat(output.frames()).hasSize(1);
    assertThat(refused.verdict()).isEqualTo(Verdict.TOO_FAR);
    assertThat(refused.measured()).as("never weighed").isEmpty();
    assertThat(refused.reason()).isEqualTo(result[0].outcome());
    assertThat(refused.captureNanos()).isEqualTo(now);
    assertThat(refused.tags())
        .extracting(TagVerdict::tag, TagVerdict::verdict, TagVerdict::reason)
        .containsExactly(
            tuple(25, Verdict.TOO_FAR, result[0].outcome()),
            tuple(26, Verdict.TOO_FAR, result[0].outcome()));
    assertThat(output.report().health().orElseThrow().rejected())
        .as("the estimator never saw it")
        .isZero();
  }

  /**
   * A measurement 1.5 m off and claiming 3 cm fails the gate: the frame and each of its tags say
   * so, with the χ² past the threshold.
   */
  @Test
  void aMeasurementFarFromTheEstimateIsGated() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);

    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () ->
                localization.camera(
                    measured(now, ROBOT.plus(new Transform2d(1.5, 0, Rotation2d.ZERO)))));

    FrameVerdict gated = output.frames().get(0);
    FrameVerdict.Measured measured = gated.measured().orElseThrow();
    assertThat(gated.verdict()).isEqualTo(Verdict.GATED);
    assertThat(measured.chiSquared()).isGreaterThan(gated.threshold()).isEqualTo(gated.value());
    assertThat(measured.innovationMeters()).isCloseTo(1.5, within(0.01));
    assertThat(gated.reason()).startsWith("gate: ").endsWith("> 13.8");
    assertThat(output.frames().get(0).tags())
        .allSatisfy(
            tag -> {
              assertThat(tag.accepted()).isFalse();
              assertThat(tag.reason()).as("each tag, as its frame").isEqualTo(gated.reason());
            });
    assertThat(output.report().pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
  }

  /** A measurement taken before the robot was placed says so, without numbers. */
  @Test
  void aMeasurementFromBeforeThePlacementIsRefused() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);

    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () -> {
              localization.place(
                  new PoseReset(
                      now + LOOP_NANOS,
                      ROBOT,
                      PoseCovariance.fromStdDevs(0.05, 0.05, 0.01),
                      "placed again"));
              localization.camera(measured(now, ROBOT));
            });

    FrameVerdict refused = output.frames().get(0);
    assertThat(refused.verdict()).isEqualTo(Verdict.BEFORE_RESET);
    assertThat(refused.measured().orElseThrow().chiSquared()).isNaN();
    assertThat(refused.reason()).isEqualTo("before the robot was last placed or recovered");
  }

  /**
   * Each camera's trust: its mount's doubt as the robot last said it, and whether the estimator
   * keeps its repeated errors apart, which it does for eight cameras and no more.
   */
  @Test
  void trustSaysHowFarEachCameraIsTrusted() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);
    localization.mountDoubt("front", 0.04, 3.0);

    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () -> {
              for (int camera = 1; camera <= 9; camera++) {
                localization.camera(
                    new VisionMeasurement(
                        now,
                        ROBOT,
                        PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY),
                        VisionMeasurement.Lever.NONE,
                        "camera " + camera,
                        List.of(26),
                        VisionMeasurement.Solver.HEADING_CONSTRAINED));
              }
            });

    Trust trust = output.trust().orElseThrow();
    assertThat(trust.cameras())
        .containsEntry(
            "front", CameraTrust.builder().mountDoubtMeters(0.04).mountDistanceMeters(3.0).build());
    assertThat(trust.cameras())
        .containsEntry("camera 8", CameraTrust.builder().build())
        .containsEntry("camera 9", CameraTrust.builder().keptApart(false).build());
    assertThat(trust.cameras().keySet())
        .as("as the estimator first heard from each, then the robot's others")
        .endsWith("camera 9", "front");
    assertThat(trust.tagsWithoutPlaces()).as("every tag on the field's layout").isEmpty();
  }

  /**
   * A measurement from a tag the field's layout doesn't have (one solved elsewhere, say): the
   * estimator can't keep its placement apart, takes each image's as new, and says so.
   */
  @Test
  void trustSaysWhichTagsAreOffTheLayout() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);
    int offTheLayout =
        1 + FIELD.tags().keySet().stream().mapToInt(Integer::intValue).max().orElse(0);

    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () ->
                localization.camera(
                    new VisionMeasurement(
                        now,
                        ROBOT,
                        PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY),
                        VisionMeasurement.Lever.NONE,
                        "front",
                        List.of(26, offTheLayout),
                        VisionMeasurement.Solver.EXTERNAL)));

    assertThat(output.frames().get(0).accepted()).isTrue();
    assertThat(output.trust().orElseThrow().tagsWithoutPlaces()).containsExactly(offTheLayout);
  }

  /** The wheels' trust: what the IMU says of them, and that a disabled robot may be carried. */
  @Test
  void trustSaysHowFarTheWheelsAreTrusted() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10) + LOOP_NANOS;
    double pitch = Math.toRadians(-12);

    LoopContext disabled = new LoopContext(now, false);
    localization.drivetrain(still(now, pitch, 0), disabled);
    WheelTrust wheels = localization.update().trust().orElseThrow().wheels();

    assertThat(wheels.conditions().disabled()).isTrue();
    assertThat(wheels.conditions().tiltRadians()).isCloseTo(Math.abs(pitch), within(1e-9));
    assertThat(wheels.conditions().groundSlopeRadians()).as("no bump explains it").isZero();
    assertThat(wheels.accelerometerLeaksGravity()).isFalse();
    assertThat(localization.localizer().wheels().orElseThrow()).isEqualTo(wheels);
  }

  /** An image is solved with the robot tilted as it was when the image was taken. */
  @Test
  void anImageIsSolvedAsTheRobotStoodWhenItWasTaken() {
    LocalizationService localization = placed();
    long tilted = standStill(localization, 10) + LOOP_NANOS;
    double pitch = Math.toRadians(-8);
    LoopContext loop = new LoopContext(tilted, true);
    localization.drivetrain(still(tilted, pitch, 0), loop);
    localization.update();
    long level = tilted + LOOP_NANOS;
    loop = new LoopContext(level, true);
    localization.drivetrain(still(level, 0, 0), loop);

    CaptureContext then = localization.capture(tilted);

    assertThat(then.pitchRadians()).isEqualTo(pitch);
    assertThat(then.heading()).isEqualTo(ROBOT.getRotation());
    assertThat(localization.capture(level).pitchRadians()).isZero();
    assertThat(localization.capture(T0 - 10 * LOOP_NANOS).heading())
        .as("before the history")
        .isNull();
  }

  @Test
  void anImageFromACameraTheRobotDoesntHaveIsAnError() {
    LocalizationService localization = placed();

    assertThatThrownBy(() -> localization.camera(new TagDetections(T0, "back", List.of(), null)))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining("back");
  }

  /** A loop happens at a time on the robot's clock. */
  @Test
  void itsRecordsRefuseWhatTheyCantBe() {
    assertThatThrownBy(() -> new LoopContext(-1, true))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(
            () ->
                new VisionResult(
                    hubFace(T0),
                    null,
                    null,
                    "neither",
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN))
        .isInstanceOf(IllegalArgumentException.class);
  }

  /**
   * The same inputs, in the same order, give the same outputs, to the last bit of every number:
   * what a replay of a logged match relies on. The session drives, turns, crosses a bump, tilts,
   * loses the gyro and then the backup IMU too, while the camera sends an image every other loop
   * and the drive team places the robot once; everything the service says along the way is held to
   * it, its outputs, each image's solve and capture, and every reading of its state.
   */
  @Test
  void theSameInputsGiveTheSameOutputs() {
    String first = session();

    assertThat(session()).isEqualTo(first);
  }

  /**
   * A camera whose mount is in doubt has its images widened by the doubt, as independent errors
   * add: what the robot says of a mount reaches every image after it.
   */
  @Test
  void aCamerasMountDoubtWidensItsImages() {
    LocalizationService doubted = placed();
    LocalizationService trusted = placed();
    long now = standStill(doubted, 10);
    standStill(trusted, 10);
    doubted.mountDoubt("front", 0.2, 3.0);

    VisionResult[] results = new VisionResult[2];
    LocalizationOutput output =
        loop(doubted, now + LOOP_NANOS, () -> results[0] = doubted.camera(hubFace(now)));
    loop(trusted, now + LOOP_NANOS, () -> results[1] = trusted.camera(hubFace(now)));

    PoseCovariance wide = Objects.requireNonNull(results[0].measurement()).covariance();
    PoseCovariance narrow = Objects.requireNonNull(results[1].measurement()).covariance();
    assertThat(wide.xx()).isCloseTo(narrow.xx() + 0.2 * 0.2, within(1e-12));
    assertThat(wide.yy()).isCloseTo(narrow.yy() + 0.2 * 0.2, within(1e-12));
    assertThat(output.trust().orElseThrow().cameras().get("front"))
        .isEqualTo(CameraTrust.builder().mountDoubtMeters(0.2).mountDistanceMeters(3.0).build());
  }

  /** The scripted robot's loops, each taken by the service: what each update said. */
  private static List<LocalizationOutput> drive(
      LocalizationService localization,
      ScriptedDrivetrain robot,
      int loops,
      ChassisVelocities velocity,
      boolean seeing) {
    List<LocalizationOutput> outputs = new ArrayList<>();
    for (int k = 0; k < loops; k++) {
      DrivetrainReadings readings = robot.loop(velocity);
      LoopContext loop = new LoopContext(robot.now(), true);
      localization.drivetrain(readings, loop);
      if (seeing) {
        localization.camera(hubFace(robot.truth3d(), robot.now()));
      }
      outputs.add(localization.update());
    }
    return outputs;
  }

  private static ScriptedDrivetrain scripted() {
    return new ScriptedDrivetrain(DEFINITION.moduleTranslations(), ROBOT, T0 - LOOP_NANOS, 0.3);
  }

  /**
   * The gyro turns odometry while it answers; lost, the backup IMU stands in, on its own zero,
   * without that taken as a turn; with both lost the wheels turn it, and the estimate says the gyro
   * is lost; back, the gyro takes over again.
   */
  @Test
  void theGyroThenTheBackupThenTheWheelsTurnOdometry() {
    LocalizationService localization = placed();
    ScriptedDrivetrain robot = scripted();
    ChassisVelocities turning = new ChassisVelocities(0.5, 0, 0.5);
    drive(localization, robot, 20, turning, false);
    assertThat(localization.localizer().yawSource().orElseThrow()).isEqualTo(YawSource.GYRO);

    robot.gyroLost = true;
    LocalizationOutput backup = drive(localization, robot, 20, turning, false).getLast();

    assertThat(localization.localizer().yawSource().orElseThrow()).isEqualTo(YawSource.BACKUP);
    assertThat(backup.report().mode().orElseThrow())
        .as("no camera has placed it, and the backup is no IMU fault")
        .isEqualTo(LocalizationEstimate.Mode.INITIALIZING);
    assertThat(headingError(localization, robot)).as("degrees").isLessThan(0.5);

    robot.backupLost = true;
    LocalizationOutput wheels = drive(localization, robot, 10, turning, false).getLast();

    assertThat(localization.localizer().yawSource().orElseThrow()).isEqualTo(YawSource.WHEELS);
    assertThat(wheels.report().mode().orElseThrow()).isEqualTo(LocalizationEstimate.Mode.IMU_FAULT);
    assertThat(localization.pitchRadians()).as("no IMU, no tilt").isZero();

    robot.gyroLost = false;
    robot.backupLost = false;
    drive(localization, robot, 5, turning, false);

    assertThat(localization.localizer().yawSource().orElseThrow()).isEqualTo(YawSource.GYRO);
    assertThat(headingError(localization, robot)).as("degrees").isLessThan(1);
  }

  private static double headingError(LocalizationService localization, ScriptedDrivetrain robot) {
    return Math.abs(
        localization.pose().getRotation().minus(robot.truth().getRotation()).getDegrees());
  }

  /**
   * Wheels that turn the robot while the gyro says it doesn't are sliding: the slip check says so,
   * and the estimate grows less sure of where the robot is than a twin's whose wheels grip.
   */
  @Test
  void wheelsSlidingWhileDrivingAreTrustedLess() {
    LocalizationService slipping = placed();
    LocalizationService gripping = placed();
    ScriptedDrivetrain sliding = scripted();
    ScriptedDrivetrain rolling = scripted();
    sliding.gyroFrozen = true;
    ChassisVelocities turning = new ChassisVelocities(1.0, 0, 2.0);

    LocalizationOutput slid = drive(slipping, sliding, 50, turning, false).getLast();
    LocalizationOutput rolled = drive(gripping, rolling, 50, turning, false).getLast();

    assertThat(slipping.localizer().wheels().orElseThrow().turnSlipMetersPerSecond()).isPositive();
    assertThat(slid.trust().orElseThrow().wheels().conditions().slipMetersPerSecond()).isPositive();
    assertThat(gripping.localizer().wheels().orElseThrow().turnSlipMetersPerSecond()).isZero();
    assertThat(slid.report().covariance().orElseThrow().largestPositionStdDev())
        .isGreaterThan(2 * rolled.report().covariance().orElseThrow().largestPositionStdDev());
  }

  /**
   * A camera that goes silent leaves the estimate on odometry: tracking while it sees, dead
   * reckoning a second after its last image, and not good enough to score from.
   */
  @Test
  void aCameraGoingSilentLeavesTheEstimateDeadReckoning() {
    LocalizationService localization = placed();
    ScriptedDrivetrain robot = scripted();

    LocalizationOutput seeing =
        drive(localization, robot, 25, new ChassisVelocities(), true).getLast();
    assertThat(seeing.report().mode().orElseThrow()).isEqualTo(LocalizationEstimate.Mode.TRACKING);
    assertThat(seeing.frames()).hasSize(1);

    List<LocalizationOutput> silent =
        drive(localization, robot, 60, new ChassisVelocities(), false);

    assertThat(silent).allSatisfy(output -> assertThat(output.frames()).isEmpty());
    LocalizationEstimate estimate = ((DefaultLocalizer) localization.localizer()).estimate();
    assertThat(estimate.mode()).isEqualTo(LocalizationEstimate.Mode.DEAD_RECKONING);
    assertThat(estimate.health().visionAgeSeconds()).isGreaterThan(1);
    assertThat(LocalizationGate.SCORING.whyNot(estimate)).contains("no camera measurement lately");
  }

  /** What the session compares by: every bit, where WPILib's geometry allows 1e-9. */
  @Test
  void aFingerprintTellsApartWhatWpilibCallsEqual() {
    Pose2d pose = new Pose2d(1, 2, Rotation2d.ZERO);
    Pose2d nearly = new Pose2d(1 + 1e-12, 2, Rotation2d.ZERO);

    assertThat(nearly).isEqualTo(pose);
    assertThat(Fingerprint.of(nearly)).isNotEqualTo(Fingerprint.of(pose));
    assertThatThrownBy(() -> Fingerprint.of(new Object()))
        .isInstanceOf(IllegalArgumentException.class);
  }

  /**
   * Treads said to grip past the jolt the slip check calls a collision, as a path planner's
   * friction often is, are a doubt about the robot, naming what it costs; localization still takes
   * it.
   */
  @Test
  void aGripPastWhatTheSlipCheckTellsIsADoubt() {
    RobotDefinition grippy =
        RobotDefinitionBuilder.builder(DEFINITION)
            .drivetrain(
                DrivetrainGeometry.inset(
                    DrivetrainGeometryModuleTypeBuilder.builder(DEFINITION.drivetrain().module(0))
                        .wheelCoefficientOfFriction(2.255)
                        .build(),
                    BODY,
                    Inches.of(2.625)))
            .build();

    assertThat(DefaultLocalizer.doubts(DEFINITION)).isEmpty();
    assertThat(DefaultLocalizer.doubts(grippy))
        .singleElement()
        .satisfies(
            doubt -> {
              assertThat(doubt.level()).isEqualTo(Finding.Level.DOUBT);
              assertThat(doubt.subject()).isEqualTo("test");
              assertThat(doubt.message())
                  .startsWith(
                      "the treads' grip, 2.26, is past 1.53, above which the slip check can't tell"
                          + " a slide from a collision");
            });
    assertThat(
            new LocalizationService(
                FIELD,
                grippy,
                new SwerveKinematicsModel(grippy.moduleTranslations()),
                EstimatorConfig.DEFAULTS,
                0.02))
        .isNotNull();
  }

  /** A bump across the robot's way, its crest a meter ahead. */
  static final Bump BUMP =
      new Bump(
          "Test bump",
          new Translation2d(ROBOT.getX() + 1.0, 6.0),
          new Translation2d(ROBOT.getX() + 1.0, 2.0),
          0.165,
          1.0);

  /** The robot placed in front of the hub, with the test bump ahead of it. */
  private static LocalizationService beforeTheBump() {
    return new LocalizationService(
        new PoseEstimator(
            EstimatorConfig.DEFAULTS,
            FIELD::contains,
            FIELD.tags().keySet(),
            new PoseReset(0, ROBOT, PoseCovariance.fromStdDevs(0.05, 0.05, 0.01), "placed")),
        new SwerveKinematicsModel(DEFINITION.moduleTranslations()),
        FIELD,
        new Terrain(List.of(BUMP)),
        DEFINITION,
        0.02);
  }

  /** The front camera's exact view of the hub's near face, from where the robot really is. */
  private static TagDetections hubFace(Pose3d robot, long takenNanos) {
    return hubFace(robot, FRONT.robotToCamera(), "front", takenNanos);
  }

  /** A camera's exact view of the hub's near face, from where it really is on the robot. */
  private static TagDetections hubFace(
      Pose3d robot, Transform3d mount, String camera, long takenNanos) {
    Pose3d cameraPose = robot.transformBy(mount);
    List<TagDetections.Tag> tags = new ArrayList<>();
    for (int id : new int[] {25, 26}) {
      Transform3d view = new Transform3d(cameraPose, FIELD.tag(id).orElseThrow());
      tags.add(
          new TagDetections.Tag(
              id,
              List.of(
                  new Translation2d(600, 380),
                  new Translation2d(680, 380),
                  new Translation2d(680, 420),
                  new Translation2d(600, 420)),
              view,
              Transform3d.ZERO,
              0.05));
    }
    return new TagDetections(takenNanos, camera, tags, null);
  }

  /** One leg of the session: how long, how fast, and what changes as it starts. */
  private record Leg(int loops, ChassisVelocities velocity, Consumer<ScriptedDrivetrain> setUp) {}

  /** The session, every number it said fingerprinted, loop by loop. */
  private static String session() {
    LocalizationService localization = beforeTheBump();
    ScriptedDrivetrain robot =
        new ScriptedDrivetrain(DEFINITION.moduleTranslations(), ROBOT, T0 - LOOP_NANOS, 0.3);
    robot.bump = BUMP;
    List<Leg> legs =
        List.of(
            new Leg(15, new ChassisVelocities(), drivetrain -> {}),
            new Leg(60, new ChassisVelocities(1.5, 0, 0), drivetrain -> {}), // over the bump
            new Leg(25, new ChassisVelocities(1.0, 0, 2.5), drivetrain -> {}), // turning
            new Leg(
                20,
                new ChassisVelocities(1.0, 0.3, 0),
                drivetrain -> drivetrain.pitch = Math.toRadians(-7)), // tilted
            new Leg(
                20,
                new ChassisVelocities(0.5, 0, 0.5),
                drivetrain -> {
                  drivetrain.pitch = 0;
                  drivetrain.gyroLost = true; // the backup stands in
                }),
            new Leg(
                15,
                new ChassisVelocities(0.5, 0, 0),
                drivetrain -> drivetrain.backupLost = true), // the wheels alone
            new Leg(
                20,
                new ChassisVelocities(),
                drivetrain -> {
                  drivetrain.gyroLost = false;
                  drivetrain.backupLost = false;
                }));
    StringBuilder said = new StringBuilder();
    Set<YawSource> sources = EnumSet.noneOf(YawSource.class);
    int loops = 0;
    for (Leg leg : legs) {
      leg.setUp().accept(robot);
      for (int k = 0; k < leg.loops(); k++, loops++) {
        DrivetrainReadings readings = robot.loop(leg.velocity());
        LoopContext loop = new LoopContext(robot.now(), loops < 160);
        localization.drivetrain(readings, loop);
        if (loops == 140) {
          localization.place(
              new PoseReset(
                  robot.now(),
                  robot.truth(),
                  PoseCovariance.fromStdDevs(0.05, 0.05, 0.02),
                  "placed again"));
        }
        if (loops % 2 == 0) {
          long taken = robot.now();
          said.append(Fingerprint.of(localization.capture(taken)));
          said.append(Fingerprint.of(localization.camera(hubFace(robot.truth3d(), taken))));
        }
        said.append(Fingerprint.of(localization.update()));
        said.append(
            Fingerprint.of(
                List.of(
                    localization.pose(),
                    localization.pose3d(localization.pose()),
                    localization.pitchRadians(),
                    localization.rollRadians(),
                    localization.localizer().yawSource().orElseThrow(),
                    localization.localizer().wheels().orElseThrow().conditions(),
                    localization.localizer().wheels().orElseThrow(),
                    localization.imuDisagreementRadians(),
                    localization.imusDisagree())));
        said.append('\n');
        sources.add(localization.localizer().yawSource().orElseThrow());
      }
    }
    assertThat(sources).as("every IMU, and none").containsExactlyInAnyOrder(YawSource.values());
    return said.toString();
  }
}
