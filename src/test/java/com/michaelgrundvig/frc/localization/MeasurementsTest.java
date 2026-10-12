package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/** The measurement types hold only what can be real, and keep their own copies. */
class MeasurementsTest {
  private static final List<Translation2d> CORNERS =
      List.of(
          new Translation2d(100, 100),
          new Translation2d(140, 100),
          new Translation2d(140, 140),
          new Translation2d(100, 140));

  @Test
  void timestampsConvertBetweenSecondsAndNanoseconds() {
    assertThat(Timestamps.fromSeconds(1.5)).isEqualTo(1_500_000_000L);
    assertThat(Timestamps.toSeconds(1_500_000_000L)).isEqualTo(1.5);
    double cameraTime = 609208.071697128; // a camera result's time, seven days into a run
    assertThat(Timestamps.toSeconds(Timestamps.fromSeconds(cameraTime)))
        .isCloseTo(cameraTime, within(1e-9));
    assertThatThrownBy(() -> Timestamps.fromSeconds(-1)).hasMessageContaining("robot's clock");
    assertThatThrownBy(() -> Timestamps.fromSeconds(Double.NaN))
        .hasMessageContaining("robot's clock");
  }

  @Test
  void bodyMotionIsNumbersWithAResidualOfZeroOrMore() {
    BodyMotion motion = new BodyMotion(0.02, -0.01, 0.003, 0.0005);

    assertThat(motion.residualMeters()).isEqualTo(0.0005);
    assertThatThrownBy(() -> new BodyMotion(Double.NaN, 0, 0, 0)).hasMessageContaining("numbers");
    assertThatThrownBy(() -> new BodyMotion(0, 0, 0, -1)).hasMessageContaining("residual");
    assertThatThrownBy(() -> new BodyMotion(0, 0, 0, Double.POSITIVE_INFINITY))
        .hasMessageContaining("residual");
  }

  @Test
  void anImuIsUsableOnlyWhileItReportsNumbers() {
    ImuReadings level =
        ImuReadings.builder()
            .reporting(true)
            .yawRadians(0.3)
            .yawRateRadiansPerSecond(0)
            .pitchRadians(0)
            .rollRadians(0)
            .forwardAccelerationMetersPerSecSq(0)
            .leftAccelerationMetersPerSecSq(0)
            .yaws(YawSamples.of(new long[] {1L, 2L}, new double[] {0.29, 0.3}))
            .build();

    assertThat(level.usable()).isTrue();
    assertThat(ImuReadings.NONE.usable()).as("not there").isFalse();
    assertThat(ImuReadingsBuilder.builder(level).pitchRadians(Double.NaN).build().usable())
        .as("an attitude that isn't a number")
        .isFalse();
    assertThat(
            ImuReadingsBuilder.builder(level)
                .yaws(
                    YawSamples.of(
                        new long[] {1L, 2L}, new double[] {0.29, Double.POSITIVE_INFINITY}))
                .build()
                .usable())
        .as("a yaw reading that isn't one")
        .isFalse();
    assertThatThrownBy(() -> YawSamples.of(new long[] {1L, 2L}, new double[] {0.3}))
        .hasMessage(
            "ImuReadings.yawsRadians: must have one reading for each time: 2 times, 1 readings");
  }

  @Test
  void tagDetectionsKeepTheirOwnListsAndCheckEachTag() {
    List<TagDetections.Tag> tags = new ArrayList<>();
    tags.add(new TagDetections.Tag(26, CORNERS, Transform3d.ZERO, Transform3d.ZERO, 0.1));
    TagDetections.MultiTag multiTag =
        new TagDetections.MultiTag(Transform3d.ZERO, 0.3, List.of(25, 26));
    TagDetections seen = new TagDetections(5, "front-left", tags, multiTag);

    tags.clear();

    assertThat(seen.tags()).hasSize(1);
    assertThat(seen.tags().get(0).corners()).hasSize(4);
    assertThat(seen.multiTag()).isNotNull();
    assertThat(new TagDetections(5, "front-left", List.of(), null).multiTag()).isNull();
    assertThatThrownBy(
            () ->
                new TagDetections.Tag(
                    26, CORNERS.subList(0, 3), Transform3d.ZERO, Transform3d.ZERO, 0.1))
        .hasMessageContaining("4 corners");
    assertThatThrownBy(
            () -> new TagDetections.Tag(26, CORNERS, Transform3d.ZERO, Transform3d.ZERO, 1.5))
        .hasMessageContaining("ambiguity");
    assertThatThrownBy(() -> new TagDetections.MultiTag(Transform3d.ZERO, -1, List.of(26)))
        .hasMessageContaining("reprojection");
    assertThatThrownBy(() -> new TagDetections.MultiTag(Transform3d.ZERO, 0.3, List.of()))
        .hasMessageContaining("at least one tag");
    assertThatThrownBy(() -> new TagDetections(5, " ", List.of(), null))
        .hasMessageContaining("camera");
    assertThatThrownBy(() -> new TagDetections(-5, "front-left", List.of(), null))
        .hasMessageContaining("robot's clock");
  }

  @Test
  void aVisionMeasurementSaysWhereItCameFrom() {
    List<Integer> tags = new ArrayList<>(List.of(25, 26));
    VisionMeasurement measurement =
        new VisionMeasurement(
            5,
            new Pose2d(3, 4, Rotation2d.ZERO),
            PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY),
            VisionMeasurement.Lever.NONE,
            "front-left",
            tags,
            VisionMeasurement.Solver.HEADING_CONSTRAINED);

    tags.clear();

    assertThat(measurement.tagIds()).containsExactly(25, 26);
    assertThat(measurement.covariance().hasHeading()).isFalse();
    assertThatThrownBy(
            () ->
                new VisionMeasurement(
                    5,
                    Pose2d.ZERO,
                    PoseCovariance.fromStdDevs(0.05, 0.05, 0.1),
                    VisionMeasurement.Lever.NONE,
                    "front-left",
                    List.of(),
                    VisionMeasurement.Solver.MULTI_TAG))
        .hasMessageContaining("at least one tag");
    assertThatThrownBy(
            () ->
                new VisionMeasurement(
                    5,
                    Pose2d.ZERO,
                    PoseCovariance.fromStdDevs(0.05, 0.05, 0.1),
                    VisionMeasurement.Lever.NONE,
                    "",
                    List.of(1),
                    VisionMeasurement.Solver.SINGLE_TAG))
        .hasMessageContaining("camera");
    assertThatThrownBy(
            () ->
                new VisionMeasurement(
                    -5,
                    Pose2d.ZERO,
                    PoseCovariance.fromStdDevs(0.05, 0.05, 0.1),
                    VisionMeasurement.Lever.NONE,
                    "front-left",
                    List.of(1),
                    VisionMeasurement.Solver.EXTERNAL))
        .hasMessageContaining("robot's clock");
    assertThatThrownBy(
            () -> new VisionMeasurement.Lever(Rotation2d.ZERO, new Translation2d(Double.NaN, 0)))
        .hasMessageContaining("offset to its tags");
  }

  /**
   * Each tag's share of a measurement's position: equal unless said, and as given otherwise, made
   * to add up to 1.
   */
  @Test
  void aVisionMeasurementsTagsShareItsPosition() {
    VisionMeasurement equal =
        new VisionMeasurement(
            5,
            Pose2d.ZERO,
            PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY),
            VisionMeasurement.Lever.NONE,
            "front-left",
            List.of(25, 26),
            VisionMeasurement.Solver.MULTI_TAG);
    VisionMeasurement weighed = share(List.of(25, 26), List.of(3.0, 1.0));

    assertThat(equal.tagShares()).containsExactly(0.5, 0.5);
    assertThat(weighed.tagShares()).containsExactly(0.75, 0.25);
    assertThatThrownBy(() -> share(List.of(25, 26), List.of(1.0)))
        .hasMessageContaining("a share for each");
    assertThatThrownBy(() -> share(List.of(25, 26), List.of(1.0, -1.0)))
        .hasMessageContaining("zero or more");
    assertThatThrownBy(() -> share(List.of(25), List.of(Double.POSITIVE_INFINITY)))
        .hasMessageContaining("zero or more");
    assertThatThrownBy(() -> share(List.of(25, 26), List.of(0.0, 0.0)))
        .hasMessageContaining("share its position");
  }

  private static VisionMeasurement share(List<Integer> tags, List<Double> shares) {
    return new VisionMeasurement(
        5,
        Pose2d.ZERO,
        PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY),
        VisionMeasurement.Lever.NONE,
        "front-left",
        tags,
        shares,
        VisionMeasurement.Solver.HEADING_CONSTRAINED);
  }

  @Test
  void aPoseResetSaysWhy() {
    PoseReset reset =
        new PoseReset(5, Pose2d.ZERO, PoseCovariance.fromStdDevs(0.1, 0.1, 0.05), "placed");

    assertThat(reset.reason()).isEqualTo("placed");
    assertThatThrownBy(
            () -> new PoseReset(5, Pose2d.ZERO, PoseCovariance.fromStdDevs(0.1, 0.1, 0.05), ""))
        .hasMessageContaining("reason");
    assertThatThrownBy(
            () -> new PoseReset(-5, Pose2d.ZERO, PoseCovariance.fromStdDevs(0.1, 0.1, 0.05), "x"))
        .hasMessageContaining("robot's clock");
  }
}
