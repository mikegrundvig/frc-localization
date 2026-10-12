package com.michaelgrundvig.frc.localization;

import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;

/**
 * What became of one camera image in an update: as a whole, by the coprocessor's multi-tag
 * solution, and by tag. Every image localization was given is one, whether the estimator weighed
 * its measurement or it was refused before it became one. Logs, the pit's page, and the trust
 * checks read these, so none of them reaches into the estimator.
 *
 * @param camera the camera's name, as in the robot's definition
 * @param captureNanos when the image was taken, on the robot's clock
 * @param verdict what became of it: accepted, or why not
 * @param threshold the limit the verdict's value was held to; NaN where the verdict has none
 * @param value the verdict's number: a χ² for the estimator's verdicts, the tags' distance, the
 *     robot's turning, a tag's ambiguity or height for the pipeline's; NaN where it has none
 * @param multiTag the verdict on the coprocessor's multi-tag solution, when the measurement was
 *     that solution; empty when it was solved another way, or not at all
 * @param tags the tags the verdict applies to, each with its own: those the measurement rested on,
 *     or every tag the image saw, if it was refused or held back before becoming one
 * @param measured the image's measurement as the estimator weighed it; empty for an image refused,
 *     or held back by the robot, before it got there
 */
@OutputRecord
public record FrameVerdict(
    String camera,
    long captureNanos,
    Verdict verdict,
    @RecordBuilder.Initializer(value = "NONE", source = Verdict.class) double threshold,
    @RecordBuilder.Initializer(value = "NONE", source = Verdict.class) double value,
    Optional<Verdict> multiTag,
    List<TagVerdict> tags,
    Optional<Measured> measured) {
  public FrameVerdict {
    Objects.requireNonNull(camera, "a frame's camera");
    Objects.requireNonNull(verdict, "a frame's verdict");
    tags = List.copyOf(tags);
  }

  /** Builds one by name; a number left out is NaN, and the tags and the measurement none. */
  public static FrameVerdictBuilder builder() {
    return FrameVerdictBuilder.builder();
  }

  /** Whether it moved the estimate. */
  public boolean accepted() {
    return verdict == Verdict.ACCEPTED;
  }

  /** Why, in words with its numbers, as the log has them: "gate: 45.2 > 13.8", "too far: 6.3 m". */
  public String reason() {
    return verdict.describe(value, threshold);
  }

  /**
   * An image's measurement, as the estimator weighed it.
   *
   * @param pose the robot's pose, as the image measured it
   * @param solver how the image was solved
   * @param innovationMeters how far it was from the estimate at its capture time; NaN if it was
   *     refused before being compared
   * @param chiSquared its squared Mahalanobis distance from the estimate; NaN if not compared
   */
  @OutputRecord
  public record Measured(
      Pose2d pose,
      VisionMeasurement.Solver solver,
      @RecordBuilder.Initializer(value = "NONE", source = Verdict.class) double innovationMeters,
      @RecordBuilder.Initializer(value = "NONE", source = Verdict.class) double chiSquared) {
    public Measured {
      Objects.requireNonNull(pose, "a measurement's pose");
      Objects.requireNonNull(solver, "how a measurement was solved");
    }

    /** Builds one by name; a number left out is NaN. */
    public static FrameVerdictMeasuredBuilder builder() {
      return FrameVerdictMeasuredBuilder.builder();
    }
  }

  /** What the estimator decided about one camera measurement, by tag. */
  static FrameVerdict of(VisionDecision decision) {
    VisionMeasurement measurement = decision.measurement();
    Verdict verdict = decision.verdict();
    List<TagVerdict> tags = new ArrayList<>(measurement.tagIds().size());
    for (int id : measurement.tagIds()) {
      tags.add(
          TagVerdict.builder()
              .tag(id)
              .verdict(verdict)
              .innovationMeters(decision.innovationMeters())
              .chiSquared(decision.chiSquared())
              .threshold(decision.threshold())
              .value(decision.chiSquared())
              .build());
    }
    return builder()
        .camera(measurement.camera())
        .captureNanos(measurement.timestampNanos())
        .verdict(verdict)
        .threshold(decision.threshold())
        .value(decision.chiSquared())
        .multiTag(
            measurement.solver() == VisionMeasurement.Solver.MULTI_TAG
                ? Optional.of(verdict)
                : Optional.empty())
        .tags(tags)
        .measured(
            Optional.of(
                Measured.builder()
                    .pose(measurement.pose())
                    .solver(measurement.solver())
                    .innovationMeters(decision.innovationMeters())
                    .chiSquared(decision.chiSquared())
                    .build()))
        .build();
  }

  /**
   * A measurement given to a localizer that does not report what became of it: every tag it rested
   * on, given, and nothing compared.
   */
  static FrameVerdict given(VisionMeasurement measurement) {
    List<TagVerdict> tags = new ArrayList<>(measurement.tagIds().size());
    for (int id : measurement.tagIds()) {
      tags.add(TagVerdict.builder().tag(id).verdict(Verdict.GIVEN).build());
    }
    return builder()
        .camera(measurement.camera())
        .captureNanos(measurement.timestampNanos())
        .verdict(Verdict.GIVEN)
        .tags(tags)
        .measured(
            Optional.of(
                Measured.builder().pose(measurement.pose()).solver(measurement.solver()).build()))
        .build();
  }

  /** An image the robot held back, solved: every tag it saw, held. */
  static FrameVerdict held(TagDetections image, Verdict why) {
    List<TagVerdict> tags = new ArrayList<>(image.tags().size());
    for (TagDetections.Tag tag : image.tags()) {
      tags.add(TagVerdict.builder().tag(tag.id()).verdict(why).build());
    }
    return builder()
        .camera(image.camera())
        .captureNanos(image.timestampNanos())
        .verdict(why)
        .tags(tags)
        .build();
  }

  /** An image the pipeline refused, before it became a measurement: every tag it saw, refused. */
  static FrameVerdict refused(VisionResult result) {
    Verdict verdict = result.refused();
    if (verdict == null) {
      throw new IllegalArgumentException("the image became a measurement: it wasn't refused");
    }
    TagDetections image = result.detections();
    List<TagVerdict> tags = new ArrayList<>(image.tags().size());
    for (TagDetections.Tag tag : image.tags()) {
      tags.add(
          TagVerdict.builder()
              .tag(tag.id())
              .verdict(verdict)
              .threshold(result.threshold())
              .value(result.value())
              .build());
    }
    return builder()
        .camera(image.camera())
        .captureNanos(image.timestampNanos())
        .verdict(verdict)
        .threshold(result.threshold())
        .value(result.value())
        .tags(tags)
        .build();
  }
}
