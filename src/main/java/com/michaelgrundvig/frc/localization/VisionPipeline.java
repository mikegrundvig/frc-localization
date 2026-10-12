package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Turns one camera image's AprilTags into a measurement for the estimator, or says why not.
 *
 * <p>How it solves, in order of preference:
 *
 * <ol>
 *   <li><b>Heading-constrained</b>, whenever the estimate is sure of the heading: each tag fixes
 *       only the position, using the robot's heading from the gyro. Of the two poses a single tag
 *       allows, the one whose heading agrees with the gyro is used, which removes single-tag
 *       ambiguity, and each tag's known height checks the view. The idea behind Limelight's
 *       MegaTag2 and PhotonVision's constrained solve. Such a position leans on the gyro's heading,
 *       so the measurement says how far its tags were ({@link VisionMeasurement#lever}): the
 *       estimator carries the heading's doubt in it.
 *   <li><b>Multi-tag</b>, the vision software's solution from every tag at once, when the heading
 *       isn't known yet (a robot not yet placed).
 *   <li><b>Single tag</b>, the least ambiguous one, only if its ambiguity is low.
 * </ol>
 *
 * <p>Uncertainty grows with the tags' distance (squared, by default), shrinks with their count, and
 * grows with the robot's speed and turning (blur). A camera whose mount is in doubt (its tags land
 * off the same way every time: {@link MountCheck}) adds that doubt, as independent errors add. A
 * result may correct heading only from a multi-tag solution of several tags, with the robot barely
 * turning and the solution fitting well; otherwise the gyro keeps heading to itself. Everything
 * thrown out says why, as a {@link Verdict}.
 *
 * <p>A task may trust only some tags ({@link TagAllowlist}): the image is solved from those alone,
 * and the vision software's multi-tag solution only if it used no others.
 *
 * <p>Pure logic: the field's layout, the camera's mount, and the robot at the capture time all come
 * in as arguments.
 */
public final class VisionPipeline {
  /** An alternate pose closer to the camera than this is no pose: the vision software gave none. */
  private static final double NO_ALTERNATE_BELOW_METERS = 1e-6;

  private final FieldModel field;

  public VisionPipeline(FieldModel field) {
    this.field = field;
  }

  /**
   * How far to trust one camera's image beyond its configuration, for the moment.
   *
   * @param mountDoubtMeters how far the camera's measurements may be off from where its mount moved
   *     since it was calibrated ({@link MountCheck#doubtMeters}): zero for a camera that sits where
   *     its configuration says
   * @param onlyTags the tags the current task trusts; null for every tag
   */
  public record Trust(double mountDoubtMeters, @Nullable TagAllowlist onlyTags) {
    /** A camera where its configuration says, and every tag. */
    public static final Trust FULL = new Trust(0, null);

    public Trust {
      if (!(mountDoubtMeters >= 0) || Double.isInfinite(mountDoubtMeters)) {
        throw new IllegalArgumentException("a mount's doubt is zero or more: " + mountDoubtMeters);
      }
    }
  }

  /** One image, and what it's solved with: its camera's mount on the robot as it was taken. */
  private record Image(
      TagDetections seen,
      CameraConfig camera,
      Transform3d mount,
      CaptureContext robot,
      VisionConfig config,
      double mountDoubtMeters) {
    /**
     * How much a result's uncertainty grows from the one-tag-at-one-meter baseline: with distance,
     * fewer tags, the camera's own noise, and the robot's motion.
     */
    double scale(int tags, double distance) {
      double motion =
          1
              + config.speedStdDevFactor() * robot.speedMetersPerSecond()
              + config.yawRateStdDevFactor() * robot.recentYawRateRadiansPerSecond();
      return Math.pow(distance, config.distanceExponent()) / tags * camera.noiseScale() * motion;
    }
  }

  /** What to make of one image from a camera on the chassis, where its configuration mounts it. */
  public VisionResult process(
      TagDetections seen,
      CameraConfig camera,
      CaptureContext robot,
      VisionConfig config,
      Trust trust) {
    return process(seen, camera, camera.robotToCamera(), robot, config, trust);
  }

  /**
   * What to make of one image, its camera where it was on the robot as the image was taken: on a
   * joint, where the joint had carried it.
   *
   * @param mount the camera's pose in the robot's frame as the image was taken
   */
  public VisionResult process(
      TagDetections seen,
      CameraConfig camera,
      Transform3d mount,
      CaptureContext robot,
      VisionConfig config,
      Trust trust) {
    TagAllowlist onlyTags = trust.onlyTags();
    if (onlyTags == null) {
      return solve(new Image(seen, camera, mount, robot, config, trust.mountDoubtMeters()));
    }
    TagDetections trusted = trusted(seen, onlyTags);
    if (trusted.tags().isEmpty() && !seen.tags().isEmpty()) {
      return new VisionResult(
          seen,
          null,
          Verdict.UNTRUSTED_TAGS,
          Verdict.UNTRUSTED_TAGS.words() + " (" + onlyTags.describe() + ")",
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN);
    }
    return solve(new Image(trusted, camera, mount, robot, config, trust.mountDoubtMeters()));
  }

  /**
   * An image as a task sees it: only the tags it trusts, and the vision software's multi-tag
   * solution only if that used no others.
   */
  private static TagDetections trusted(TagDetections seen, TagAllowlist onlyTags) {
    List<TagDetections.Tag> tags =
        seen.tags().stream().filter(tag -> onlyTags.allows(tag.id())).toList();
    TagDetections.MultiTag multiTag = seen.multiTag();
    boolean allTrusted = multiTag != null && multiTag.tagIds().stream().allMatch(onlyTags::allows);
    return new TagDetections(
        seen.timestampNanos(), seen.camera(), tags, allTrusted ? multiTag : null);
  }

  /**
   * Solves an image, by the first of the three ways that applies (the class comment says which and
   * why), or refuses it.
   */
  private VisionResult solve(Image image) {
    TagDetections seen = image.seen();
    CaptureContext robot = image.robot();
    VisionConfig config = image.config();
    List<TagDetections.Tag> known = tagsTheFieldHas(seen);
    if (known.isEmpty()) {
      return rejected(seen, Verdict.UNKNOWN_TAGS, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }
    double distance = averageDistance(known);
    if (!Double.isFinite(distance)) {
      return rejected(
          seen, Verdict.TAG_NOT_A_NUMBER, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }
    if (robot.recentYawRateRadiansPerSecond() > config.maxYawRateRadiansPerSecond()) {
      return rejected(
          seen,
          Verdict.TURNING,
          robot.recentYawRateRadiansPerSecond(),
          config.maxYawRateRadiansPerSecond(),
          distance,
          Double.NaN);
    }
    if (distance > config.maxDistanceMeters()) {
      return rejected(
          seen, Verdict.TOO_FAR, distance, config.maxDistanceMeters(), distance, Double.NaN);
    }

    // The gyro's heading solves the image only while the estimate is sure of it.
    Rotation2d heading = robot.heading();
    if (heading != null && robot.headingStdDevRadians() <= config.maxHeadingStdDevToConstrain()) {
      return headingConstrained(image, known, heading, distance);
    }
    TagDetections.MultiTag multiTag = seen.multiTag();
    if (multiTag != null) {
      Pose3d pose = robotFromMultiTag(multiTag, image.mount());
      return fullPose(image, pose, known, distance, VisionMeasurement.Solver.MULTI_TAG);
    }
    TagDetections.Tag clearest = leastAmbiguous(known);
    if (clearest.ambiguity() > config.maxAmbiguity()) {
      return rejected(
          seen,
          Verdict.AMBIGUOUS,
          clearest.ambiguity(),
          config.maxAmbiguity(),
          distance,
          Double.NaN);
    }
    Pose3d pose =
        TagSolver.robotFromTag(
            field.tag(clearest.id()).orElseThrow(), clearest.bestCameraToTag(), image.mount());
    return fullPose(
        image,
        pose,
        List.of(clearest),
        clearest.bestCameraToTag().getTranslation().getNorm(),
        VisionMeasurement.Solver.SINGLE_TAG);
  }

  /** The tags in an image that the field's layout has: only those can place the robot. */
  private List<TagDetections.Tag> tagsTheFieldHas(TagDetections seen) {
    List<TagDetections.Tag> known = new ArrayList<>();
    for (TagDetections.Tag tag : seen.tags()) {
      if (field.tag(tag.id()).isPresent()) {
        known.add(tag);
      }
    }
    return known;
  }

  /** How far the tags are from the camera, on average, by each one's best pose. */
  private static double averageDistance(List<TagDetections.Tag> tags) {
    double sum = 0;
    for (TagDetections.Tag tag : tags) {
      sum += tag.bestCameraToTag().getTranslation().getNorm();
    }
    return sum / tags.size();
  }

  /** The tag whose two possible poses are furthest from equally likely. The first, if they tie. */
  private static TagDetections.Tag leastAmbiguous(List<TagDetections.Tag> tags) {
    TagDetections.Tag clearest = tags.get(0);
    for (TagDetections.Tag tag : tags) {
      if (tag.ambiguity() < clearest.ambiguity()) {
        clearest = tag;
      }
    }
    return clearest;
  }

  /** The robot's pose, from the vision software's solution for the camera's pose on the field. */
  private static Pose3d robotFromMultiTag(TagDetections.MultiTag multiTag, Transform3d mount) {
    Pose3d fieldToCamera =
        new Pose3d(
            multiTag.fieldToCamera().getTranslation(), multiTag.fieldToCamera().getRotation());
    return TagSolver.robotFromCamera(fieldToCamera, mount);
  }

  /**
   * The working of one heading-constrained solve: the sums that give where the robot stands as the
   * average of where each tag puts it. It lives for one image.
   *
   * <p>A tag counts by one over its distance squared, since a nearer tag is seen that much more
   * sharply. The same weights average the vector from the robot to each tag, which is the lever a
   * heading error swings the position by, and the weights are handed on as each tag's share of the
   * position.
   */
  private static final class WeightedFixes {
    /** A tag nearer than the square root of this, in meters, weighs as if it were that far. */
    private static final double LEAST_DISTANCE_SQUARED = 1e-6;

    private double sumOfWeights;
    private double weightedX;
    private double weightedY;
    private double weightedToTagX;
    private double weightedToTagY;
    private final List<Double> weights = new ArrayList<>();

    /** The largest miss of any tag's known height, with its sign: zero before any tag. */
    private double worstHeightErrorMeters;

    void add(Pose3d fieldToTag, TagSolver.PositionFix fix, double tagDistanceMeters) {
      double weight = 1 / Math.max(tagDistanceMeters * tagDistanceMeters, LEAST_DISTANCE_SQUARED);
      weightedX += weight * fix.position().getX();
      weightedY += weight * fix.position().getY();
      // What the heading turned to place the robot: from where it puts the robot, to the tag.
      weightedToTagX += weight * (fieldToTag.getX() - fix.position().getX());
      weightedToTagY += weight * (fieldToTag.getY() - fix.position().getY());
      sumOfWeights += weight;
      weights.add(weight);
      if (Math.abs(fix.heightErrorMeters()) > Math.abs(worstHeightErrorMeters)) {
        worstHeightErrorMeters = fix.heightErrorMeters();
      }
    }

    Translation2d position() {
      return new Translation2d(weightedX / sumOfWeights, weightedY / sumOfWeights);
    }

    Translation2d robotToTags() {
      return new Translation2d(weightedToTagX / sumOfWeights, weightedToTagY / sumOfWeights);
    }
  }

  /**
   * Solves an image with the robot's heading from the gyro: each tag fixes only where the robot
   * stands, and the fixes are averaged.
   *
   * @param heading the robot's heading when the image was taken
   * @param distance the tags' average distance from the camera
   */
  private VisionResult headingConstrained(
      Image image, List<TagDetections.Tag> known, Rotation2d heading, double distance) {
    TagDetections seen = image.seen();
    CaptureContext robot = image.robot();
    VisionConfig config = image.config();
    Transform3d robotToCamera = image.mount();
    Rotation3d attitude =
        new Rotation3d(robot.rollRadians(), robot.pitchRadians(), heading.getRadians());
    WeightedFixes fixes = new WeightedFixes();
    for (TagDetections.Tag tag : known) {
      Pose3d fieldToTag = field.tag(tag.id()).orElseThrow();
      Transform3d cameraToTag = agreeingWithTheGyro(tag, fieldToTag, robotToCamera, heading);
      TagSolver.PositionFix fix =
          TagSolver.positionFromTag(fieldToTag, robotToCamera.plus(cameraToTag), attitude);
      fixes.add(fieldToTag, fix, cameraToTag.getTranslation().getNorm());
    }
    double worstHeight = fixes.worstHeightErrorMeters;
    // Written so a height error that isn't a number is refused too.
    boolean heightsFit = Math.abs(worstHeight) <= config.maxHeightErrorMeters();
    if (!heightsFit) {
      return rejected(
          seen,
          Verdict.TAG_HEIGHT,
          worstHeight,
          config.maxHeightErrorMeters(),
          distance,
          worstHeight);
    }
    Translation2d position = fixes.position();
    if (!Double.isFinite(position.getX()) || !Double.isFinite(position.getY())) {
      return rejected(
          seen, Verdict.SOLVE_NOT_A_NUMBER, Double.NaN, Double.NaN, distance, worstHeight);
    }
    double spread =
        widened(
            config.linearStdDevBaseline()
                * config.headingConstrainedFactor()
                * image.scale(known.size(), distance),
            image.mountDoubtMeters());

    // The measurement's heading is the gyro's own, and says nothing new about it, unless the
    // vision software's multi-tag solution is good enough to correct the heading with.
    Rotation2d measuredHeading = heading;
    double headingStdDev = Double.POSITIVE_INFINITY;
    TagDetections.MultiTag multiTag = seen.multiTag();
    if (multiTag != null && mayCorrectHeading(multiTag, robot, config)) {
      Rotation2d solved = robotFromMultiTag(multiTag, robotToCamera).toPose2d().getRotation();
      if (Double.isFinite(solved.getRadians())) {
        measuredHeading = solved;
        headingStdDev =
            widened(
                config.angularStdDevBaseline() * image.scale(known.size(), distance),
                image.mountDoubtMeters() / distance);
      }
    }
    VisionMeasurement measurement =
        new VisionMeasurement(
            seen.timestampNanos(),
            new Pose2d(position, measuredHeading),
            PoseCovariance.fromStdDevs(spread, spread, headingStdDev),
            new VisionMeasurement.Lever(heading, fixes.robotToTags()),
            image.camera().name(),
            ids(known),
            fixes.weights,
            VisionMeasurement.Solver.HEADING_CONSTRAINED);
    return new VisionResult(
        seen,
        measurement,
        null,
        format(
            "heading-constrained: %d %s at %.1f m%s",
            known.size(),
            known.size() == 1 ? "tag" : "tags",
            distance,
            Double.isFinite(headingStdDev) ? ", with heading" : ""),
        distance,
        worstHeight,
        Double.NaN,
        Double.NaN);
  }

  /**
   * Whether a multi-tag solution may correct the heading. Only under strict conditions: it used
   * enough tags, the robot was barely turning, and the solution fits its tags well. Otherwise the
   * gyro keeps the heading to itself.
   */
  private static boolean mayCorrectHeading(
      TagDetections.MultiTag multiTag, CaptureContext robot, VisionConfig config) {
    boolean enoughTags = multiTag.tagIds().size() >= config.minTagsForHeading();
    boolean barelyTurning = robot.recentYawRateRadiansPerSecond() <= config.maxYawRateForHeading();
    boolean fitsWell =
        multiTag.reprojectionErrorPixels() <= config.maxReprojectionErrorForHeadingPixels();
    return enoughTags && barelyTurning && fitsWell;
  }

  /**
   * Of a tag's two possible poses, the one whose robot heading is closer to the gyro's. The best
   * pose is kept unless the alternate is strictly closer, and when there is no alternate (the
   * vision software gives a zero transform for none).
   */
  private static Transform3d agreeingWithTheGyro(
      TagDetections.Tag tag, Pose3d fieldToTag, Transform3d robotToCamera, Rotation2d heading) {
    Transform3d best = tag.bestCameraToTag();
    Transform3d alternate = tag.altCameraToTag();
    boolean hasAlternate = !(alternate.getTranslation().getNorm() < NO_ALTERNATE_BELOW_METERS);
    if (!hasAlternate) {
      return best;
    }
    double bestMiss = headingMiss(fieldToTag, best, robotToCamera, heading);
    double alternateMiss = headingMiss(fieldToTag, alternate, robotToCamera, heading);
    return alternateMiss < bestMiss ? alternate : best;
  }

  /** How far the robot heading one of a tag's poses gives is from the gyro's, in radians. */
  private static double headingMiss(
      Pose3d fieldToTag, Transform3d cameraToTag, Transform3d robotToCamera, Rotation2d heading) {
    return Math.abs(
        TagSolver.robotFromTag(fieldToTag, cameraToTag, robotToCamera)
            .toPose2d()
            .getRotation()
            .minus(heading)
            .getRadians());
  }

  /**
   * A measurement from a full pose, one that found its own heading: the vision software's multi-tag
   * solution, or a single tag's best pose. It is refused if it does not put the robot on the floor.
   *
   * @param used the tags the pose came from
   * @param distance how far those tags are from the camera, on average
   */
  private VisionResult fullPose(
      Image image,
      Pose3d pose,
      List<TagDetections.Tag> used,
      double distance,
      VisionMeasurement.Solver solver) {
    TagDetections seen = image.seen();
    VisionConfig config = image.config();
    double height = pose.getZ();
    if (!Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getZ())) {
      return rejected(seen, Verdict.SOLVE_NOT_A_NUMBER, Double.NaN, Double.NaN, distance, height);
    }
    // Written so a height that isn't a number is refused too.
    boolean onTheFloor = Math.abs(height) <= config.maxHeightErrorMeters();
    if (!onTheFloor) {
      return rejected(
          seen, Verdict.OFF_THE_FLOOR, height, config.maxHeightErrorMeters(), distance, height);
    }
    double scale = image.scale(used.size(), distance);
    double linear = widened(config.linearStdDevBaseline() * scale, image.mountDoubtMeters());
    double angular =
        widened(config.angularStdDevBaseline() * scale, image.mountDoubtMeters() / distance);
    VisionMeasurement measurement =
        new VisionMeasurement(
            seen.timestampNanos(),
            pose.toPose2d(),
            PoseCovariance.fromStdDevs(linear, linear, angular),
            VisionMeasurement.Lever.NONE,
            image.camera().name(),
            ids(used),
            solver);
    return new VisionResult(
        seen,
        measurement,
        null,
        format(
            "%s: %d %s at %.1f m",
            solver == VisionMeasurement.Solver.MULTI_TAG ? "multi-tag" : "single tag",
            used.size(),
            used.size() == 1 ? "tag" : "tags",
            distance),
        distance,
        height,
        Double.NaN,
        Double.NaN);
  }

  /**
   * A spread with the camera's mount doubt added, as independent errors add: in squares. For
   * heading, a doubt in meters is a turn of the doubt over the tags' distance: a camera turned by
   * an angle puts tags that far off at that distance.
   *
   * <p>Only widened, never shut off: a heading error in the estimate puts every camera's tags off
   * as if each were turned on its mount, and from one camera the two look the same. A camera in
   * doubt that couldn't correct heading at all would keep the estimate from ever turning back.
   *
   * <p>A wider spread also loosens the estimator's gate for the measurement, which weighs how far
   * it lands from the estimate against both their spreads: a doubted camera's measurements are
   * rejected less often, and count for less when they're accepted.
   */
  private static double widened(double spread, double doubt) {
    return doubt > 0 ? Math.hypot(spread, doubt) : spread;
  }

  private static List<Integer> ids(List<TagDetections.Tag> tags) {
    return tags.stream().map(TagDetections.Tag::id).toList();
  }

  /**
   * An image thrown out, and why: the verdict, with its number and the limit it was held to in its
   * words for the log.
   */
  private static VisionResult rejected(
      TagDetections seen,
      Verdict why,
      double value,
      double threshold,
      double distance,
      double heightError) {
    return new VisionResult(
        seen, null, why, why.describe(value, threshold), distance, heightError, threshold, value);
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }
}
