package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.SplittableRandom;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.util.MathUtil;

/**
 * A flat world for the estimator's tests of cameras' turns on their mounts: a robot driven by
 * velocity over a field ringed with tags, four cameras at its corners facing 45° out, and each
 * camera's image solved as the vision pipeline solves one with the gyro's heading.
 *
 * <p>Each camera may really be turned on its mount by an angle of its own ({@link #turn}), which
 * the estimator is not told. An image is solved with the estimate's heading as of when it was
 * taken, through the camera's configured mount: so a camera turned by an angle puts each tag, and
 * the robot, swung about the tag by that angle, and gives a heading off by that angle, as a real
 * one does. Each image's own noise is a fifth of what it claims, as the simulator's cameras' is.
 *
 * <p>The world is plane geometry, everything on the floor, so that a test can say exactly what each
 * camera sees and why.
 */
final class MountWorld {
  static final long STEP_NANOS = 4_000_000L; // odometry at 250 Hz
  static final long LOOP_NANOS = 20_000_000L; // the robot's loop
  static final long IMAGE_NANOS = 33_000_000L; // each camera at 30 images a second
  static final long LATENCY_NANOS = 30_000_000L;

  static final double FIELD_LENGTH = 16;
  static final double FIELD_WIDTH = 8;

  /** How far a camera sees a tag, and how wide its view is, each way from where it faces. */
  static final double SIGHT_METERS = 5;

  static final double HALF_VIEW_RADIANS = Math.toRadians(35);

  /** The cameras' names, at the front left, front right, back left and back right corners. */
  static final List<String> CAMERAS =
      List.of("front-left", "front-right", "back-left", "back-right");

  private static final double CORNER = 0.3;

  // The pipeline's noise model, for what an image claims: its baselines, over a distance squared
  // and the tags' count.
  private static final double LINEAR_BASELINE = 0.02 * 0.5;
  private static final double ANGULAR_BASELINE = 0.03;

  /** The real noise is this share of what an image claims. */
  private static final double REAL_SHARE = 0.2;

  /** One of the field's tags: where it is, and which way it faces. */
  record Tag(int id, Translation2d at, double facingRadians) {}

  final List<Tag> tags = new ArrayList<>();

  /** Where each tag really is, for a tag surveyed off the layout: the layout's place otherwise. */
  final List<Translation2d> realTags = new ArrayList<>();

  final PoseEstimator estimator;

  /** Each camera's real turn on its mount, by its index in {@link #CAMERAS}, in radians. */
  final double[] turn = new double[CAMERAS.size()];

  /** Each camera's configured mount: where it sits on the robot and which way it faces. */
  private final Translation2d[] mountAt = new Translation2d[CAMERAS.size()];

  private final double[] mountFacing = new double[CAMERAS.size()];

  private final SplittableRandom random;

  long time = 1_000_000_000L;
  Pose2d truth;
  double gyroYaw = 0.4; // the gyro started somewhere of its own

  /** A share of the wheels' travel the wheels miss: 0.3 for a robot whose wheels slip by 30%. */
  double wheelsMiss;

  /**
   * A camera that reads one tag as another, by their indices in {@link #tags}: what it sees of the
   * first it puts at the second. -1 for none.
   */
  int misreadCamera = -1;

  int misreadTag;
  int misreadAs;

  private final long[] nextImage = new long[CAMERAS.size()];
  private final List<Pending> inFlight = new ArrayList<>();

  /** An image on its way to the robot: it arrives a latency after it was taken. */
  private record Pending(long arrives, int camera, long taken, Pose2d truthThen) {}

  /**
   * @param config how the estimator weighs its sources
   * @param tolerance how far the estimator is told each camera's mount may be off
   * @param start where the robot starts, which it is placed on as surely as given
   * @param placedHeadingStdDev how sure the placement's heading is, in radians
   */
  MountWorld(
      EstimatorConfig config,
      CameraConfig.MountTolerance tolerance,
      Pose2d start,
      double placedHeadingStdDev,
      long seed) {
    random = new SplittableRandom(seed);
    int id = 1;
    for (double x = 1; x < FIELD_LENGTH; x += 2) {
      id = addTag(id, new Translation2d(x, 0), Math.PI / 2);
      id = addTag(id, new Translation2d(x, FIELD_WIDTH), -Math.PI / 2);
    }
    for (double y = 1; y < FIELD_WIDTH; y += 2) {
      id = addTag(id, new Translation2d(0, y), 0);
      id = addTag(id, new Translation2d(FIELD_LENGTH, y), Math.PI);
    }
    double[][] corners = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    for (int camera = 0; camera < CAMERAS.size(); camera++) {
      mountAt[camera] = new Translation2d(CORNER * corners[camera][0], CORNER * corners[camera][1]);
      mountFacing[camera] = Math.atan2(corners[camera][1], corners[camera][0]);
      nextImage[camera] = time + camera * IMAGE_NANOS / CAMERAS.size();
    }
    List<Integer> ids = tags.stream().map(Tag::id).toList();
    estimator =
        new PoseEstimator(
            config,
            point -> point.getX() > -1 && point.getX() < FIELD_LENGTH + 1,
            ids,
            TestCameras.mounted(tolerance, CAMERAS.toArray(String[]::new)),
            new PoseReset(0, start, PoseCovariance.fromStdDevs(0.05, 0.05, 1), "start"));
    truth = start;
    estimator.addOdometry(time, new BodyMotion(0, 0, 0, 0), gyroYaw, YawSource.GYRO);
    estimator.reset(
        new PoseReset(
            time, start, PoseCovariance.fromStdDevs(0.05, 0.05, placedHeadingStdDev), "placed"));
  }

  private int addTag(int id, Translation2d at, double facing) {
    tags.add(new Tag(id, at, facing));
    realTags.add(at);
    return id + 1;
  }

  /** Drives at field-relative velocities for some seconds, a loop at a time. */
  MountWorld drive(double seconds, double vx, double vy, double omega) {
    long until = time + Math.round(seconds * 1e9);
    while (time < until) {
      long loopEnds = time + LOOP_NANOS;
      while (time < loopEnds) {
        step(vx, vy, omega);
      }
      deliverImages();
      estimator.update();
    }
    return this;
  }

  /**
   * Drives a loop around the middle of the field, 4 m by 2 m, at 1.5 m a second, turning slowly the
   * whole way: every camera sees the walls' tags from many places and angles.
   */
  MountWorld driveLoops(int loops) {
    for (int loop = 0; loop < loops; loop++) {
      drive(4 / 1.5, 1.5, 0, 0.3);
      drive(2 / 1.5, 0, 1.5, 0.3);
      drive(4 / 1.5, -1.5, 0, 0.3);
      drive(2 / 1.5, 0, -1.5, 0.3);
    }
    return this;
  }

  /** Moves the robot where its wheels cannot see it: a shove. */
  MountWorld shove(double dx, double dy) {
    truth = new Pose2d(truth.getX() + dx, truth.getY() + dy, truth.getRotation());
    return this;
  }

  private void step(double vx, double vy, double omega) {
    double dt = STEP_NANOS / 1e9;
    double heading = truth.getRotation().getRadians();
    // The field's velocity, in the robot's frame as the step starts.
    double forward = (Math.cos(heading) * vx + Math.sin(heading) * vy) * dt;
    double left = (-Math.sin(heading) * vx + Math.cos(heading) * vy) * dt;
    truth = truth.plus(new Twist2d(forward, left, omega * dt).exp());
    time += STEP_NANOS;
    gyroYaw += omega * dt;
    double seen = 1 - wheelsMiss;
    estimator.addOdometry(
        time, new BodyMotion(seen * forward, seen * left, omega * dt, 0), gyroYaw, YawSource.GYRO);
    for (int camera = 0; camera < CAMERAS.size(); camera++) {
      if (time >= nextImage[camera]) {
        inFlight.add(new Pending(time + LATENCY_NANOS, camera, time, truth));
        nextImage[camera] += IMAGE_NANOS;
      }
    }
  }

  private void deliverImages() {
    List<Pending> arrived = new ArrayList<>();
    for (Pending image : inFlight) {
      if (image.arrives() <= time) {
        arrived.add(image);
      }
    }
    inFlight.removeAll(arrived);
    for (Pending image : arrived) {
      VisionMeasurement measurement = solve(image);
      if (measurement != null) {
        estimator.addVision(measurement);
      }
    }
  }

  /**
   * Solves one image as the pipeline does with the gyro's heading: each tag in view fixes where the
   * robot stands, through the camera's configured mount and the estimate's heading when the image
   * was taken, and the fixes are averaged, nearer tags counting more. With two tags or more and the
   * robot barely turning, the image gives a heading of its own, the camera's own less its mount.
   */
  private @Nullable VisionMeasurement solve(Pending image) {
    int camera = image.camera();
    Pose2d real = image.truthThen();
    double usedHeading =
        estimator
            .poseAt(image.taken())
            .map(pose -> pose.getRotation().getRadians())
            .orElse(real.getRotation().getRadians());
    // Where the camera really is, and which way it really faces.
    Translation2d cameraAt =
        real.getTranslation().plus(mountAt[camera].rotateBy(real.getRotation()));
    double realFacing = real.getRotation().getRadians() + mountFacing[camera] + turn[camera];
    double weights = 0;
    double sumX = 0;
    double sumY = 0;
    double leverX = 0;
    double leverY = 0;
    double distanceSum = 0;
    List<Integer> ids = new ArrayList<>();
    List<Double> shares = new ArrayList<>();
    for (int index = 0; index < tags.size(); index++) {
      Tag tag = tags.get(index);
      Translation2d realTag = realTags.get(index);
      Translation2d toTag = realTag.minus(cameraAt);
      double distance = toTag.getNorm();
      double toTagAngle = Math.atan2(toTag.getY(), toTag.getX());
      double bearing = MathUtil.angleModulus(toTagAngle - realFacing);
      double facingUs = Math.cos(toTagAngle + Math.PI - tag.facingRadians());
      boolean inView =
          distance < SIGHT_METERS && Math.abs(bearing) < HALF_VIEW_RADIANS && facingUs > 0.3;
      if (!inView) {
        continue;
      }
      int as = index;
      if (camera == misreadCamera && index == misreadTag) {
        as = misreadAs;
      }
      Tag placedAt = tags.get(as);
      // The camera sees the tag at a distance and a bearing in its own frame; the solve turns that
      // onto the field by the heading it was given and the configured mount, from the tag the
      // layout has.
      Translation2d seenFromCamera = toTag.rotateBy(new Rotation2d(-realFacing));
      Translation2d cameraSolved =
          placedAt
              .at()
              .minus(seenFromCamera.rotateBy(new Rotation2d(usedHeading + mountFacing[camera])));
      Translation2d robotSolved =
          cameraSolved.minus(mountAt[camera].rotateBy(new Rotation2d(usedHeading)));
      double weight = 1 / (distance * distance);
      weights += weight;
      sumX += weight * robotSolved.getX();
      sumY += weight * robotSolved.getY();
      leverX += weight * (placedAt.at().getX() - robotSolved.getX());
      leverY += weight * (placedAt.at().getY() - robotSolved.getY());
      distanceSum += distance;
      ids.add(placedAt.id());
      shares.add(weight);
    }
    if (ids.isEmpty()) {
      return null;
    }
    int count = ids.size();
    double distance = distanceSum / count;
    double claimed = LINEAR_BASELINE * distance * distance / count;
    double x = sumX / weights + REAL_SHARE * claimed * random.nextGaussian();
    double y = sumY / weights + REAL_SHARE * claimed * random.nextGaussian();
    double headingStdDev = Double.POSITIVE_INFINITY;
    double heading = usedHeading;
    boolean withHeading = count >= 2;
    if (withHeading) {
      headingStdDev = ANGULAR_BASELINE * distance * distance / count;
      heading =
          real.getRotation().getRadians()
              + turn[camera]
              + REAL_SHARE * headingStdDev * random.nextGaussian();
    }
    return new VisionMeasurement(
        image.taken(),
        new Pose2d(x, y, new Rotation2d(heading)),
        PoseCovariance.fromStdDevs(claimed, claimed, headingStdDev),
        new VisionMeasurement.Lever(
            new Rotation2d(usedHeading),
            new Translation2d(leverX / weights, leverY / weights),
            mountAt[camera].rotateBy(new Rotation2d(usedHeading))),
        CAMERAS.get(camera),
        ids,
        shares,
        VisionMeasurement.Solver.HEADING_CONSTRAINED);
  }

  /** How far the estimate is from the truth, in meters. */
  double positionError() {
    return estimator.pose().getTranslation().getDistance(truth.getTranslation());
  }

  /** How far the estimate's heading is from the truth, in radians. */
  double headingError() {
    return estimator.pose().getRotation().minus(truth.getRotation()).getRadians();
  }
}
