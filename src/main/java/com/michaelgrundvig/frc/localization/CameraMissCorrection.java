package com.michaelgrundvig.frc.localization;

import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;

/**
 * Takes off each camera measurement what the watch on the cameras knows its camera misses by
 * ({@link CameraWatch}), and hands the watch each accepted measurement's own miss.
 *
 * <p>Only a position solved with the gyro's heading is watched and corrected: the few measurements
 * solved another way err another way.
 */
final class CameraMissCorrection {
  private final CameraWatch watch;

  /**
   * A measurement's innovation, with what its camera is known to miss by taken off where that
   * brings it nearer the estimate.
   *
   * @param innovation the innovation, or null when the measurement can't be compared with the
   *     estimate at all
   * @param missX what was taken off along the field's x, in meters: zero where nothing was
   * @param missY and along y
   * @param lateArmX how far a second of lateness would put the measurement off along the field's x
   *     ({@link CameraWatch#lateArmX}): zero for a measurement that isn't watched
   * @param lateArmY and along y
   */
  record Corrected(
      @Nullable Innovation innovation,
      double missX,
      double missY,
      double lateArmX,
      double lateArmY) {}

  CameraMissCorrection(CameraWatch watch) {
    this.watch = watch;
  }

  /**
   * Takes the camera's known miss off a measurement's innovation, where that brings it nearer the
   * estimate. Where it doesn't, this isn't one of the images the camera's average miss is made of
   * (a camera that misread a tag for a while reads the next one rightly), and the measurement
   * stands as it came.
   *
   * @param headingThen the robot's heading when the image was taken
   * @param asItCame the innovation with nothing taken off, or null when it can't be compared
   * @param doubtVariance the watch's doubt of the camera, as a variance, which the second
   *     innovation counts as more noise
   */
  Corrected correct(
      PoseFilter filter,
      VisionMeasurement measurement,
      int camera,
      int[] tagPlaces,
      OdometryAtImage then,
      double headingThen,
      @Nullable Innovation asItCame,
      double doubtVariance) {
    if (asItCame == null || !measurement.leansOnTheHeading()) {
      return new Corrected(asItCame, 0, 0, 0, 0);
    }
    Translation2d toTags = measurement.lever().robotToTags();
    double lateArmX = CameraWatch.lateArmX(toTags.getY(), then.velocityX(), then.yawRate());
    double lateArmY = CameraWatch.lateArmY(toTags.getX(), then.velocityY(), then.yawRate());
    double offX = watch.offX(camera, headingThen, lateArmX);
    double offY = watch.offY(camera, headingThen, lateArmY);
    double asItCameMeters = Math.hypot(asItCame.xMeters(), asItCame.yMeters());
    double withItTakenOff = Math.hypot(asItCame.xMeters() - offX, asItCame.yMeters() - offY);
    boolean nearer = withItTakenOff < asItCameMeters;
    if (nearer) {
      Innovation corrected =
          filter.innovation(measurement, then.yaw(), camera, tagPlaces, offX, offY, doubtVariance);
      return new Corrected(corrected, offX, offY, lateArmX, lateArmY);
    }
    return new Corrected(asItCame, 0, 0, lateArmX, lateArmY);
  }

  /**
   * Hands the watch an accepted measurement's own miss, as it was before anything was taken off,
   * from where odometry alone puts the robot. The innovation is the measurement, less what was
   * taken off, less the estimate, so both are added back. Call it before the filter is corrected.
   *
   * @param timeNanos the moment the measurement is applied at
   * @param innovation the innovation the gate passed
   * @param headingThen the robot's heading when the image was taken
   */
  void tellWatch(
      PoseFilter filter,
      int camera,
      long timeNanos,
      VisionMeasurement measurement,
      Innovation innovation,
      Corrected corrected,
      OdometryAtImage then,
      double headingThen) {
    if (!measurement.leansOnTheHeading()) {
      return;
    }
    if (ConsideredErrors.hasPlace(camera)) {
      double turnStdDev = filter.cameraTurnStdDev(camera);
      watch.allowTurn(camera, turnStdDev * turnStdDev);
    }
    watch.accepted(
        camera,
        timeNanos,
        innovation.xMeters() + corrected.missX() + filter.x() - then.x(),
        innovation.yMeters() + corrected.missY() + filter.y() - then.y(),
        headingThen,
        measurement.lever().robotToTags().getNorm(),
        (measurement.covariance().xx() + measurement.covariance().yy()) / 2,
        then.variance(),
        corrected.lateArmX(),
        corrected.lateArmY());
  }

  /**
   * How far the watch has a camera's measurements in doubt, in meters: how far they keep landing to
   * one side of the other cameras', beyond what noise explains.
   */
  double doubtMeters(int camera) {
    return watch.doubtMeters(camera);
  }

  /** How late the watch takes a camera's images to be stamped, in seconds: negative for early. */
  double lateSeconds(int camera) {
    return watch.lateSeconds(camera);
  }

  /** Forgets what the watch judged of a camera's mount: its misses were judged against its turn. */
  void forgetMount(int camera) {
    watch.forgetMount(camera);
  }
}
