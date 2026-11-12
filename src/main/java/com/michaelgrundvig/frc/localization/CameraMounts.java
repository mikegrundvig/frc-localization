package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * The cameras' mounts as the estimator weighs them: how unsure each camera's turn and shift start,
 * from its mount's tolerance; a sign that a camera was knocked, which opens its turn up to be
 * learned anew ({@link MountKnocks}); and the mount check's doubt about a camera, which only {@link
 * HeadingCorrection#HEADING_MEASUREMENTS_AND_MOUNT_DOUBT} takes.
 *
 * <p>A camera's turn starts with the variance of a turn drawn evenly within its tolerance. It may
 * then wander, as a mount settles under a match's driving and contact, by as much again over a
 * match: after two and a half minutes without a word from the camera, its turn is as unsure as when
 * it was mounted. Its shift adds the tolerance's offset to what the camera's solves repeat beyond
 * its mount ({@link EstimatorConfig#cameraShiftStdDevMeters}).
 */
final class CameraMounts {
  /** How long a match lasts, in seconds: 2 minutes 30 seconds. */
  static final double MATCH_SECONDS = 150;

  private final EstimatorConfig config;
  private final HeadingCorrection heading;
  private final ErrorPlaces places;
  private final PoseFilter filter;
  private final OdometryHistory history;
  private final CameraMissCorrection corrections;
  private final MountKnocks knocks;

  /**
   * How unsure each camera's turn on its mount starts, by its place, as a variance in radians
   * squared. A sign that it was knocked opens its turn up by at least this again.
   */
  private final double[] turnVariance = new double[ConsideredErrors.MAX_CAMERAS];

  /** How unsure each camera's shift is along each axis, by its place, before any mount doubt. */
  private final double[] shiftVariance = new double[ConsideredErrors.MAX_CAMERAS];

  /**
   * How far this update found each camera's turn must be opened up, as the variance to add to it,
   * by the camera's place: zero for none.
   */
  private final double[] turnsToReopen = new double[ConsideredErrors.MAX_CAMERAS];

  /** Each camera's doubt about its mount, as last told, for the next update to apply. */
  private final Map<String, MountDoubt> doubts = new HashMap<>();

  /**
   * Every camera starts as a mount measured from CAD and checked does ({@link
   * CameraConfig.MountTolerance#MEASURED}), until {@link #mount} says otherwise.
   */
  CameraMounts(
      EstimatorConfig config,
      HeadingCorrection heading,
      ErrorPlaces places,
      PoseFilter filter,
      OdometryHistory history,
      CameraMissCorrection corrections) {
    this.config = config;
    this.heading = heading;
    this.places = places;
    this.filter = filter;
    this.history = history;
    this.corrections = corrections;
    knocks = new MountKnocks(config.cameraWatchMeasurements());
    CameraConfig.MountTolerance measured = CameraConfig.MountTolerance.MEASURED;
    Arrays.fill(turnVariance, square(measured.turnStdDevRadians()));
    Arrays.fill(shiftVariance, shiftVariance(config, measured));
  }

  /** How unsure a camera's shift is along each axis: its mount's offset, and its solves' beyond. */
  static double shiftVariance(EstimatorConfig config, CameraConfig.MountTolerance mount) {
    return square(mount.offsetStdDevMeters()) + square(config.cameraShiftStdDevMeters());
  }

  /** Gives a camera its place, and starts its mount as unsure as its tolerance says. */
  void mount(String name, CameraConfig.MountTolerance tolerance) {
    int camera = places.cameraPlace(name);
    if (!ConsideredErrors.hasPlace(camera)) {
      return;
    }
    turnVariance[camera] = square(tolerance.turnStdDevRadians());
    shiftVariance[camera] = shiftVariance(config, tolerance);
    filter.setCameraMount(
        camera, turnVariance[camera], turnVariance[camera] / MATCH_SECONDS, shiftVariance[camera]);
  }

  /**
   * Hands an accepted measurement's own heading, if it has one, to the watch for a knocked mount.
   * If the camera's headings now stand clear of where the estimate has them, its turn is to be
   * opened up by its mount's tolerance and by how far its headings have turned.
   */
  void checkForKnock(int camera, Innovation innovation) {
    double headingsTurnedBy =
        knocks.headingMissed(
            camera, innovation.headingRadians(), filter.cameraHeadingVariance(camera));
    if (headingsTurnedBy > 0) {
      reopenLater(camera, turnVariance[camera] + headingsTurnedBy);
    }
  }

  /**
   * Notes that a camera's turn is to be opened up by a variance, once this update's replay ends.
   */
  private void reopenLater(int camera, double variance) {
    if (ConsideredErrors.hasPlace(camera)) {
      turnsToReopen[camera] = Math.max(turnsToReopen[camera], variance);
    }
  }

  /**
   * Opens up the turn of each camera this update found a sign of a knock for, at the newest step.
   * The history keeps that it did, so a replay from before opens it again at the same step. What
   * the watch on the cameras judged of the camera's mount is forgotten with it.
   */
  void reopenKnockedTurns() {
    boolean anyReopened = false;
    for (int camera = 0; camera < ConsideredErrors.MAX_CAMERAS; camera++) {
      double variance = turnsToReopen[camera];
      if (variance > 0) {
        filter.reopenCameraTurn(camera, variance);
        history.reopenedTurnAtNewestStep(camera, variance);
        corrections.forgetMount(camera);
        turnsToReopen[camera] = 0;
        anyReopened = true;
      }
    }
    if (anyReopened) {
      history.store(history.newestStep(), filter);
    }
  }

  /** Opens up again, in a replay, the turns that were opened up as this step ended. */
  void reopenTurnsAt(int step) {
    for (int camera = 0; camera < ConsideredErrors.MAX_CAMERAS; camera++) {
      double variance = history.turnReopenedBy(step, camera);
      if (variance > 0) {
        filter.reopenCameraTurn(camera, variance);
      }
    }
  }

  /**
   * A camera's doubt about its mount ({@link MountCheck}), for the next update to apply. Only
   * {@link HeadingCorrection#HEADING_MEASUREMENTS_AND_MOUNT_DOUBT} takes it; the vision pipeline
   * widens each image by it either way.
   */
  void doubt(String camera, double doubtMeters, double distanceMeters) {
    boolean considered = heading == HeadingCorrection.HEADING_MEASUREMENTS_AND_MOUNT_DOUBT;
    if (considered) {
      doubts.put(camera, new MountDoubt(doubtMeters, distanceMeters));
    }
  }

  /**
   * Sets each doubted camera's turn and shift variances from its doubt, at the newest step, and
   * keeps them there for a replay to start from. A replay from before restores the variances as
   * they were then; each update sets them again.
   *
   * <p>A doubt of some meters adds its square to the camera's shift variance. Seen from as far as
   * its tags were, it is also a turn of the doubt over that distance, and adds that turn's square
   * to the camera's turn variance. Until the distance has been judged, it adds no turn.
   */
  void applyDoubts() {
    if (doubts.isEmpty()) {
      return;
    }
    for (Map.Entry<String, MountDoubt> told : doubts.entrySet()) {
      int camera = places.cameraPlace(told.getKey());
      if (!ConsideredErrors.hasPlace(camera)) {
        continue;
      }
      MountDoubt doubt = told.getValue();
      double meters = doubt.meters();
      double turnRadians = doubt.tagDistanceMeters() > 0 ? meters / doubt.tagDistanceMeters() : 0;
      filter.setCameraVariances(
          camera,
          turnVariance[camera] + turnRadians * turnRadians,
          shiftVariance[camera] + meters * meters);
    }
    doubts.clear();
    history.store(history.newestStep(), filter);
  }

  private static double square(double value) {
    return value * value;
  }
}
