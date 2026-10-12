package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.world.config.Rules.atLeast;
import static com.michaelgrundvig.frc.world.config.Rules.below;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;

/**
 * How the pose estimator weighs odometry, the gyro, and the cameras. The defaults are reasonable
 * starting points, not measurements: fit them from logged residuals (the calibration procedures),
 * never from one run.
 *
 * @param translationStdDevPerMeter how far odometry's position may be off after one meter of
 *     travel, in meters. Its variance grows with distance.
 * @param translationStdDevPerRadian how far odometry's position may be off after turning one
 *     radian, in meters: wheels scrub as they turn
 * @param residualScale how much of the wheels' disagreement (the kinematic residual, which says how
 *     far it may have put their travel off) counts as suspect travel: 1 counts all of it
 * @param tiltStartDegrees below this tilt the wheels are trusted fully
 * @param tiltZeroDegrees at and above this tilt the wheels' travel counts for nothing: a robot
 *     climbing a bump or a ramp has wheels in the air (a top team scaled to zero at 25 degrees).
 *     Between the two, their travel counts for less, and the share left out counts as suspect.
 * @param slipCorrelationSeconds how long slip keeps its direction. Suspect travel within this time
 *     adds up before it's squared: a slip that builds one way for a quarter second grows the
 *     uncertainty as one error, not as many small ones that would average out.
 * @param collisionStdDevMeters how far a collision may have moved the robot beyond what its wheels
 *     saw: added at once, since the robot has already moved by the time the jolt is read
 * @param collisionWindowSeconds how long after a jolt the robot counts as just hit: more jolts
 *     within it are the same collision, and a scoring gate waits it out
 * @param handlingStdDevPerSqrtSecond while the robot is disabled, how far people may carry it where
 *     its wheels can't see, in meters after one second; the variance grows with time
 * @param headingDriftDegreesPerMinute how far the gyro's heading may drift in a minute
 * @param headingStdDevDegreesPerTurn how far the gyro's heading may be off after a full turn: its
 *     scale error, the same in every turn, so five turns are five times as far off (the estimator
 *     considers it, but never corrects it: the gyro-scale procedure measures it)
 * @param backupHeadingStdDevDegreesPerTurn the same for the backup IMU, while it turns odometry in
 *     the gyro's place: its own scale error, the same in every turn it makes, and unrelated to the
 *     gyro's (the gyro-scale procedure calibrates the gyro alone)
 * @param headingNoiseDegreesPerTurn how far an IMU's heading may be off at random after a full
 *     turn, beyond its scale: errors that don't repeat from turn to turn, whose variance grows with
 *     the turning
 * @param wheelHeadingStdDevDegreesPerTurn the same for heading from the wheels, used only while
 *     both IMUs are lost; its errors are taken as new with every turn
 * @param cameraTurnStdDevDegrees how far each camera may be turned from its configured mount, the
 *     same in every image: what calibrating its mount leaves, and its lens. A camera turned by an
 *     angle puts its tags, and so the robot, that angle times their distance off, as a heading
 *     error does, and its headings by the angle. However many images it takes, the estimator never
 *     trusts a camera's positions or headings more than this allows.
 * @param cameraShiftStdDevMeters how far each camera's positions may be off the same way image
 *     after image, beyond its turn and the tags': its mount's shift (what calibrating it leaves),
 *     along the robot's forward and left, so it turns with the robot. Today it also stands in for a
 *     single tag's solve coming out short from noisy corners, which hangs on the tag's range and
 *     angle and is shared by every camera seeing it face-on: a stop-gap (docs/values.md).
 * @param cameraShiftStdDevDegrees and how far its headings may, beyond its turn: tags off the
 *     layout turn a multi-tag solve's heading by how far off they are over how far apart they sit
 * @param tagPlacementStdDevMeters how far each tag may sit from where the field's layout says,
 *     along the field and across it (the field's tolerance, and how it was built): the same for
 *     every camera that sees it and every time it's seen, so the estimator keeps it apart for each
 *     tag, and a tag several cameras see counts once
 * @param headingCorrection what may correct the heading the gyro owns, beside measurements with a
 *     heading of their own; three ways under comparison (docs/testing.md), {@link
 *     HeadingCorrection#HEADING_MEASUREMENTS} until the team chooses
 * @param cameraShiftCorrelationMeters how far the robot moves before what a camera's headings share
 *     is mostly new ({@code cameraShiftStdDevDegrees}): it sees other tags, or the same ones from
 *     elsewhere
 * @param cameraShiftCorrelationDegrees and how far it turns
 * @param historySeconds how far back a late camera measurement can still be applied at its capture
 *     time
 * @param futureToleranceSeconds how far a measurement may be stamped after the newest odometry
 *     sample (clocks differ a little) before it's rejected
 * @param visionTimeoutSeconds how long without an accepted camera measurement before the estimate
 *     counts as dead reckoning
 * @param disagreementWindowSeconds how far back the health's vision-odometry disagreement looks
 * @param gateChiSquaredPosition the gate for a position measurement: its innovation's squared
 *     Mahalanobis distance may not exceed this (13.8 passes 99.9% of good measurements, 2 degrees
 *     of freedom)
 * @param gateChiSquaredPose the same for a measurement with heading (16.3: 99.9%, 3 degrees)
 * @param recoveryMeasurements how many rejected measurements must agree with each other, within the
 *     recovery window, before odometry is judged wrong and the estimate jumps to them
 * @param recoveryWindowSeconds how recent those rejected measurements must be
 * @param recoveryAgreementMeters how closely their innovations must agree
 * @param recoveryStdDevMeters the position uncertainty after a recovery
 * @param recoveryAgreementDegrees how closely rejected measurements' headings must agree before the
 *     gyro's heading is judged wrong (a bad heading reset, a robot placed backwards, a gyro that
 *     jumped) and the estimate turns to them; and how closely a camera's heading must agree with a
 *     driver's heading reset to back it
 * @param recoveryHeadingStdDevDegrees the heading uncertainty after a heading recovery
 * @param lostAfterRejections this many camera measurements rejected in a row, with none accepted
 *     between, and the estimate is lost: the cameras keep disagreeing with it
 * @param lostPositionStdDevMeters the position uncertainty after a heading recovery: the position
 *     was worked out with the wrong heading, so it could be far off
 */
@ConfigRecord
public record EstimatorConfig(
    double translationStdDevPerMeter,
    double translationStdDevPerRadian,
    double residualScale,
    double tiltStartDegrees,
    double tiltZeroDegrees,
    double slipCorrelationSeconds,
    double collisionStdDevMeters,
    double collisionWindowSeconds,
    double handlingStdDevPerSqrtSecond,
    double headingDriftDegreesPerMinute,
    double headingStdDevDegreesPerTurn,
    double backupHeadingStdDevDegreesPerTurn,
    double headingNoiseDegreesPerTurn,
    double wheelHeadingStdDevDegreesPerTurn,
    double cameraTurnStdDevDegrees,
    double cameraShiftStdDevMeters,
    double cameraShiftStdDevDegrees,
    double tagPlacementStdDevMeters,
    HeadingCorrection headingCorrection,
    double cameraShiftCorrelationMeters,
    double cameraShiftCorrelationDegrees,
    double historySeconds,
    double futureToleranceSeconds,
    double visionTimeoutSeconds,
    double disagreementWindowSeconds,
    double gateChiSquaredPosition,
    double gateChiSquaredPose,
    int recoveryMeasurements,
    double recoveryWindowSeconds,
    double recoveryAgreementMeters,
    double recoveryStdDevMeters,
    double recoveryAgreementDegrees,
    double recoveryHeadingStdDevDegrees,
    int lostAfterRejections,
    double lostPositionStdDevMeters) {
  /** Starting points for a swerve robot with a good gyro. */
  public static final EstimatorConfig DEFAULTS =
      builder()
          .translationStdDevPerMeter(0.03)
          .translationStdDevPerRadian(0.02)
          .residualScale(1.0)
          .tiltStartDegrees(5)
          .tiltZeroDegrees(25)
          .slipCorrelationSeconds(0.5)
          .collisionStdDevMeters(0.25)
          .collisionWindowSeconds(0.25)
          .handlingStdDevPerSqrtSecond(0.02)
          .headingDriftDegreesPerMinute(0.5)
          .headingStdDevDegreesPerTurn(1.0)
          .backupHeadingStdDevDegreesPerTurn(2.0)
          .headingNoiseDegreesPerTurn(0.1)
          .wheelHeadingStdDevDegreesPerTurn(10)
          .cameraTurnStdDevDegrees(0.25)
          .cameraShiftStdDevMeters(0.005)
          .cameraShiftStdDevDegrees(2.0)
          .tagPlacementStdDevMeters(0.01)
          .headingCorrection(HeadingCorrection.HEADING_MEASUREMENTS)
          .cameraShiftCorrelationMeters(1.0)
          .cameraShiftCorrelationDegrees(45)
          .historySeconds(1.5)
          .futureToleranceSeconds(0.05)
          .visionTimeoutSeconds(1.0)
          .disagreementWindowSeconds(0.5)
          .gateChiSquaredPosition(13.816)
          .gateChiSquaredPose(16.266)
          .recoveryMeasurements(3)
          .recoveryWindowSeconds(1.0)
          .recoveryAgreementMeters(0.25)
          .recoveryStdDevMeters(0.25)
          .recoveryAgreementDegrees(5)
          .recoveryHeadingStdDevDegrees(2)
          .lostAfterRejections(2)
          .lostPositionStdDevMeters(1.0)
          .build();

  public EstimatorConfig {
    Checks.on(EstimatorConfig.class)
        .field("translationStdDevPerMeter", translationStdDevPerMeter, nonNegative())
        .field("translationStdDevPerRadian", translationStdDevPerRadian, nonNegative())
        .field("residualScale", residualScale, nonNegative())
        .field("tiltStartDegrees", tiltStartDegrees, nonNegative())
        .field("tiltZeroDegrees", tiltZeroDegrees, below(90))
        .field("slipCorrelationSeconds", slipCorrelationSeconds, positive())
        .field("collisionStdDevMeters", collisionStdDevMeters, nonNegative())
        .field("collisionWindowSeconds", collisionWindowSeconds, positive())
        .field("handlingStdDevPerSqrtSecond", handlingStdDevPerSqrtSecond, nonNegative())
        .field("headingDriftDegreesPerMinute", headingDriftDegreesPerMinute, nonNegative())
        .field("headingStdDevDegreesPerTurn", headingStdDevDegreesPerTurn, nonNegative())
        .field(
            "backupHeadingStdDevDegreesPerTurn", backupHeadingStdDevDegreesPerTurn, nonNegative())
        .field("headingNoiseDegreesPerTurn", headingNoiseDegreesPerTurn, nonNegative())
        .field("wheelHeadingStdDevDegreesPerTurn", wheelHeadingStdDevDegreesPerTurn, nonNegative())
        .field("cameraTurnStdDevDegrees", cameraTurnStdDevDegrees, nonNegative())
        .field("cameraShiftStdDevMeters", cameraShiftStdDevMeters, nonNegative())
        .field("cameraShiftStdDevDegrees", cameraShiftStdDevDegrees, nonNegative())
        .field("tagPlacementStdDevMeters", tagPlacementStdDevMeters, nonNegative())
        .field("cameraShiftCorrelationMeters", cameraShiftCorrelationMeters, positive())
        .field("cameraShiftCorrelationDegrees", cameraShiftCorrelationDegrees, positive())
        .field("historySeconds", historySeconds, positive())
        .field("futureToleranceSeconds", futureToleranceSeconds, nonNegative())
        .field("visionTimeoutSeconds", visionTimeoutSeconds, positive())
        .field("disagreementWindowSeconds", disagreementWindowSeconds, positive())
        .field("gateChiSquaredPosition", gateChiSquaredPosition, positive())
        .field("gateChiSquaredPose", gateChiSquaredPose, positive())
        .field("recoveryMeasurements", recoveryMeasurements, atLeast(2))
        .field("recoveryWindowSeconds", recoveryWindowSeconds, positive())
        .field("recoveryAgreementMeters", recoveryAgreementMeters, nonNegative())
        .field("recoveryStdDevMeters", recoveryStdDevMeters, positive())
        .field("recoveryAgreementDegrees", recoveryAgreementDegrees, positive())
        .field("recoveryHeadingStdDevDegrees", recoveryHeadingStdDevDegrees, positive())
        .field("lostAfterRejections", lostAfterRejections, atLeast(1))
        .field("lostPositionStdDevMeters", lostPositionStdDevMeters, positive())
        .check(
            "tiltZeroDegrees",
            () ->
                tiltZeroDegrees > tiltStartDegrees
                    ? null
                    : shown(tiltZeroDegrees)
                        + " must be above tiltStartDegrees ("
                        + shown(tiltStartDegrees)
                        + "): wheels stop counting above where they start to count less")
        .orThrow();
  }

  /**
   * What corrects the heading beside the gyro. A camera's position solved with the gyro's heading
   * leans on it, and so says something about the heading too; whether that's let correct the
   * heading is the choice. Each is honest about its uncertainty; they differ in what they trust the
   * cameras with.
   */
  public enum HeadingCorrection {
    /**
     * Only measurements with a heading of their own (multi-tag solves, under strict conditions),
     * with all three of their rows: a position solved with the gyro's heading carries the heading's
     * doubt but never turns it, so the gyro owns heading. Honest on a calibrated robot; a camera
     * turned on its mount turns the heading with its own headings, and the mount check then doubts
     * the cameras that disagree with it.
     */
    HEADING_MEASUREMENTS,
    /**
     * As {@link #HEADING_MEASUREMENTS}, and a camera the mount check doubts has that doubt added to
     * the errors it repeats in every image (its turn and shift), so its headings can pull the
     * heading only so far, however many there are. Only as good as the mount check's verdict: with
     * a camera turned, much of the doubt lands on its neighbour too (docs/testing.md).
     */
    HEADING_MEASUREMENTS_AND_MOUNT_DOUBT,
    /**
     * Every fix: a position fix corrects the heading too, through how far its tags are and how the
     * position's error has come to depend on the heading's. A full Kalman update: more accurate in
     * heading after turns. With a camera turned on its mount, its positions turn the heading, and
     * the mount check then doubts its healthy neighbour more than it. It leans on the images'
     * capture times while the robot turns: an image stamped a few milliseconds late shows the robot
     * turned a little less, as a heading error would, the same way in every image, so check them
     * before choosing it (docs/testing.md, docs/characterization.md).
     */
    EVERY_FIX
  }

  /** Starts building estimator settings, one named value at a time. */
  public static EstimatorConfigBuilder.TranslationStdDevPerMeterStage builder() {
    return EstimatorConfigBuilder.builder();
  }
}
