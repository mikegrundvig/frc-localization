package com.michaelgrundvig.frc.localization;

import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Where the robot is, worked out from its wheels, its IMUs and the cameras' measurements. This is
 * the part of localization a team can replace: the robot program talks to a localizer only through
 * this interface, so one written to it drives the robot as the two here do ({@link
 * DefaultLocalizer}, {@link WpilibLocalizer}).
 *
 * <p>What stays in front of every localizer is the solving of camera images. The cameras' tag
 * detections are solved and filtered into {@link VisionMeasurement}s by the {@link VisionPipeline},
 * and a localizer is handed those, each with its capture time and its uncertainty. It never sees an
 * image.
 *
 * <h2>How the robot calls it</h2>
 *
 * <p>Every call is made on the robot loop's thread. Each loop makes them in this order:
 *
 * <ol>
 *   <li>{@link #odometry}, once, with every wheel reading since the last loop and both IMUs.
 *   <li>{@link #place} or {@link #resetHeading}, only in a loop where the drive team placed the
 *       robot or the driver reset its heading.
 *   <li>{@link #mountDoubt}, once for each camera.
 *   <li>{@link #measurements}, once, with the loop's camera measurements. The list is empty in a
 *       loop that has none.
 * </ol>
 *
 * <p>{@link #pose}, {@link #poseAt}, {@link #timestampNanos} and the options are asked at any point
 * between those calls, some of them many times a loop.
 *
 * <h2>The contract</h2>
 *
 * <p>What every localizer keeps, clause by clause. The conformance suite ({@code
 * LocalizerConformance}, in the test fixtures) checks each clause against both localizers here, and
 * a localizer of your own is checked by giving the suite a bench for it ({@code LocalizerBench}).
 * The clauses say what a localizer must do to be driven by. None of them says how accurate it must
 * be.
 *
 * <p><b>Frames, units and time</b>
 *
 * <ul>
 *   <li><b>L1.</b> Every pose is the robot's center on the field, in the field's frame as {@code
 *       com.michaelgrundvig.frc.world.field} has it: meters, and a heading counterclockwise in
 *       radians.
 *   <li><b>L2.</b> Every time is in nanoseconds on the robot's clock. A localizer reads no clock of
 *       its own: the loop's time comes in the {@link LoopContext}, and every reading and
 *       measurement carries the moment it was taken. Given the same calls, it gives the same
 *       answers, so a replay of a logged match reproduces every pose.
 * </ul>
 *
 * <p><b>The pose</b>
 *
 * <ul>
 *   <li><b>L3.</b> {@link #pose} always answers, with numbers, from the moment the localizer is
 *       made. Before anything is known it is a placeholder, and the localizer chooses which.
 *   <li><b>L4.</b> {@link #timestampNanos} is the time of the newest wheel reading taken in, and
 *       zero before the first. It never runs backwards.
 *   <li><b>L5.</b> {@link #poseAt} is where the robot was at a moment, as the localizer has it now.
 *       It answers for every moment from half a second before the newest wheel reading up to that
 *       reading, once the localizer has run that long since the robot was last placed, and at the
 *       newest reading it is {@link #pose}. Outside what it keeps it is empty, never a guess.
 * </ul>
 *
 * <p><b>Odometry</b>
 *
 * <ul>
 *   <li><b>L6.</b> With odometry alone the pose moves as the wheels and the gyro say, and only so:
 *       a robot that drives a meter forward ends a meter from where it began, and one that stands
 *       still stays put.
 *   <li><b>L7.</b> The first readings ever taken move nothing: wheels that have already rolled and
 *       a gyro that isn't at zero are where the robot starts, not a distance it drove.
 *   <li><b>L8.</b> A loop with no wheel readings, a wheel whose reading isn't a number, and an IMU
 *       that isn't usable ({@link ImuReadings#usable}) are all ordinary. The pose stays a number,
 *       and the localizer carries on with what it has: the backup IMU, or the wheels' own turning.
 * </ul>
 *
 * <p><b>Placing the robot</b>
 *
 * <ul>
 *   <li><b>L9.</b> After {@link #place}, {@link #pose} is the placed pose, and odometry carries on
 *       from there. Where the placement says nothing of the heading (an infinite heading variance),
 *       the heading stays as it was.
 *   <li><b>L10.</b> After {@link #resetHeading}, the pose faces the given heading and stands where
 *       it stood.
 * </ul>
 *
 * <p><b>Measurements</b>
 *
 * <ul>
 *   <li><b>L11.</b> A measurement that agrees with the estimate to within its own uncertainty is
 *       used: given the same pose loop after loop, the estimate closes on it. A localizer may
 *       refuse one that disagrees, for as long as it judges right.
 *   <li><b>L12.</b> A measurement never moves the estimate away from itself, or past itself.
 *   <li><b>L13.</b> A measurement is applied at its capture time, not when it arrives: it corrects
 *       where the robot was then, and the robot's motion since stays. One measurement of a robot
 *       driving straight leaves it no farther back along its path than it was.
 *   <li><b>L14.</b> A measurement whose heading variance is infinite says nothing about heading,
 *       and does not turn the estimate.
 *   <li><b>L15.</b> A loop's measurements come camera by camera, as the cameras were read, not in
 *       the order they were taken. A localizer that needs them oldest first sorts them.
 *   <li><b>L16.</b> One measurement's bad data costs that measurement, never the loop. One taken
 *       before the localizer started, or older than it keeps, is dropped. One stamped after the
 *       newest wheel reading is dropped or taken as of now. Nothing is thrown.
 * </ul>
 *
 * <p><b>Cost</b>
 *
 * <ul>
 *   <li><b>L17.</b> What a loop costs does not grow with how long the robot has run: what the
 *       localizer keeps is bounded, in memory and in time.
 * </ul>
 *
 * <p><b>The options</b>
 *
 * <p>Each option is one method that answers with a value, or with nothing. A localizer that has
 * none of them is a whole localizer. The robot program copes without each one, and says "not
 * reported" where it would have shown it, so never return a made-up value to fill one in.
 *
 * <ul>
 *   <li><b>L18.</b> An option is reported in every loop from the first {@link #odometry}, or never.
 *   <li><b>L19.</b> {@link #covariance} is how sure the pose is. It is honest: it grows while only
 *       odometry comes in, and the pose's real error stays within what it claims.
 *   <li><b>L20.</b> {@link #mode} is what the pose rests on, and {@link #health} the numbers behind
 *       it. A pose that rests on the cameras says {@link LocalizationEstimate.Mode#TRACKING} only
 *       while a measurement was used lately.
 *   <li><b>L21.</b> {@link #decisions} has one decision for each measurement the last {@link
 *       #measurements} was given, each naming its measurement.
 *   <li><b>L22.</b> {@link #fieldVelocities} is how the robot moves over the field, in the field's
 *       frame: zero for a robot that stands still.
 *   <li><b>L23.</b> {@link #wheels}, {@link #trust} and {@link #yawSource} are as of the last
 *       {@link #odometry} or {@link #measurements}, whichever each says.
 * </ul>
 *
 * <h2>Writing one</h2>
 *
 * <p>Implement the six methods that have no default. Override an option only where the localizer
 * really knows the answer. Then give the conformance suite a bench for it, and choose it for a
 * robot in the robot's configuration ({@code RobotConfig}'s {@code localizer}), with a {@link
 * LocalizerMaker}.
 */
public interface Localizer {
  // ---- What it is told: required ----

  /**
   * One loop of the drivetrain's readings: every wheel reading since the last loop, oldest first,
   * what both IMUs read, and how the wheels say the robot moves. The first call of every loop.
   *
   * @param readings what the drivetrain read over the loop
   * @param loop the loop's time on the robot's clock, and whether the robot is enabled
   */
  void odometry(DrivetrainReadings readings, LoopContext loop);

  /**
   * The robot is here: the drive team placed it on its start, or a driver said so. Called after the
   * loop's {@link #odometry}, so a turn made in placing the robot is not counted twice.
   *
   * @param placed where, when, and how sure the teller is. A localizer that keeps no uncertainty
   *     takes the pose and ignores how sure it is.
   */
  void place(PoseReset placed);

  /**
   * The loop's camera measurements, each a pose with its capture time and its uncertainty. The last
   * call of every loop, with an empty list in a loop that has none. The localizer applies them
   * before it returns, or refuses them.
   *
   * @param taken the measurements, in the order the cameras were read
   */
  void measurements(List<VisionMeasurement> taken);

  // ---- What it says: required ----

  /** Where the robot is now. It is asked many times a loop, so it should be cheap. */
  Pose2d pose();

  /**
   * Where the robot was at a moment, as the localizer has it now. Each camera image is solved with
   * the heading the robot had when the image was taken, which is asked here.
   *
   * @param timestampNanos the moment, on the robot's clock
   * @return the pose then, or empty outside what the localizer keeps
   */
  Optional<Pose2d> poseAt(long timestampNanos);

  /** The time of the newest wheel reading the pose includes, and zero before the first. */
  long timestampNanos();

  // ---- What it is told: each with a default that suits a localizer that needs nothing more ----

  /**
   * The robot faces this heading, as sure as the given spread, and stands where it stood: the
   * driver's "facing straight downfield" when the heading has gone wrong.
   *
   * <p>By default this places the robot where the localizer has it, with the new heading. Override
   * it where the localizer can do better with a heading alone.
   *
   * @param heading the heading the robot has now
   * @param stdDevRadians how sure that is: how straight a driver lines a robot up by eye
   */
  default void resetHeading(Rotation2d heading, double stdDevRadians) {
    PoseCovariance position = covariance().orElse(PoseCovariance.fromStdDevs(0, 0, 0));
    PoseCovariance spread =
        new PoseCovariance(
            position.xx(), position.xy(), position.yy(), stdDevRadians * stdDevRadians);
    Pose2d here = new Pose2d(pose().getTranslation(), heading);
    place(new PoseReset(timestampNanos(), here, spread, "heading reset"));
  }

  /**
   * How far one camera's mount may have moved since it was calibrated, as the robot's check of it
   * judges now ({@link MountCheck}). Every measurement from that camera already comes widened by
   * this, so a localizer needs to do nothing with it, and by default it does nothing.
   *
   * @param camera the camera's name
   * @param doubtMeters how far its measurements may be off
   * @param distanceMeters how far its tags were, on average, when that was judged; NaN until then
   */
  default void mountDoubt(String camera, double doubtMeters, double distanceMeters) {}

  /**
   * The heading to solve an image taken at a moment with, or empty when the image should be solved
   * without one. By default it is the heading of {@link #poseAt}. Override it to answer empty while
   * the localizer would not stake an image on its heading: the default localizer does after a
   * driver's heading reset, until a camera has backed the new heading.
   *
   * <p>A heading is leaned on only while {@link #covariance} says it is sure. A localizer that
   * reports no covariance has every image solved without its heading.
   */
  default Optional<Rotation2d> headingAt(long timestampNanos) {
    return poseAt(timestampNanos).map(Pose2d::getRotation);
  }

  // ---- The options: each empty unless the localizer knows the answer ----

  /**
   * How sure the pose is. Without it, the pit's page says the uncertainty is not reported, the
   * World view draws no ellipse, no camera's mount is checked or calibrated against the pose, and
   * no image is solved with the pose's heading.
   */
  default Optional<PoseCovariance> covariance() {
    return Optional.empty();
  }

  /**
   * What the pose rests on: tags seen lately, odometry alone, or nothing yet. Without it, the pit's
   * page does not say, and no camera's mount is checked or calibrated against the pose.
   */
  default Optional<LocalizationEstimate.Mode> mode() {
    return Optional.empty();
  }

  /**
   * The numbers behind the mode: how old the newest measurement used is, how well the cameras and
   * odometry agree, how much of the wheels' travel is in doubt. Without it they are not logged.
   * Without it, {@link #mode} or {@link #covariance}, the robot's readiness to score is not judged.
   */
  default Optional<LocalizationEstimate.Health> health() {
    return Optional.empty();
  }

  /**
   * What became of each measurement the last {@link #measurements} was given. Without it, the log,
   * the pit's page and the World view say the measurements were given to the localizer, and not
   * what it did with them, and no camera's mount is checked against the pose.
   */
  default Optional<List<VisionDecision>> decisions() {
    return Optional.empty();
  }

  /**
   * How the robot moves over the field, as of the last {@link #odometry}: {@code vx} and {@code vy}
   * in meters a second along the field's axes, {@code omega} in radians a second counterclockwise.
   * Without it, the robot's speed is taken from the wheels and its turning from the IMU.
   */
  default Optional<ChassisVelocities> fieldVelocities() {
    return Optional.empty();
  }

  /**
   * How far the wheels are trusted, as of the last {@link #odometry}: the robot's tilt and how much
   * of it the ground explains, a collision felt, and how fast the wheels may be sliding. Without
   * it, none of those are logged, and nothing warns of an IMU mounted another way than the robot's
   * definition says.
   */
  default Optional<WheelTrust> wheels() {
    return Optional.empty();
  }

  /**
   * How far each source is trusted, as of the last {@link #measurements}: each camera, the wheels,
   * and the tags the localizer has no place for. Without it, nothing warns of cameras or tags the
   * localizer cannot keep apart.
   */
  default Optional<Trust> trust() {
    return Optional.empty();
  }

  /**
   * What turns odometry, as of the last {@link #odometry}: the gyro, the backup IMU, or the wheels.
   * Without it, it is not logged.
   */
  default Optional<YawSource> yawSource() {
    return Optional.empty();
  }
}
