package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.localization.StateLayout.CELLS;
import static com.michaelgrundvig.frc.localization.StateLayout.GYRO_DRIFT;
import static com.michaelgrundvig.frc.localization.StateLayout.GYRO_SCALE;
import static com.michaelgrundvig.frc.localization.StateLayout.HEADING;
import static com.michaelgrundvig.frc.localization.StateLayout.HH;
import static com.michaelgrundvig.frc.localization.StateLayout.STATES;
import static com.michaelgrundvig.frc.localization.StateLayout.X;
import static com.michaelgrundvig.frc.localization.StateLayout.XX;
import static com.michaelgrundvig.frc.localization.StateLayout.XY;
import static com.michaelgrundvig.frc.localization.StateLayout.Y;
import static com.michaelgrundvig.frc.localization.StateLayout.YX;
import static com.michaelgrundvig.frc.localization.StateLayout.YY;
import static com.michaelgrundvig.frc.localization.StateLayout.cell;

import java.util.Arrays;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The pose estimator's filter: a Kalman filter over the robot's position, a heading offset, and the
 * gyro's two errors, its scale and its drift. The field heading is the gyro's yaw plus the offset,
 * so the gyro turns the estimate and the filter only corrects what the gyro gets wrong: the share
 * of every turn its scale misreads, and what it drifts each second. It predicts through odometry
 * steps ({@link #predict}) and corrects with camera measurements ({@link #innovation} then {@link
 * #correct}). What to feed it, when, and whether to believe a measurement is the estimator's
 * business.
 *
 * <p><b>What it holds.</b>
 *
 * <ul>
 *   <li>The state: x and y in meters on the field, the heading offset in radians, the gyro's scale
 *       error, in radians of heading per radian the gyro turns, and the gyro's drift, in radians of
 *       heading per second.
 *   <li>The state's covariance, P: 5 by 5, stored row by row in the order x, y, heading, scale,
 *       drift ({@link StateLayout} names its cells).
 *   <li>The cross-covariance, C: how each value of the state has come to depend on each error the
 *       filter considers, 5 rows by {@link #consideredCount()} columns ({@link ConsideredErrors}
 *       says which column is which).
 * </ul>
 *
 * <p><b>The gyro's errors.</b> A gyro that reads 0.3% too much turn is 1.1° off after one turn and
 * 5.4° after five, the same way each time, not a little more at random. One that drifts a degree a
 * minute is five degrees off after five minutes, however it turned. So both are values of the
 * state: each radian the gyro turns adds the scale to the heading offset, each second adds the
 * drift, and the heading's error comes to depend on theirs. Any measurement that corrects the
 * heading then says something about both, by how far the heading was off after how much turning and
 * how much time, and the filter corrects all three. Once they are known, the heading stays right
 * through a turn and through a stretch without tags, with no camera needed. The filter can also be
 * made to leave them alone ({@link GyroErrors#CONSIDERED}): it then carries the scale's doubt but
 * never corrects it, and takes the drift as noise that grows the heading's doubt with time.
 *
 * <p><b>Errors it considers but never estimates.</b> Some errors repeat: in every image a camera
 * takes, or in every turn the gyro makes. Averaged as if each image's or each turn's error were
 * new, they would shrink the uncertainty to millimeters while the estimate stays centimeters off.
 * So the filter treats them as Schmidt "consider" parameters (Schmidt 1966; Zanetti and D'Souza
 * 2015). Each has a fixed variance, the filter never corrects it, and the filter keeps track of how
 * the estimate's own errors have come to depend on it.
 *
 * <ul>
 *   <li><b>The heading, for a position a camera solved with it.</b> Such a position leans on the
 *       gyro's heading: if the heading is off by an angle, the position is off by that angle times
 *       the distance to the tags, at right angles to them ({@link VisionMeasurement#lever}). So
 *       each fix measures the position and the heading's error together. By default such a fix
 *       still does not correct the heading: the gyro owns it, and only a measurement with a heading
 *       of its own corrects its drift. With {@code positionFixesCorrectHeading}, every fix corrects
 *       it, which is an ordinary full Kalman update.
 *   <li><b>The backup IMU's scale.</b> It errs as the gyro's does, by a scale of its own: three
 *       turns one way on the gyro and three back on the backup do not cancel. It turns the robot
 *       too seldom to be worth estimating.
 *   <li><b>Each camera's turn on its mount</b>, what is left after calibrating it. It moves the
 *       camera's positions just as a heading error does, and its headings by the angle itself.
 *   <li><b>Each camera's shift on its mount:</b> what is left of where it sits, along the robot's
 *       forward and left. It moves every position the camera finds by the same amount on the robot,
 *       so on the field it turns as the robot turns.
 *   <li><b>Each camera's headings' shift:</b> the rest of what its headings share from image to
 *       image, from the tags it sees being a little off the layout. Two tags a third of a meter
 *       apart, each half an inch from where the layout says, turn a multi-tag heading by up to 4°.
 *       It changes as the robot moves and turns and the camera sees other tags from elsewhere, so
 *       the filter lets go of it over a correlation distance and angle.
 *   <li><b>Each tag's placement</b> on the field, along the field and across it. A tag off the
 *       layout puts every position solved from it off by its share of the solve, in every image of
 *       every camera that sees it. Two cameras seeing the same tags share its error, and turning
 *       away and back finds it again.
 * </ul>
 *
 * <p>The considered errors' own variances stay fixed, and two considered errors stay unrelated to
 * each other, since nothing ever corrects them. So a robot standing still, its camera seeing the
 * same tags image after image, still follows any change in where it is, but cannot place itself
 * better than that camera's repeated errors and its tags' allow. Several cameras, each with errors
 * of its own, together do better, but no better than the tags they share.
 */
final class PoseFilter {
  /**
   * Below this much turn, a step is taken as straight: the arc's formulas divide by the turn, and
   * for a turn this small they give the straight line anyway.
   */
  private static final double STRAIGHT_BELOW_RADIANS = 1e-9;

  /** An uncertainty as big as the field: the estimate knows nothing. */
  private static final PoseCovariance NO_IDEA = PoseCovariance.fromStdDevs(100, 100, Math.PI);

  /**
   * How unsure the filter is of the errors it considers.
   *
   * @param gyroScaleStdDev the gyro's scale error: radians of heading per radian it turns
   * @param backupScaleStdDev the backup IMU's
   * @param cameraTurnStdDevRadians each camera's turn on its mount
   * @param cameraShiftStdDevMeters each camera's shift on its mount, forward and left
   * @param cameraShiftStdDevRadians and of its headings
   * @param tagPlacementStdDevMeters each tag's placement, along x and along y
   * @param shiftCorrelationMeters how far the robot moves before a camera's headings' shift is
   *     mostly a new one: what the filter keeps of it falls by a factor of e over this distance
   * @param shiftCorrelationRadians and over this much turning
   */
  record Considered(
      double gyroScaleStdDev,
      double backupScaleStdDev,
      double cameraTurnStdDevRadians,
      double cameraShiftStdDevMeters,
      double cameraShiftStdDevRadians,
      double tagPlacementStdDevMeters,
      double shiftCorrelationMeters,
      double shiftCorrelationRadians) {}

  /**
   * How far a camera's measurement is from the estimate, and how surprising that is.
   *
   * @param xMeters the position's innovation along the field's x: measured less predicted
   * @param yMeters and along y
   * @param withHeading whether the measurement has a heading of its own
   * @param headingRadians the heading's innovation; NaN for a measurement without a heading
   * @param chiSquared the whole innovation's squared distance from zero, measured in its own
   *     uncertainty (its squared Mahalanobis distance): what the estimator's gate tests. For a
   *     camera whose mount is in doubt, that uncertainty is taken as an undoubted camera's would
   *     be: the doubt makes the measurement count for less, and never passes one it would fail
   * @param positionChiSquared the same for the position alone
   * @param stamp which of the filter's innovations this is. {@link #correct(Innovation)} applies
   *     only the latest, because the filter holds the working of only one at a time
   */
  record Innovation(
      double xMeters,
      double yMeters,
      boolean withHeading,
      double headingRadians,
      double chiSquared,
      double positionChiSquared,
      long stamp) {
    double distanceMeters() {
      return Math.hypot(xMeters, yMeters);
    }
  }

  /** Whether camera measurements may correct the gyro's own errors, or only carry their doubt. */
  enum GyroErrors {
    /**
     * The scale's doubt is carried, at the variance the filter was told, and never corrected. A
     * steady drift is not considered at all: the heading noise each step is given stands for it.
     */
    CONSIDERED,
    /** Every measurement that corrects the heading corrects the scale and the drift with it. */
    ESTIMATED
  }

  /**
   * How unsure the filter is of the gyro's drift, and how far both of the gyro's errors may wander,
   * where it estimates them.
   *
   * @param driftStdDevRadiansPerSecond how fast the gyro may drift steadily, before any camera has
   *     said anything of it
   * @param scaleWanderPerSqrtSecond how far the scale may have wandered after one second, as a
   *     standard deviation: its variance grows with time
   * @param driftWanderPerSqrtSecond the same for the drift, in radians a second
   */
  record GyroDoubt(
      double driftStdDevRadiansPerSecond,
      double scaleWanderPerSqrtSecond,
      double driftWanderPerSqrtSecond) {
    /** A gyro whose drift is not estimated, and whose errors stay what they are. */
    static final GyroDoubt NONE = new GyroDoubt(0, 0, 0);
  }

  /** How many errors the filter considers, which is how many columns C has. */
  private final int consideredCount;

  // The considered errors' variances.
  private final double backupScaleVariance;

  /**
   * Each camera's turn's variance, and its shift's along each axis, by the camera's place. They
   * start as the filter was told, and a doubt about a camera's mount raises them ({@link
   * #setCameraVariances}).
   */
  private final double[] cameraTurnVariance = new double[ConsideredErrors.MAX_CAMERAS];

  private final double[] cameraShiftVariance = new double[ConsideredErrors.MAX_CAMERAS];

  private final double shiftCorrelationMeters;
  private final double shiftCorrelationRadians;

  // How fast each of the gyro's errors may wander: the variance each second adds to it.
  private final double scaleVariancePerSecond;
  private final double driftVariancePerSecond;

  // The state.
  private double x;
  private double y;
  private double headingOffset;

  /**
   * The gyro's scale error: how many radians each radian the gyro turns adds to the heading offset.
   * A gyro that reads 3% too much turn has a scale error of about -0.03.
   */
  private double gyroScale;

  /**
   * The gyro's drift: how many radians each second adds to the heading offset. A gyro that drifts
   * 10° a minute counterclockwise has a drift of -10° a minute here.
   */
  private double gyroDrift;

  /** P, the state's covariance: 5 by 5, row by row. */
  private final double[] covariance = new double[CELLS];

  /** C, the state's cross-covariance with the considered errors: 5 rows of consideredCount. */
  private final double[] crossCovariance;

  /** The working of camera measurements against this filter's P and C, which it shares. */
  private final CameraCorrection correction;

  /**
   * A filter in which a position fix leaves the heading to measurements with a heading of their
   * own.
   *
   * @param tags how many tags' placement to keep apart: the field's, each given a place from 0
   */
  PoseFilter(Considered sizes, int tags) {
    this(sizes, tags, false);
  }

  /**
   * @param tags how many tags' placement to keep apart: the field's, each given a place from 0
   * @param positionFixesCorrectHeading whether a position fix also corrects the heading, through
   *     its lever and how the position's error has come to relate to the heading's: a full Kalman
   *     update. False leaves the heading to measurements with a heading of their own: the gyro owns
   *     it.
   */
  PoseFilter(Considered sizes, int tags, boolean positionFixesCorrectHeading) {
    this(sizes, tags, positionFixesCorrectHeading, GyroErrors.CONSIDERED, GyroDoubt.NONE);
  }

  /**
   * @param tags how many tags' placement to keep apart: the field's, each given a place from 0
   * @param positionFixesCorrectHeading whether a position fix also corrects the heading
   * @param gyroErrors whether measurements that correct the heading correct the gyro's scale and
   *     drift too. Either way the scale starts at zero, as unsure as {@code sizes} says.
   * @param gyroDoubt how unsure the drift starts, and how far both may wander. It counts only where
   *     the gyro's errors are estimated.
   */
  PoseFilter(
      Considered sizes,
      int tags,
      boolean positionFixesCorrectHeading,
      GyroErrors gyroErrors,
      GyroDoubt gyroDoubt) {
    if (tags < 0) {
      throw new IllegalArgumentException("how many tags to keep apart: " + tags);
    }
    consideredCount = ConsideredErrors.count(tags);
    crossCovariance = new double[STATES * consideredCount];
    boolean estimated = gyroErrors == GyroErrors.ESTIMATED;
    covariance[cell(GYRO_SCALE, GYRO_SCALE)] = square(sizes.gyroScaleStdDev());
    covariance[cell(GYRO_DRIFT, GYRO_DRIFT)] =
        estimated ? square(gyroDoubt.driftStdDevRadiansPerSecond()) : 0;
    scaleVariancePerSecond = estimated ? square(gyroDoubt.scaleWanderPerSqrtSecond()) : 0;
    driftVariancePerSecond = estimated ? square(gyroDoubt.driftWanderPerSqrtSecond()) : 0;
    backupScaleVariance = square(sizes.backupScaleStdDev());
    shiftCorrelationMeters = sizes.shiftCorrelationMeters();
    shiftCorrelationRadians = sizes.shiftCorrelationRadians();
    Arrays.fill(cameraTurnVariance, square(sizes.cameraTurnStdDevRadians()));
    Arrays.fill(cameraShiftVariance, square(sizes.cameraShiftStdDevMeters()));
    correction =
        new CameraCorrection(
            sizes,
            tags,
            positionFixesCorrectHeading,
            estimated,
            covariance,
            crossCovariance,
            cameraTurnVariance,
            cameraShiftVariance);
  }

  double x() {
    return x;
  }

  double y() {
    return y;
  }

  /** The heading offset, in radians: the field heading is the yaw plus this. */
  double headingOffset() {
    return headingOffset;
  }

  /** The gyro's scale error: radians of heading for each radian the gyro turns. */
  double gyroScale() {
    return gyroScale;
  }

  /** The gyro's drift: radians of heading for each second. */
  double gyroDrift() {
    return gyroDrift;
  }

  /** The estimated pose, given the yaw now. */
  Pose2d pose(double yawRadians) {
    return new Pose2d(x, y, new Rotation2d(yawRadians + headingOffset));
  }

  // ---------------------------------------------------------------------------------------------
  // Being told where the robot is.
  // ---------------------------------------------------------------------------------------------

  /** Takes a reset's position and covariance, and its heading if it gives one. */
  void set(PoseReset reset, double yawRadians) {
    x = reset.pose().getX();
    y = reset.pose().getY();
    PoseCovariance given = reset.covariance();
    if (given.hasHeading()) {
      headingOffset = reset.pose().getRotation().getRadians() - yawRadians;
      setAfresh(HEADING, given.heading());
    }
    setPositionCovariance(given.xx(), given.xy(), given.yy());
  }

  /** A new heading, as sure as given. The position stays where it is, unrelated to the heading. */
  void resetHeading(double newOffsetRadians, double variance) {
    headingOffset = newOffsetRadians;
    setAfresh(HEADING, variance);
  }

  /** Moves the position, leaving it unrelated to the heading and as sure as given each way. */
  void shift(double dxMeters, double dyMeters, double variance) {
    x += dxMeters;
    y += dyMeters;
    setPositionCovariance(variance, 0, variance);
  }

  /** Turns the heading, with a fresh uncertainty for it and for the position. */
  void turn(double radians, double positionVariance, double headingVariance) {
    headingOffset += radians;
    setAfresh(HEADING, headingVariance);
    setPositionCovariance(positionVariance, 0, positionVariance);
  }

  /**
   * Sets one camera's turn's and shift's variances, from now on: raised by a doubt about its mount,
   * or brought back down.
   *
   * <p>A variance that falls takes the estimate's dependence on that error down with it, in
   * proportion. Otherwise the estimate would depend on the error more than an error that small
   * allows, and the covariance of the estimate and the error together would no longer be one any
   * real errors could have (it would not be positive semidefinite).
   *
   * @return whether either variance changed
   */
  boolean setCameraVariances(int camera, double turnVariance, double shiftVariance) {
    boolean changes =
        turnVariance != cameraTurnVariance[camera] || shiftVariance != cameraShiftVariance[camera];
    if (!changes) {
      return false;
    }
    shrinkDependence(ConsideredErrors.cameraTurn(camera), turnVariance, cameraTurnVariance[camera]);
    shrinkDependence(
        ConsideredErrors.cameraShiftForward(camera), shiftVariance, cameraShiftVariance[camera]);
    shrinkDependence(
        ConsideredErrors.cameraShiftLeft(camera), shiftVariance, cameraShiftVariance[camera]);
    cameraTurnVariance[camera] = turnVariance;
    cameraShiftVariance[camera] = shiftVariance;
    return true;
  }

  /**
   * Scales the state's dependence on one considered error by the square root of how far its
   * variance fell, which is how far its standard deviation fell. A variance that rose or stayed
   * changes nothing.
   */
  private void shrinkDependence(int column, double newVariance, double oldVariance) {
    boolean fell = newVariance < oldVariance;
    if (!fell) {
      return;
    }
    double factor = oldVariance > 0 ? Math.sqrt(newVariance / oldVariance) : 0;
    for (int state = 0; state < STATES; state++) {
      crossCovariance[cross(state, column)] *= factor;
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Predicting through odometry.
  // ---------------------------------------------------------------------------------------------

  /**
   * The Kalman filter's prediction, through part of an odometry step: moves the position by the
   * wheels' travel along the arc the gyro turned through, and grows the uncertainty by what the
   * move adds.
   *
   * @param startYaw the gyro's yaw where this part of the step starts, in radians
   * @param turn how far the gyro turned over it
   * @param forward the wheels' travel forward over it, in the robot's frame at its start, in meters
   * @param left and to the left
   * @param seconds how long this part of the step lasted
   * @param positionNoise the variance to add to each position axis
   * @param headingNoise the variance to add to the heading
   * @param turnedBy what turned the robot. An IMU's scale error grows the heading's error by the
   *     turn, and the gyro's drift by the time. The wheels' own turning errors are already in the
   *     heading noise.
   */
  void predict(
      double startYaw,
      double turn,
      double forward,
      double left,
      double seconds,
      double positionNoise,
      double headingNoise,
      YawSource turnedBy) {
    // What the gyro got wrong over this part of the step, as the estimate has it: the share of the
    // turn its scale misread, and what it drifted in the time. The robot really turned by the
    // gyro's turn and that, and the arc below is drawn through that turn.
    double gyroError = turnedBy == YawSource.GYRO ? gyroScale * turn + gyroDrift * seconds : 0;
    double trueTurn = turn + gyroError;
    // A robot that turns evenly while it drives traces an arc. The straight line from the arc's
    // start to its end, in the robot's frame at the start, is the travel times these two factors:
    // sin(turn)/turn along the way it was heading, (1 - cos(turn))/turn across it. (This is the
    // exact integral, the same as WPILib's Twist2d.exp.)
    double along = forward;
    double across = left;
    boolean turned = Math.abs(trueTurn) > STRAIGHT_BELOW_RADIANS;
    if (turned) {
      double sineOverTurn = Math.sin(trueTurn) / trueTurn;
      double oneMinusCosineOverTurn = (1 - Math.cos(trueTurn)) / trueTurn;
      along = sineOverTurn * forward - oneMinusCosineOverTurn * left;
      across = oneMinusCosineOverTurn * forward + sineOverTurn * left;
    }
    double heading = startYaw + headingOffset;
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);
    double moveX = cos * along - sin * across;
    double moveY = sin * along + cos * across;
    x += moveX;
    y += moveY;
    headingOffset += gyroError;

    // A heading offset that is a little off swings the move about its start: an offset off by dθ
    // puts x off by -moveY dθ and y off by moveX dθ.
    errorGrowsBy(X, HEADING, -moveY);
    errorGrowsBy(Y, HEADING, moveX);
    switch (turnedBy) {
      case GYRO -> {
        // The gyro's scale error adds the turn times itself to the heading, and its drift the
        // time times itself.
        errorGrowsBy(HEADING, GYRO_SCALE, turn);
        errorGrowsBy(HEADING, GYRO_DRIFT, seconds);
        covariance[cell(GYRO_SCALE, GYRO_SCALE)] += scaleVariancePerSecond * seconds;
        covariance[cell(GYRO_DRIFT, GYRO_DRIFT)] += driftVariancePerSecond * seconds;
      }
      case BACKUP -> addScaleError(turn, ConsideredErrors.BACKUP_SCALE, backupScaleVariance);
      case WHEELS -> {}
    }
    letGoOfHeadingsShifts(Math.hypot(moveX, moveY), Math.abs(turn));

    covariance[XX] += positionNoise;
    covariance[YY] += positionNoise;
    covariance[HH] += headingNoise;
  }

  /**
   * The covariance's prediction, P = F P F^T, for one way an error feeds another: the error of one
   * value of the state grows by a factor times the error of another. F, the Jacobian of the new
   * state with respect to the old, is then the identity with that factor in the first value's row
   * and the other's column. A heading error swinging a move into x is one such F, and the gyro's
   * scale error turning into heading is another.
   *
   * <p>F P adds the factor times the other's row to the first value's row, and then (F P) F^T adds
   * the factor times the other's column to the first value's column. Between them the first value's
   * own variance gains twice the factor times the two's covariance, plus the factor squared times
   * the other's variance. C, the dependence on the considered errors, is predicted by the same F: C
   * = F C adds the factor times the other's row to the first value's row.
   *
   * @param grows the value of the state whose error grows
   * @param from the value whose error feeds it, which is another one
   * @param factor how much of that error is added
   */
  private void errorGrowsBy(int grows, int from, double factor) {
    for (int column = 0; column < STATES; column++) {
      covariance[cell(grows, column)] += factor * covariance[cell(from, column)];
    }
    for (int row = 0; row < STATES; row++) {
      covariance[cell(row, grows)] += factor * covariance[cell(row, from)];
    }
    for (int column = 0; column < consideredCount; column++) {
      crossCovariance[cross(grows, column)] += factor * crossCovariance[cross(from, column)];
    }
  }

  /**
   * The backup IMU's scale error, which is considered and never estimated. An IMU that reads its
   * turns a little large or small adds its scale error times the turn to the heading's error. It is
   * the same error every turn, so the heading comes to depend on it: with s the scale error and v
   * its variance, the heading's error grows by turn times s, which adds
   *
   * <pre>
   *   to the heading's variance                    2 turn cov(heading, s) + turn² v
   *   to cov(any other value, heading)             turn cov(that value, s)
   *   to cov(heading, s)                           turn v
   * </pre>
   *
   * <p>A scale with no variance is not considered at all.
   */
  private void addScaleError(double turn, int scaleColumn, double scaleVariance) {
    boolean considered = scaleVariance > 0;
    if (!considered) {
      return;
    }
    for (int state = 0; state < STATES; state++) {
      if (state == HEADING) {
        continue;
      }
      covariance[cell(state, HEADING)] += turn * crossCovariance[cross(state, scaleColumn)];
      covariance[cell(HEADING, state)] = covariance[cell(state, HEADING)];
    }
    covariance[HH] +=
        2 * turn * crossCovariance[cross(HEADING, scaleColumn)] + turn * turn * scaleVariance;
    crossCovariance[cross(HEADING, scaleColumn)] += turn * scaleVariance;
  }

  /**
   * As the robot moves and turns, each camera sees other tags, or the same ones from elsewhere, and
   * its headings' shift becomes a different one. So the state's dependence on each camera's
   * headings' shift decays, by a factor of e for each correlation distance moved and each
   * correlation angle turned. A camera's shift on its mount and each tag's placement stay what they
   * are, and their columns are left alone.
   */
  private void letGoOfHeadingsShifts(double movedMeters, double turnedRadians) {
    double kept =
        Math.exp(-movedMeters / shiftCorrelationMeters - turnedRadians / shiftCorrelationRadians);
    if (kept < 1) {
      for (int camera = 0; camera < ConsideredErrors.MAX_CAMERAS; camera++) {
        for (int state = 0; state < STATES; state++) {
          crossCovariance[cross(state, ConsideredErrors.cameraHeadingsShift(camera))] *= kept;
        }
      }
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Correcting with a camera measurement.
  // ---------------------------------------------------------------------------------------------

  /**
   * Works out a camera measurement's innovation against the estimate, and everything {@link
   * #correct(Innovation)} needs to apply it: the first half of a Kalman update, up to the gain. The
   * estimator gates the innovation between the two halves.
   *
   * @param yawThen the gyro's yaw when the image was taken
   * @param camera the camera's place, from 0 to {@link ConsideredErrors#MAX_CAMERAS} - 1: its turn
   *     and shifts are the ones it shares with its other images. {@link ConsideredErrors#NO_PLACE}
   *     for a camera without a place, whose turn and shifts are taken as new in each image
   * @param tagPlaces each of its tags' place, in the order of its {@link VisionMeasurement#tagIds}:
   *     the placement it shares with every image of that tag. {@link ConsideredErrors#NO_PLACE} for
   *     a tag without a place, whose placement is taken as new in each image
   * @return the innovation, or null when the measurement cannot be compared with the estimate at
   *     all: its uncertainty and the estimate's together leave no spread to measure a distance in
   */
  @Nullable Innovation innovation(
      VisionMeasurement measurement, double yawThen, int camera, int[] tagPlaces) {
    return innovation(measurement, yawThen, camera, tagPlaces, 0, 0);
  }

  /**
   * The same, for a measurement whose camera is known to put its positions off: the known miss is
   * taken off the measurement's position before it is compared.
   *
   * @param knownMissX how far the camera is known to put this measurement off along the field's x,
   *     in meters ({@link CameraWatch})
   * @param knownMissY and along y
   */
  @Nullable Innovation innovation(
      VisionMeasurement measurement,
      double yawThen,
      int camera,
      int[] tagPlaces,
      double knownMissX,
      double knownMissY) {
    double headingThen = yawThen + headingOffset;
    return correction.compare(
        measurement, x, y, headingThen, camera, tagPlaces, knownMissX, knownMissY);
  }

  /**
   * Applies the camera measurement last given to {@link #innovation}: the second half of the Kalman
   * update. It moves the state by the gain times the innovation, and corrects the uncertainty. The
   * working of both halves is {@link CameraCorrection}'s.
   */
  void correct(Innovation applied) {
    correction.requireLatest(applied);
    x += correction.stateCorrection(X);
    y += correction.stateCorrection(Y);
    headingOffset += correction.stateCorrection(HEADING);
    gyroScale += correction.stateCorrection(GYRO_SCALE);
    gyroDrift += correction.stateCorrection(GYRO_DRIFT);
    correction.correctUncertainty();
  }

  // ---------------------------------------------------------------------------------------------
  // Reporting and storing the uncertainty.
  // ---------------------------------------------------------------------------------------------

  /**
   * The covariance, as the estimate reports it: the position's block and the heading's variance,
   * with rounding that would make it impossible (a variance a hair below zero, x and y varying
   * together a hair more than their variances allow) taken out.
   */
  PoseCovariance covariance() {
    if (!Double.isFinite(covariance[XX])
        || !Double.isFinite(covariance[XY])
        || !Double.isFinite(covariance[YY])
        || !Double.isFinite(covariance[HH])) {
      // Only a bug gets here, since every input is checked on the way in. Say the estimate knows
      // nothing, rather than throw in the robot loop.
      return NO_IDEA;
    }
    double xx = Math.max(0, covariance[XX]);
    double yy = Math.max(0, covariance[YY]);
    double mostXyCanBe = Math.sqrt(xx * yy);
    double xy = Math.clamp(covariance[XY], -mostXyCanBe, mostXyCanBe);
    return new PoseCovariance(xx, xy, yy, Math.max(0, covariance[HH]));
  }

  /**
   * How sure the gyro's scale is, as a standard deviation: radians of heading per radian turned.
   */
  double gyroScaleStdDev() {
    return Math.sqrt(Math.max(0, covariance[cell(GYRO_SCALE, GYRO_SCALE)]));
  }

  /** How sure the gyro's drift is, as a standard deviation, in radians a second. */
  double gyroDriftStdDev() {
    return Math.sqrt(Math.max(0, covariance[cell(GYRO_DRIFT, GYRO_DRIFT)]));
  }

  /** How many errors it considers: the backup IMU's scale, each camera's four, each tag's two. */
  int consideredCount() {
    return consideredCount;
  }

  /**
   * How many numbers its uncertainty takes to store: P's cells, then C's five rows, then each
   * camera's turn variance, then each camera's shift variance. {@link #copyUncertaintyTo} writes
   * them in that order and {@link #restore} reads them back.
   */
  int uncertaintyLength() {
    return CELLS + crossCovariance.length + 2 * ConsideredErrors.MAX_CAMERAS;
  }

  /** Copies the uncertainty, {@link #uncertaintyLength()} numbers, for the history to keep. */
  void copyUncertaintyTo(double[] into, int at) {
    System.arraycopy(covariance, 0, into, at, CELLS);
    int crossAt = at + CELLS;
    System.arraycopy(crossCovariance, 0, into, crossAt, crossCovariance.length);
    int turnVariancesAt = crossAt + crossCovariance.length;
    System.arraycopy(cameraTurnVariance, 0, into, turnVariancesAt, ConsideredErrors.MAX_CAMERAS);
    int shiftVariancesAt = turnVariancesAt + ConsideredErrors.MAX_CAMERAS;
    System.arraycopy(cameraShiftVariance, 0, into, shiftVariancesAt, ConsideredErrors.MAX_CAMERAS);
  }

  /**
   * Puts the filter back to a state, and to an uncertainty that {@link #copyUncertaintyTo} kept.
   */
  void restore(
      double x,
      double y,
      double headingOffset,
      double gyroScale,
      double gyroDrift,
      double[] uncertainty,
      int at) {
    this.x = x;
    this.y = y;
    this.headingOffset = headingOffset;
    this.gyroScale = gyroScale;
    this.gyroDrift = gyroDrift;
    System.arraycopy(uncertainty, at, covariance, 0, CELLS);
    int crossAt = at + CELLS;
    System.arraycopy(uncertainty, crossAt, crossCovariance, 0, crossCovariance.length);
    int turnVariancesAt = crossAt + crossCovariance.length;
    System.arraycopy(
        uncertainty, turnVariancesAt, cameraTurnVariance, 0, ConsideredErrors.MAX_CAMERAS);
    int shiftVariancesAt = turnVariancesAt + ConsideredErrors.MAX_CAMERAS;
    System.arraycopy(
        uncertainty, shiftVariancesAt, cameraShiftVariance, 0, ConsideredErrors.MAX_CAMERAS);
  }

  // ---------------------------------------------------------------------------------------------
  // The matrices' layouts.
  // ---------------------------------------------------------------------------------------------

  /**
   * Where a cell of a matrix with one column for each considered error sits in its array, stored
   * row by row: C, whose rows are the state's, and the measurement's covariance with the considered
   * errors, whose rows are the measurement's.
   */
  private int cross(int row, int column) {
    return row * consideredCount + column;
  }

  /**
   * Sets the position's own block of P, and leaves the position unrelated to everything else: it
   * was just set afresh, so nothing that came before bears on it.
   */
  private void setPositionCovariance(double xx, double xy, double yy) {
    setAfresh(X, xx);
    setAfresh(Y, yy);
    covariance[XY] = xy;
    covariance[YX] = xy;
  }

  /**
   * Gives one value of the state a variance of its own and leaves it unrelated to every other value
   * and to every considered error: it was just told afresh, so nothing that came before bears on
   * it. The gyro's errors, which are never told afresh, keep their estimates and their variances:
   * they are the gyro's, not the pose's.
   */
  private void setAfresh(int state, double variance) {
    for (int other = 0; other < STATES; other++) {
      covariance[cell(state, other)] = 0;
      covariance[cell(other, state)] = 0;
    }
    covariance[cell(state, state)] = variance;
    forgetDependenceOnConsideredErrors(state);
  }

  /** Clears one state's row of C: that state was just set afresh. */
  private void forgetDependenceOnConsideredErrors(int state) {
    Arrays.fill(crossCovariance, cross(state, 0), cross(state + 1, 0), 0);
  }

  private static double square(double value) {
    return value * value;
  }
}
