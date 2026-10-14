package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.localization.MeasurementLayout.HEADING_WITH_HEADING;
import static com.michaelgrundvig.frc.localization.MeasurementLayout.X_WITH_X;
import static com.michaelgrundvig.frc.localization.MeasurementLayout.X_WITH_Y;
import static com.michaelgrundvig.frc.localization.MeasurementLayout.Y_WITH_X;
import static com.michaelgrundvig.frc.localization.MeasurementLayout.Y_WITH_Y;
import static com.michaelgrundvig.frc.localization.MeasurementLayout.between;
import static com.michaelgrundvig.frc.localization.StateLayout.CELLS;
import static com.michaelgrundvig.frc.localization.StateLayout.GYRO_DRIFT;
import static com.michaelgrundvig.frc.localization.StateLayout.GYRO_SCALE;
import static com.michaelgrundvig.frc.localization.StateLayout.HEADING;
import static com.michaelgrundvig.frc.localization.StateLayout.POSE_ROWS;
import static com.michaelgrundvig.frc.localization.StateLayout.POSITION_ROWS;
import static com.michaelgrundvig.frc.localization.StateLayout.STATES;
import static com.michaelgrundvig.frc.localization.StateLayout.X;
import static com.michaelgrundvig.frc.localization.StateLayout.Y;
import static com.michaelgrundvig.frc.localization.StateLayout.cell;
import static com.michaelgrundvig.frc.localization.StateLayout.isACameraTurn;
import static com.michaelgrundvig.frc.localization.StateLayout.turnOf;

import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.util.MathUtil;

/**
 * The working of one camera measurement against a {@link PoseFilter}: both halves of its Kalman
 * update. {@link #compare} works out the measurement's innovation and its gain. The estimator gates
 * the innovation, and if it passes, the filter moves its state by {@link #stateCorrection} and
 * {@link #correctUncertainty} corrects the uncertainty.
 *
 * <p>It works directly on the filter's own covariance P and cross-covariance C, which the filter
 * hands it when it is made: the arrays are the filter's, shared, not copies. The filter's class
 * comment says what they hold and which errors are considered and why.
 *
 * <p>It holds one measurement's working at a time, in fields that are refilled for each
 * measurement, so that applying one allocates nothing.
 */
final class CameraCorrection {
  private final int consideredCount;

  /** How many tags' placements have columns of their own. */
  private final int tagCount;

  private final boolean positionFixesCorrectHeading;

  /** Whether a measurement that corrects the heading corrects the gyro's errors with it. */
  private final boolean estimatesGyroErrors;

  /** What a camera's turn and shift on its mount are taken to be for a camera without a place. */
  private final double usualCameraTurnVariance;

  private final double usualCameraShiftVariance;
  private final double headingsShiftVariance;
  private final double tagPlacementVariance;

  // The filter's own arrays, shared with it.

  /** P, the state's covariance: 5 by 5, row by row. */
  private final double[] covariance;

  /** C, the state's cross-covariance with the considered errors: 5 rows of consideredCount. */
  private final double[] crossCovariance;

  /** Each camera's shift's variance, by the camera's place. */
  private final double[] cameraShiftVariance;

  /** How unsure each camera's turn was before its images, by the camera's place. */
  private final double[] turnPriorVariance;

  /**
   * How many rows the measurement has: {@link StateLayout#POSITION_ROWS} or {@link
   * StateLayout#POSE_ROWS}.
   */
  private int rowCount;

  /** The measurement less what the estimate predicts for it: x, y in meters, heading in radians. */
  private final double[] innovation = new double[POSE_ROWS];

  // The measurement with its camera's turn, as the estimate has it, taken off ({@link
  // #takeOffTheTurn}): its position, the lever from it to its tags, and the camera's own lever to
  // them, on the field, in meters.
  private double turnedBackX;
  private double turnedBackY;
  private double leverX;
  private double leverY;
  private double cameraLeverX;
  private double cameraLeverY;

  /**
   * How many cameras' turns are learned together, places 0 up: what they share is carried, never
   * corrected ({@link #carryWhatTheTurnsShare}).
   */
  private int cameraCount = ConsideredErrors.MAX_CAMERAS;

  /** How many values of the state are in use: the first five, and each camera's turn. */
  private int activeStates = STATES;

  /**
   * H, the measurement's Jacobian with respect to the state: one row for each row of the
   * measurement, one column for each value of the state. No measurement sees the gyro's errors
   * themselves, so their columns are zero: a measurement tells of them only through how the
   * heading's error has come to depend on them. It sees its own camera's turn exactly as it sees
   * the heading, and no other camera's.
   */
  private final double[] stateSeenBy = new double[CELLS];

  /**
   * The values of the state the measurement sees at all, the columns of H that are not zero: x, y,
   * the heading, and its camera's turn if it has a place. Every product with H runs over these
   * alone, since the rest add nothing.
   */
  private final int[] seenStates = new int[4];

  private int seenCount;

  /** The considered errors the measurement sees, with how much each of its rows sees of each. */
  private final SharedErrors shared = new SharedErrors();

  /**
   * The covariance of the state's errors with the measurement's: P H^T, plus what reaches the
   * measurement through the considered errors. a row for each value of the state, a column for each
   * row of the measurement.
   */
  private final double[] stateWithMeasurement = new double[CELLS];

  /**
   * S, the innovation's covariance: rowCount by rowCount, in the top left of a 3 by 3 ({@link
   * MeasurementLayout}).
   */
  private final double[] innovationCovariance = new double[MeasurementLayout.CELLS];

  private final double[] innovationCovarianceInverse = new double[MeasurementLayout.CELLS];

  /** K, the gain: a row for each value of the state, a column for each row of the measurement. */
  private final double[] gain = new double[CELLS];

  /** K S, used once in the covariance's update. */
  private final double[] gainTimesInnovationCovariance = new double[CELLS];

  /**
   * The covariance of the measurement's errors with each considered error: a row for each row of
   * the measurement, of consideredCount columns.
   */
  private final double[] measurementWithConsidered;

  /** How many camera innovations the filter has worked out: the latest one's stamp. */
  private long innovations;

  /**
   * @param sizes how unsure the filter is of the errors it considers
   * @param tags how many tags' placements have columns of their own
   * @param positionFixesCorrectHeading whether a position fix also corrects the heading
   * @param estimatesGyroErrors whether a measurement that corrects the heading corrects the gyro's
   *     scale and drift too
   * @param covariance the filter's P, which this corrects in place
   * @param crossCovariance the filter's C, which this corrects in place
   * @param cameraShiftVariance the filter's variance of each camera's shift, which this only reads
   * @param turnPriorVariance how unsure each camera's turn was before its images, which this only
   *     reads
   */
  CameraCorrection(
      PoseFilter.Considered sizes,
      int tags,
      boolean positionFixesCorrectHeading,
      boolean estimatesGyroErrors,
      double[] covariance,
      double[] crossCovariance,
      double[] cameraShiftVariance,
      double[] turnPriorVariance) {
    tagCount = tags;
    consideredCount = ConsideredErrors.count(tags);
    this.positionFixesCorrectHeading = positionFixesCorrectHeading;
    this.estimatesGyroErrors = estimatesGyroErrors;
    usualCameraTurnVariance = square(sizes.cameraTurnStdDevRadians());
    usualCameraShiftVariance = square(sizes.cameraShiftStdDevMeters());
    headingsShiftVariance = square(sizes.cameraShiftStdDevRadians());
    tagPlacementVariance = square(sizes.tagPlacementStdDevMeters());
    this.covariance = covariance;
    this.crossCovariance = crossCovariance;
    this.cameraShiftVariance = cameraShiftVariance;
    this.turnPriorVariance = turnPriorVariance;
    measurementWithConsidered = new double[POSE_ROWS * consideredCount];
  }

  /**
   * Works out a camera measurement's innovation against the estimate, and everything the second
   * half needs to apply it: the first half of a Kalman update, up to the gain.
   *
   * @param x the filter's x now, in meters
   * @param y and its y
   * @param headingThen the field heading when the image was taken: the yaw then plus the filter's
   *     heading offset
   * @param camera the camera's place, or {@link ConsideredErrors#NO_PLACE}
   * @param cameraTurnRadians the camera's turn on its mount as the estimate has it, in radians:
   *     zero for a camera without a place
   * @param tagPlaces each of its tags' place, or {@link ConsideredErrors#NO_PLACE}
   * @param knownMissX how far the camera is known to put this measurement off along the field's x,
   *     in meters, which is taken off its position
   * @param knownMissY and along y
   * @param doubtVariance how unsure what was taken off is, as a variance along each axis, in square
   *     meters: more noise for this measurement's position, which makes it count for less. The
   *     gate's test is made without it, so a camera in doubt is refused whatever it would be
   *     refused for undoubted
   * @return the innovation, or null when the measurement cannot be compared with the estimate at
   *     all: its uncertainty and the estimate's together leave no spread to measure a distance in
   */
  PoseFilter.@Nullable Innovation compare(
      VisionMeasurement measurement,
      double x,
      double y,
      double headingThen,
      int camera,
      double cameraTurnRadians,
      int[] tagPlaces,
      double knownMissX,
      double knownMissY,
      double doubtVariance) {
    PoseCovariance noise = measurement.covariance();
    rowCount = noise.hasHeading() ? POSE_ROWS : POSITION_ROWS;
    takeOffTheTurn(measurement, cameraTurnRadians);
    describeWhatItSees(measurement, headingThen, camera, tagPlaces);
    subtractThePrediction(measurement, x, y, headingThen, cameraTurnRadians);
    innovation[X] -= knownMissX;
    innovation[Y] -= knownMissY;
    covarianceOfStateWithMeasurement();
    covarianceOfInnovation(noise);

    // What the gate tests: the innovation's squared distance from zero in S, for the position alone
    // and for the whole measurement.
    double positionChiSquared = positionChiSquared();
    if (Double.isNaN(positionChiSquared) || !invertInnovationCovariance()) {
      return null;
    }
    double chiSquared = positionChiSquared;
    if (rowCount == POSE_ROWS) {
      chiSquared = Matrices.quadratic3(innovationCovarianceInverse, innovation);
    }
    if (doubtVariance > 0) {
      // The doubt is more noise in the measurement's position, new in each image, so it adds to S
      // on the position's own cells. It comes after the gate's test and before the gain: a
      // doubted measurement counts for less, and passes nothing an undoubted one would fail.
      innovationCovariance[X_WITH_X] += doubtVariance;
      innovationCovariance[Y_WITH_Y] += doubtVariance;
      if (!invertInnovationCovariance()) {
        return null;
      }
    }
    workOutGain();
    return new PoseFilter.Innovation(
        innovation[X],
        innovation[Y],
        rowCount == POSE_ROWS,
        innovation[HEADING],
        chiSquared,
        positionChiSquared,
        ++innovations);
  }

  /**
   * The position's own chi-squared: d^T S^-1 d for the 2 by 2 position block of S, with the inverse
   * of a 2 by 2 written out: [a b; b c]^-1 = [c -b; -b a] / (a c - b²).
   *
   * @return NaN where the block leaves no spread to measure a distance in
   */
  private double positionChiSquared() {
    double sxx = innovationCovariance[X_WITH_X];
    double sxy = innovationCovariance[X_WITH_Y];
    double syy = innovationCovariance[Y_WITH_Y];
    double determinant = sxx * syy - sxy * sxy;
    // Written so a determinant that is not a number counts as degenerate too.
    boolean positionHasSpread = determinant > 0;
    if (!positionHasSpread) {
      return Double.NaN;
    }
    return (syy * innovation[X] * innovation[X]
            - 2 * sxy * innovation[X] * innovation[Y]
            + sxx * innovation[Y] * innovation[Y])
        / determinant;
  }

  /**
   * Fills in the inverse of S, over as many rows as the measurement has: all three, or the
   * position's two, by the same 2 by 2 inverse written out.
   *
   * @return false where S has no inverse
   */
  private boolean invertInnovationCovariance() {
    if (rowCount == POSE_ROWS) {
      return Matrices.invert3(innovationCovariance, innovationCovarianceInverse);
    }
    double sxx = innovationCovariance[X_WITH_X];
    double sxy = innovationCovariance[X_WITH_Y];
    double syy = innovationCovariance[Y_WITH_Y];
    double determinant = sxx * syy - sxy * sxy;
    // Written so a determinant that is not a number counts as no inverse too.
    boolean invertible = determinant > 0;
    if (!invertible) {
      return false;
    }
    innovationCovarianceInverse[X_WITH_X] = syy / determinant;
    innovationCovarianceInverse[X_WITH_Y] = -sxy / determinant;
    innovationCovarianceInverse[Y_WITH_X] = -sxy / determinant;
    innovationCovarianceInverse[Y_WITH_Y] = sxx / determinant;
    return true;
  }

  /**
   * Takes the camera's turn on its mount, as the estimate has it, off the measurement's position.
   *
   * <p>A position solved with the gyro's heading came from turning the camera's view of its tags
   * onto the field. A camera turned on its mount by an angle saw its tags turned by that angle, so
   * the place it gives itself is swung about the tags the other way by that angle, and the robot's
   * with it. Swinging the camera back about the tags, by the turn the estimate has for it, gives
   * the position the camera would have given sitting where its configuration says. With c the tags'
   * middle, Lc the camera's lever to them and R(t) the turn by t:
   *
   * <pre>
   *   camera's lever, turned back    Lc' = R(t) Lc
   *   position, turned back          position + Lc - Lc'
   *   robot's lever, turned back     the robot's lever + Lc' - Lc
   * </pre>
   *
   * <p>It is swung exactly, not in the small-angle form the rest of the update uses, so that a
   * camera turned several degrees, once its turn is known, gives positions as good as any. A
   * position that found its own heading has no lever, and is left as it is.
   */
  private void takeOffTheTurn(VisionMeasurement measurement, double turnRadians) {
    VisionMeasurement.Lever lever = measurement.lever();
    double solvedCameraLeverX = lever.cameraToTags().getX();
    double solvedCameraLeverY = lever.cameraToTags().getY();
    double cos = Math.cos(turnRadians);
    double sin = Math.sin(turnRadians);
    cameraLeverX = cos * solvedCameraLeverX - sin * solvedCameraLeverY;
    cameraLeverY = sin * solvedCameraLeverX + cos * solvedCameraLeverY;
    double movedX = solvedCameraLeverX - cameraLeverX;
    double movedY = solvedCameraLeverY - cameraLeverY;
    turnedBackX = measurement.pose().getX() + movedX;
    turnedBackY = measurement.pose().getY() + movedY;
    leverX = lever.robotToTags().getX() - movedX;
    leverY = lever.robotToTags().getY() - movedY;
  }

  /**
   * Fills in H, what each row of the measurement sees of the state, and the considered errors the
   * measurement shares with other measurements.
   *
   * <p>Each position row sees its own axis, and sees the heading through the tags' lever: a heading
   * off by dθ moves a position solved with it by dθ times the robot-to-tags vector, turned a
   * quarter turn. The camera's turn on its mount moves it the same way, through the camera's own
   * lever to the tags. With the robot's lever (lx, ly) and the camera's (cx, cy), the rows' entries
   * for x, y, the heading and the camera's turn are, every other entry zero,
   *
   * <pre>
   *   1  0  -ly  -cy
   *   0  1   lx   cx
   *   0  0   1    1     (the heading row, used only by a measurement with a heading of its own)
   * </pre>
   *
   * <p>So no one image can tell the heading from its camera's turn, but for the few centimeters
   * between the camera and the robot's center. What tells them apart is everything else: the gyro
   * carrying the heading from one camera's images to another's, and the wheels carrying the
   * position, which a heading error swings as the robot drives and a camera's turn does not.
   */
  private void describeWhatItSees(
      VisionMeasurement measurement, double headingThen, int camera, int[] tagPlaces) {
    java.util.Arrays.fill(stateSeenBy, 0);
    seenStates[0] = X;
    seenStates[1] = Y;
    seenStates[2] = HEADING;
    seenCount = 3;
    stateSeenBy[cell(X, X)] = 1;
    stateSeenBy[cell(X, HEADING)] = -leverY;
    stateSeenBy[cell(Y, Y)] = 1;
    stateSeenBy[cell(Y, HEADING)] = leverX;
    stateSeenBy[cell(HEADING, HEADING)] = 1;

    // A camera with a place has a turn of its own in the state, and shares its shifts with its
    // other images, at their own variances. One without a place has the usual variances and no
    // columns, and its turn is taken as new in each image.
    double newTurnVariance = usualCameraTurnVariance;
    int shiftForwardColumn = ConsideredErrors.NO_COLUMN;
    int shiftLeftColumn = ConsideredErrors.NO_COLUMN;
    int headingsShiftColumn = ConsideredErrors.NO_COLUMN;
    double shiftVariance = usualCameraShiftVariance;
    if (ConsideredErrors.hasPlace(camera)) {
      int turn = turnOf(camera);
      stateSeenBy[cell(X, turn)] = -cameraLeverY;
      stateSeenBy[cell(Y, turn)] = cameraLeverX;
      stateSeenBy[cell(HEADING, turn)] = 1;
      seenStates[seenCount++] = turn;
      newTurnVariance = 0;
      shiftForwardColumn = ConsideredErrors.cameraShiftForward(camera);
      shiftLeftColumn = ConsideredErrors.cameraShiftLeft(camera);
      headingsShiftColumn = ConsideredErrors.cameraHeadingsShift(camera);
      shiftVariance = cameraShiftVariance[camera];
    }
    shared.clear();
    // A turn new in each image moves the position as a heading error does, and the heading by the
    // turn itself.
    shared.add(ConsideredErrors.NO_COLUMN, newTurnVariance, -cameraLeverY, cameraLeverX, 1);
    double cos = Math.cos(headingThen);
    double sin = Math.sin(headingThen);
    // Its shift on its mount moves the position along the robot's forward, and along its left, as
    // the robot was turned on the field.
    shared.add(shiftForwardColumn, shiftVariance, cos, sin, 0);
    shared.add(shiftLeftColumn, shiftVariance, -sin, cos, 0);
    // Its headings' shift moves only its heading.
    shared.add(headingsShiftColumn, headingsShiftVariance, 0, 0, 1);
    // A tag that is off the layout moves the position the other way, by the tag's share of it.
    List<Double> shares = measurement.tagShares();
    for (int index = 0; index < shares.size(); index++) {
      double share = shares.get(index);
      int alongXColumn = ConsideredErrors.NO_COLUMN;
      int alongYColumn = ConsideredErrors.NO_COLUMN;
      if (index < tagPlaces.length && isOneOfTheFilterTags(tagPlaces[index])) {
        alongXColumn = ConsideredErrors.tagAlongX(tagPlaces[index]);
        alongYColumn = ConsideredErrors.tagAlongY(tagPlaces[index]);
      }
      shared.add(alongXColumn, tagPlacementVariance, -share, 0, 0);
      shared.add(alongYColumn, tagPlacementVariance, 0, -share, 0);
    }
  }

  private boolean isOneOfTheFilterTags(int tagPlace) {
    return ConsideredErrors.hasPlace(tagPlace) && tagPlace < tagCount;
  }

  /**
   * Fills in the innovation: the measurement, its camera's turn taken off, less what the estimate
   * predicts for it.
   *
   * <p>The position was solved with the heading its lever carries, and the estimate's heading for
   * that moment may have moved since (a measurement applied late, in the history). What the
   * estimate predicts for such a position is its own position turned about the tags by the
   * difference between the two headings, which is that difference times the lever, turned a quarter
   * turn. A heading of the measurement's own is its camera's heading less the camera's turn.
   */
  private void subtractThePrediction(
      VisionMeasurement measurement, double x, double y, double headingThen, double turnRadians) {
    double solvedHeadingOff =
        MathUtil.angleModulus(measurement.lever().heading().getRadians() - headingThen);
    innovation[X] = turnedBackX - x - solvedHeadingOff * leverY;
    innovation[Y] = turnedBackY - y + solvedHeadingOff * leverX;
    if (rowCount == POSE_ROWS) {
      innovation[HEADING] =
          MathUtil.angleModulus(
              measurement.pose().getRotation().getRadians() - turnRadians - headingThen);
    } else {
      innovation[HEADING] = Double.NaN;
    }
  }

  /**
   * Fills in the covariance of the state's errors with the measurement's. In a plain Kalman filter
   * this is P H^T. Here the measurement also sees considered errors the state depends on, so each
   * adds the state's dependence on it (its column of C) times how much the measurement sees of it.
   * An error without a column is one the state does not depend on.
   */
  private void covarianceOfStateWithMeasurement() {
    for (int state = 0; state < activeStates; state++) {
      for (int row = 0; row < rowCount; row++) {
        double sum = 0;
        for (int index = 0; index < seenCount; index++) {
          int seen = seenStates[index];
          sum += covariance[cell(state, seen)] * stateSeenBy[cell(row, seen)];
        }
        for (int error = 0; error < shared.count(); error++) {
          int column = shared.column(error);
          if (ConsideredErrors.hasColumn(column)) {
            sum += crossCovariance[cross(state, column)] * shared.seenBy(row, error);
          }
        }
        stateWithMeasurement[cell(state, row)] = sum;
      }
    }
  }

  /**
   * Fills in S, the innovation's covariance: how far apart the measurement and the estimate's
   * prediction of it are expected to be. In a plain Kalman filter this is H P H^T + R. Here each
   * shared error adds two more things: its own variance, through how much two rows see of it, and
   * its covariance with the state (its column of C), through what the other row sees of the state.
   * An error without a column is unrelated to the state and adds only its own variance.
   *
   * <p>Each cell is S[rowA][rowB]. The first term, H times the covariance of the state with the
   * measurement, already holds H P H^T and one of the two cross terms. The loop adds the other
   * cross term and the errors' own variances.
   */
  private void covarianceOfInnovation(PoseCovariance noise) {
    for (int rowA = 0; rowA < rowCount; rowA++) {
      for (int rowB = 0; rowB < rowCount; rowB++) {
        double sum = 0;
        for (int index = 0; index < seenCount; index++) {
          int state = seenStates[index];
          sum += stateSeenBy[cell(rowA, state)] * stateWithMeasurement[cell(state, rowB)];
        }
        for (int error = 0; error < shared.count(); error++) {
          double seenByRowA = shared.seenBy(rowA, error);
          if (seenByRowA != 0) {
            double errorWithRowB = shared.variance(error) * shared.seenBy(rowB, error);
            int column = shared.column(error);
            if (ConsideredErrors.hasColumn(column)) {
              for (int index = 0; index < seenCount; index++) {
                int state = seenStates[index];
                errorWithRowB +=
                    crossCovariance[cross(state, column)] * stateSeenBy[cell(rowB, state)];
              }
            }
            sum += seenByRowA * errorWithRowB;
          }
        }
        innovationCovariance[between(rowA, rowB)] = sum;
      }
    }
    // R, the measurement's own noise.
    innovationCovariance[X_WITH_X] += noise.xx();
    innovationCovariance[X_WITH_Y] += noise.xy();
    innovationCovariance[Y_WITH_X] += noise.xy();
    innovationCovariance[Y_WITH_Y] += noise.yy();
    if (rowCount == POSE_ROWS) {
      innovationCovariance[HEADING_WITH_HEADING] += noise.heading();
    }
    makeInnovationCovarianceSymmetric();
  }

  /**
   * Fills in the gain, K = C S^-1, where C is the covariance of the state with the measurement.
   *
   * <p>Some values of the state are left alone, their gain set to zero:
   *
   * <ul>
   *   <li>A measurement without a heading of its own leaves the heading alone unless the filter was
   *       made to let every fix correct it, and the cameras' turns with it: no fix of either kind
   *       turns anything then.
   *   <li>The gyro's errors are corrected only by a measurement that corrects the heading, since
   *       the heading is all they show themselves in, and only if the filter was made to estimate
   *       them.
   *   <li>The cameras' turns are corrected only relative to one another: what they share is carried
   *       ({@link #carryWhatTheTurnsShare}).
   * </ul>
   *
   * <p>A zeroed gain is not the gain a Kalman filter would choose, which is why {@link
   * #correctUncertainty} updates the covariance in the form that is right for any gain.
   */
  private void workOutGain() {
    boolean leavesHeadingAlone = rowCount == POSITION_ROWS && !positionFixesCorrectHeading;
    boolean leavesGyroErrorsAlone = leavesHeadingAlone || !estimatesGyroErrors;
    boolean leavesTurnsAlone = leavesHeadingAlone;
    for (int state = 0; state < activeStates; state++) {
      boolean isAGyroError = state == GYRO_SCALE || state == GYRO_DRIFT;
      boolean leftAlone =
          (state == HEADING && leavesHeadingAlone)
              || (isAGyroError && leavesGyroErrorsAlone)
              || (isACameraTurn(state) && leavesTurnsAlone);
      for (int row = 0; row < rowCount; row++) {
        double sum = 0;
        for (int k = 0; k < rowCount; k++) {
          sum +=
              stateWithMeasurement[cell(state, k)] * innovationCovarianceInverse[between(k, row)];
        }
        gain[cell(state, row)] = leftAlone ? 0 : sum;
      }
    }
    carryWhatTheTurnsShare();
  }

  /** From now on, the cameras with places 0 up to this many are learned together. */
  void cameraCount(int count) {
    cameraCount = Math.clamp(count, 0, ConsideredErrors.MAX_CAMERAS);
    activeStates = StateLayout.FIRST_CAMERA_TURN + cameraCount;
  }

  /**
   * Leaves what the cameras' turns share where it is: the gain for each camera's turn gives up the
   * gain's weighted average over the cameras, so that the corrections it makes to their turns
   * average to nothing.
   *
   * <p>No camera image can tell a turn every camera shares from a heading error, since each image
   * sees its camera's turn exactly as it sees the heading. The gyro tells the cameras' turns apart
   * from one another, carrying the heading from one camera's images to the next's, so their
   * differences are learned. What they share, only the wheels could say, by the way the robot
   * travels as against where the cameras put it, and a wheel zeroed a few degrees off or a worn
   * tread puts the wheels' travel off that way, steadily, which would be learned as a turn of every
   * camera and a heading as wrong. So what they share is carried, as a considered error is: as
   * unsure as the cameras' mounts allow together, never corrected. One camera alone has only that,
   * and its turn is never corrected.
   *
   * <p>What they share is their average weighted by how sure each was before its images, one over
   * that variance: what the mounts' tolerances say of it. A camera knocked has its turn opened up,
   * and counts for less in it, so its new turn is learned as its own and does not turn the heading.
   *
   * <p>A gain chosen this way is not the Kalman gain, which is why {@link #correctUncertainty}
   * updates the covariance in the form that is right for any gain.
   */
  private void carryWhatTheTurnsShare() {
    double allTheWeight = 0;
    for (int camera = 0; camera < cameraCount; camera++) {
      allTheWeight += 1 / turnPriorVariance[camera];
    }
    // Written so a weight that is not a number, or none at all, leaves every gain alone.
    boolean anyWeight = allTheWeight > 0 && Double.isFinite(allTheWeight);
    if (!anyWeight) {
      return;
    }
    for (int row = 0; row < rowCount; row++) {
      double shared = 0;
      for (int camera = 0; camera < cameraCount; camera++) {
        shared += gain[cell(turnOf(camera), row)] / turnPriorVariance[camera];
      }
      shared /= allTheWeight;
      for (int camera = 0; camera < cameraCount; camera++) {
        gain[cell(turnOf(camera), row)] -= shared;
      }
    }
  }

  /** Refuses an innovation that is not the one whose working is still held here. */
  void requireLatest(PoseFilter.Innovation applied) {
    if (applied.stamp() != innovations) {
      throw new IllegalStateException(
          "a camera's innovation is applied right after it's worked out: "
              + (innovations - applied.stamp())
              + " more came between");
    }
  }

  /**
   * The second half of the Kalman update, for the uncertainty: corrects the covariance of what the
   * measurement may correct, while only keeping account of the errors the filter considers (a
   * Schmidt-Kalman update). The filter applies the state's own correction ({@link
   * #stateCorrection}).
   */
  void correctUncertainty() {
    // The covariance: P = P - K C^T - C K^T + K S K^T, with C the covariance of the state with the
    // measurement. This form is right for any gain, so also for one that leaves the heading alone.
    // (For the full Kalman gain K = C S^-1 the last two terms cancel, leaving the textbook
    // P - K C^T.)
    for (int state = 0; state < activeStates; state++) {
      for (int row = 0; row < rowCount; row++) {
        double sum = 0;
        for (int k = 0; k < rowCount; k++) {
          sum += gain[cell(state, k)] * innovationCovariance[between(k, row)];
        }
        gainTimesInnovationCovariance[cell(state, row)] = sum;
      }
    }
    for (int stateA = 0; stateA < activeStates; stateA++) {
      for (int stateB = 0; stateB < activeStates; stateB++) {
        double change = 0;
        for (int row = 0; row < rowCount; row++) {
          change +=
              -gain[cell(stateA, row)] * stateWithMeasurement[cell(stateB, row)]
                  - stateWithMeasurement[cell(stateA, row)] * gain[cell(stateB, row)]
                  + gainTimesInnovationCovariance[cell(stateA, row)] * gain[cell(stateB, row)];
        }
        covariance[cell(stateA, stateB)] += change;
      }
    }
    makeCovarianceSymmetric();

    // The dependence on the considered errors: C = C - K M, where M is the covariance of the
    // measurement's errors with each considered error. The measurement relates to an error through
    // the state (H times that error's column of C) and, for an error it sees itself, through the
    // error's own variance times how much each row sees of it.
    for (int row = 0; row < rowCount; row++) {
      for (int column = 0; column < consideredCount; column++) {
        double throughTheState = 0;
        for (int index = 0; index < seenCount; index++) {
          int state = seenStates[index];
          throughTheState += crossCovariance[cross(state, column)] * stateSeenBy[cell(row, state)];
        }
        measurementWithConsidered[cross(row, column)] = throughTheState;
      }
      for (int error = 0; error < shared.count(); error++) {
        int column = shared.column(error);
        if (ConsideredErrors.hasColumn(column)) {
          measurementWithConsidered[cross(row, column)] +=
              shared.variance(error) * shared.seenBy(row, error);
        }
      }
    }
    for (int state = 0; state < activeStates; state++) {
      for (int column = 0; column < consideredCount; column++) {
        double change = 0;
        for (int row = 0; row < rowCount; row++) {
          change += gain[cell(state, row)] * measurementWithConsidered[cross(row, column)];
        }
        crossCovariance[cross(state, column)] -= change;
      }
    }
  }

  /**
   * How far the measurement moves one value of the state: x = x + K d, with d the innovation, so
   * that value's row of the gain times the innovation.
   */
  double stateCorrection(int state) {
    double correction = 0;
    for (int row = 0; row < rowCount; row++) {
      correction += gain[cell(state, row)] * innovation[row];
    }
    return correction;
  }

  /** Where a cell of a matrix with one column for each considered error sits in its array. */
  private int cross(int row, int column) {
    return row * consideredCount + column;
  }

  /** Sets each pair of mirrored cells of P to their mean, so that rounding cannot part them. */
  private void makeCovarianceSymmetric() {
    for (int stateA = 0; stateA < activeStates; stateA++) {
      for (int stateB = stateA + 1; stateB < activeStates; stateB++) {
        double mean = (covariance[cell(stateA, stateB)] + covariance[cell(stateB, stateA)]) / 2;
        covariance[cell(stateA, stateB)] = mean;
        covariance[cell(stateB, stateA)] = mean;
      }
    }
  }

  private void makeInnovationCovarianceSymmetric() {
    for (int rowA = 0; rowA < rowCount; rowA++) {
      for (int rowB = rowA + 1; rowB < rowCount; rowB++) {
        double mean =
            (innovationCovariance[between(rowA, rowB)] + innovationCovariance[between(rowB, rowA)])
                / 2;
        innovationCovariance[between(rowA, rowB)] = mean;
        innovationCovariance[between(rowB, rowA)] = mean;
      }
    }
  }

  private static double square(double value) {
    return value * value;
  }
}
