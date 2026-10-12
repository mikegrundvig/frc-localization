package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.localization.StateLayout.CELLS;
import static com.michaelgrundvig.frc.localization.StateLayout.HEADING;
import static com.michaelgrundvig.frc.localization.StateLayout.HH;
import static com.michaelgrundvig.frc.localization.StateLayout.HX;
import static com.michaelgrundvig.frc.localization.StateLayout.HY;
import static com.michaelgrundvig.frc.localization.StateLayout.POSE_ROWS;
import static com.michaelgrundvig.frc.localization.StateLayout.POSITION_ROWS;
import static com.michaelgrundvig.frc.localization.StateLayout.STATES;
import static com.michaelgrundvig.frc.localization.StateLayout.X;
import static com.michaelgrundvig.frc.localization.StateLayout.XH;
import static com.michaelgrundvig.frc.localization.StateLayout.XX;
import static com.michaelgrundvig.frc.localization.StateLayout.XY;
import static com.michaelgrundvig.frc.localization.StateLayout.Y;
import static com.michaelgrundvig.frc.localization.StateLayout.YH;
import static com.michaelgrundvig.frc.localization.StateLayout.YX;
import static com.michaelgrundvig.frc.localization.StateLayout.YY;
import static com.michaelgrundvig.frc.localization.StateLayout.cell;

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

  /** What a camera's turn and shift on its mount are taken to be for a camera without a place. */
  private final double usualCameraTurnVariance;

  private final double usualCameraShiftVariance;
  private final double headingsShiftVariance;
  private final double tagPlacementVariance;

  // The filter's own arrays, shared with it.

  /** P, the state's covariance: 3 by 3, row by row. */
  private final double[] covariance;

  /** C, the state's cross-covariance with the considered errors: 3 rows of consideredCount. */
  private final double[] crossCovariance;

  /** Each camera's turn's variance, and its shift's, by the camera's place. */
  private final double[] cameraTurnVariance;

  private final double[] cameraShiftVariance;

  /**
   * How many rows the measurement has: {@link StateLayout#POSITION_ROWS} or {@link
   * StateLayout#POSE_ROWS}.
   */
  private int rowCount;

  /** The measurement less what the estimate predicts for it: x, y in meters, heading in radians. */
  private final double[] innovation = new double[POSE_ROWS];

  /**
   * H, the measurement's Jacobian with respect to the state: one row for each row of the
   * measurement, one column for each of x, y and the heading offset.
   */
  private final double[] stateSeenBy = new double[CELLS];

  /** The considered errors the measurement sees, with how much each of its rows sees of each. */
  private final SharedErrors shared = new SharedErrors();

  /**
   * The covariance of the state's errors with the measurement's: P H^T, plus what reaches the
   * measurement through the considered errors. 3 rows for the state, a column for each row of the
   * measurement.
   */
  private final double[] stateWithMeasurement = new double[CELLS];

  /** S, the innovation's covariance: rowCount by rowCount, in the top left of a 3 by 3. */
  private final double[] innovationCovariance = new double[CELLS];

  private final double[] innovationCovarianceInverse = new double[CELLS];

  /** K, the gain: 3 rows for the state, a column for each row of the measurement. */
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
   * @param covariance the filter's P, which this corrects in place
   * @param crossCovariance the filter's C, which this corrects in place
   * @param cameraTurnVariance the filter's variance of each camera's turn, which this only reads
   * @param cameraShiftVariance and of each camera's shift
   */
  CameraCorrection(
      PoseFilter.Considered sizes,
      int tags,
      boolean positionFixesCorrectHeading,
      double[] covariance,
      double[] crossCovariance,
      double[] cameraTurnVariance,
      double[] cameraShiftVariance) {
    tagCount = tags;
    consideredCount = ConsideredErrors.count(tags);
    this.positionFixesCorrectHeading = positionFixesCorrectHeading;
    usualCameraTurnVariance = square(sizes.cameraTurnStdDevRadians());
    usualCameraShiftVariance = square(sizes.cameraShiftStdDevMeters());
    headingsShiftVariance = square(sizes.cameraShiftStdDevRadians());
    tagPlacementVariance = square(sizes.tagPlacementStdDevMeters());
    this.covariance = covariance;
    this.crossCovariance = crossCovariance;
    this.cameraTurnVariance = cameraTurnVariance;
    this.cameraShiftVariance = cameraShiftVariance;
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
   * @param tagPlaces each of its tags' place, or {@link ConsideredErrors#NO_PLACE}
   * @return the innovation, or null when the measurement cannot be compared with the estimate at
   *     all: its uncertainty and the estimate's together leave no spread to measure a distance in
   */
  PoseFilter.@Nullable Innovation compare(
      VisionMeasurement measurement,
      double x,
      double y,
      double headingThen,
      int camera,
      int[] tagPlaces) {
    PoseCovariance noise = measurement.covariance();
    rowCount = noise.hasHeading() ? POSE_ROWS : POSITION_ROWS;
    describeWhatItSees(measurement, headingThen, camera, tagPlaces);
    subtractThePrediction(measurement, x, y, headingThen);
    covarianceOfStateWithMeasurement();
    covarianceOfInnovation(noise);

    // The position's own chi-squared, d^T S^-1 d for the 2 by 2 position block of S, with the
    // inverse of a 2 by 2 written out: [a b; b c]^-1 = [c -b; -b a] / (a c - b²).
    double sxx = innovationCovariance[XX];
    double sxy = innovationCovariance[XY];
    double syy = innovationCovariance[YY];
    double determinant = sxx * syy - sxy * sxy;
    // Written so a determinant that is not a number counts as degenerate too.
    boolean positionHasSpread = determinant > 0;
    if (!positionHasSpread) {
      return null;
    }
    double positionChiSquared =
        (syy * innovation[X] * innovation[X]
                - 2 * sxy * innovation[X] * innovation[Y]
                + sxx * innovation[Y] * innovation[Y])
            / determinant;
    double chiSquared = positionChiSquared;
    if (rowCount == POSE_ROWS) {
      if (!Matrices.invert3(innovationCovariance, innovationCovarianceInverse)) {
        return null;
      }
      chiSquared = Matrices.quadratic3(innovationCovarianceInverse, innovation);
    } else {
      innovationCovarianceInverse[XX] = syy / determinant;
      innovationCovarianceInverse[XY] = -sxy / determinant;
      innovationCovarianceInverse[YX] = -sxy / determinant;
      innovationCovarianceInverse[YY] = sxx / determinant;
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
   * Fills in H, what each row of the measurement sees of the state, and the considered errors the
   * measurement shares with other measurements.
   *
   * <p>Each position row sees its own axis, and sees the heading through the tags' lever: a heading
   * off by dθ moves a position solved with it by dθ times the robot-to-tags vector, turned a
   * quarter turn. With the lever (lx, ly), H is
   *
   * <pre>
   *   1  0  -ly
   *   0  1   lx
   *   0  0   1     (the heading row, used only by a measurement with a heading of its own)
   * </pre>
   */
  private void describeWhatItSees(
      VisionMeasurement measurement, double headingThen, int camera, int[] tagPlaces) {
    double leverX = measurement.lever().robotToTags().getX();
    double leverY = measurement.lever().robotToTags().getY();
    stateSeenBy[cell(X, X)] = 1;
    stateSeenBy[cell(X, Y)] = 0;
    stateSeenBy[cell(X, HEADING)] = -leverY;
    stateSeenBy[cell(Y, X)] = 0;
    stateSeenBy[cell(Y, Y)] = 1;
    stateSeenBy[cell(Y, HEADING)] = leverX;
    stateSeenBy[cell(HEADING, X)] = 0;
    stateSeenBy[cell(HEADING, Y)] = 0;
    stateSeenBy[cell(HEADING, HEADING)] = 1;

    // A camera with a place shares its errors with its other images, at their own variances. One
    // without a place has the usual variances and no columns.
    int turnColumn = ConsideredErrors.NO_COLUMN;
    int shiftForwardColumn = ConsideredErrors.NO_COLUMN;
    int shiftLeftColumn = ConsideredErrors.NO_COLUMN;
    int headingsShiftColumn = ConsideredErrors.NO_COLUMN;
    double turnVariance = usualCameraTurnVariance;
    double shiftVariance = usualCameraShiftVariance;
    if (ConsideredErrors.hasPlace(camera)) {
      turnColumn = ConsideredErrors.cameraTurn(camera);
      shiftForwardColumn = ConsideredErrors.cameraShiftForward(camera);
      shiftLeftColumn = ConsideredErrors.cameraShiftLeft(camera);
      headingsShiftColumn = ConsideredErrors.cameraHeadingsShift(camera);
      turnVariance = cameraTurnVariance[camera];
      shiftVariance = cameraShiftVariance[camera];
    }
    double cos = Math.cos(headingThen);
    double sin = Math.sin(headingThen);
    shared.clear();
    // The camera's turn on its mount moves its position as a heading error does, and its heading by
    // the turn itself.
    shared.add(turnColumn, turnVariance, -leverY, leverX, 1);
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
   * Fills in the innovation: the measurement less what the estimate predicts for it.
   *
   * <p>The position was solved with the heading its lever carries, and the estimate's heading for
   * that moment may have moved since (a measurement applied late, in the history). What the
   * estimate predicts for such a position is its own position turned about the tags by the
   * difference between the two headings, which is that difference times the lever, turned a quarter
   * turn.
   */
  private void subtractThePrediction(
      VisionMeasurement measurement, double x, double y, double headingThen) {
    double leverX = measurement.lever().robotToTags().getX();
    double leverY = measurement.lever().robotToTags().getY();
    double solvedHeadingOff =
        MathUtil.angleModulus(measurement.lever().heading().getRadians() - headingThen);
    innovation[X] = measurement.pose().getX() - x - solvedHeadingOff * leverY;
    innovation[Y] = measurement.pose().getY() - y + solvedHeadingOff * leverX;
    if (rowCount == POSE_ROWS) {
      innovation[HEADING] =
          MathUtil.angleModulus(measurement.pose().getRotation().getRadians() - headingThen);
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
    for (int state = 0; state < STATES; state++) {
      for (int row = 0; row < rowCount; row++) {
        double sum =
            covariance[cell(state, X)] * stateSeenBy[cell(row, X)]
                + covariance[cell(state, Y)] * stateSeenBy[cell(row, Y)]
                + covariance[cell(state, HEADING)] * stateSeenBy[cell(row, HEADING)];
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
        double sum =
            stateSeenBy[cell(rowA, X)] * stateWithMeasurement[cell(X, rowB)]
                + stateSeenBy[cell(rowA, Y)] * stateWithMeasurement[cell(Y, rowB)]
                + stateSeenBy[cell(rowA, HEADING)] * stateWithMeasurement[cell(HEADING, rowB)];
        for (int error = 0; error < shared.count(); error++) {
          double seenByRowA = shared.seenBy(rowA, error);
          if (seenByRowA != 0) {
            double errorWithRowB = shared.variance(error) * shared.seenBy(rowB, error);
            int column = shared.column(error);
            if (ConsideredErrors.hasColumn(column)) {
              errorWithRowB +=
                  crossCovariance[cross(X, column)] * stateSeenBy[cell(rowB, X)]
                      + crossCovariance[cross(Y, column)] * stateSeenBy[cell(rowB, Y)]
                      + crossCovariance[cross(HEADING, column)] * stateSeenBy[cell(rowB, HEADING)];
            }
            sum += seenByRowA * errorWithRowB;
          }
        }
        innovationCovariance[cell(rowA, rowB)] = sum;
      }
    }
    // R, the measurement's own noise.
    innovationCovariance[XX] += noise.xx();
    innovationCovariance[XY] += noise.xy();
    innovationCovariance[YX] += noise.xy();
    innovationCovariance[YY] += noise.yy();
    if (rowCount == POSE_ROWS) {
      innovationCovariance[HH] += noise.heading();
    }
    makeInnovationCovarianceSymmetric();
  }

  /**
   * Fills in the gain, K = C S^-1, where C is the covariance of the state with the measurement.
   *
   * <p>A measurement without a heading of its own leaves the heading alone unless the filter was
   * made to let every fix correct it: its gain for the heading offset is set to zero. That is not
   * the gain a Kalman filter would choose, which is why {@link #correctUncertainty} updates the
   * covariance in the form that is right for any gain.
   */
  private void workOutGain() {
    boolean leavesHeadingAlone = rowCount == POSITION_ROWS && !positionFixesCorrectHeading;
    for (int state = 0; state < STATES; state++) {
      for (int row = 0; row < rowCount; row++) {
        double sum = 0;
        for (int k = 0; k < rowCount; k++) {
          sum += stateWithMeasurement[cell(state, k)] * innovationCovarianceInverse[cell(k, row)];
        }
        if (state == HEADING && leavesHeadingAlone) {
          gain[cell(state, row)] = 0;
        } else {
          gain[cell(state, row)] = sum;
        }
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
    for (int state = 0; state < STATES; state++) {
      for (int row = 0; row < rowCount; row++) {
        double sum = 0;
        for (int k = 0; k < rowCount; k++) {
          sum += gain[cell(state, k)] * innovationCovariance[cell(k, row)];
        }
        gainTimesInnovationCovariance[cell(state, row)] = sum;
      }
    }
    for (int stateA = 0; stateA < STATES; stateA++) {
      for (int stateB = 0; stateB < STATES; stateB++) {
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
        measurementWithConsidered[cross(row, column)] =
            crossCovariance[cross(X, column)] * stateSeenBy[cell(row, X)]
                + crossCovariance[cross(Y, column)] * stateSeenBy[cell(row, Y)]
                + crossCovariance[cross(HEADING, column)] * stateSeenBy[cell(row, HEADING)];
      }
      for (int error = 0; error < shared.count(); error++) {
        int column = shared.column(error);
        if (ConsideredErrors.hasColumn(column)) {
          measurementWithConsidered[cross(row, column)] +=
              shared.variance(error) * shared.seenBy(row, error);
        }
      }
    }
    for (int state = 0; state < STATES; state++) {
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

  private void makeCovarianceSymmetric() {
    covariance[XY] = covariance[YX] = (covariance[XY] + covariance[YX]) / 2;
    covariance[XH] = covariance[HX] = (covariance[XH] + covariance[HX]) / 2;
    covariance[YH] = covariance[HY] = (covariance[YH] + covariance[HY]) / 2;
  }

  private void makeInnovationCovarianceSymmetric() {
    for (int rowA = 0; rowA < rowCount; rowA++) {
      for (int rowB = rowA + 1; rowB < rowCount; rowB++) {
        double mean =
            (innovationCovariance[cell(rowA, rowB)] + innovationCovariance[cell(rowB, rowA)]) / 2;
        innovationCovariance[cell(rowA, rowB)] = mean;
        innovationCovariance[cell(rowB, rowA)] = mean;
      }
    }
  }

  private static double square(double value) {
    return value * value;
  }
}
