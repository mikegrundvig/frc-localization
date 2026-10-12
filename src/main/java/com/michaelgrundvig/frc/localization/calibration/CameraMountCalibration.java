package com.michaelgrundvig.frc.localization.calibration;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.VecBuilder;

/**
 * Where a camera really is on the robot, from the tags it sees while the robot's pose is known
 * (from the other cameras, or a surveyed spot): the mount that best explains every sighting, found
 * by least squares over its six degrees of freedom, starting from the configured mount.
 *
 * <p>Each sighting says where the tag should be if the mount were right: the robot's pose, then the
 * mount, then the camera's view of the tag. The solver moves the mount until those land on the
 * tags' real places, and turns the tags to their real orientations. Each miss is weighed by how
 * well a camera sees it ({@link TagSighting}).
 *
 * <p>Sightings must vary: a camera 4 in low and one tilted down 2° put a tag 3 m away in the same
 * wrong place. Tags seen from near and far, and from several angles, tell them apart; the result
 * says how sure it is of each value, so a poorly varied set shows as wide uncertainty.
 *
 * <p><b>The method.</b> The fit is nonlinear least squares, solved by Levenberg-Marquardt ({@link
 * #minimize}). The six values it solves for are a small change to the mount in the mount's own
 * frame: a shift along x, y and z in meters, then a rotation vector about x, y and z in radians.
 * The Jacobian (how each residual changes with each of the six) is found numerically, by nudging
 * each value a little either way.
 */
public final class CameraMountCalibration {
  /** The mount's degrees of freedom: x, y, z, then rotations about x, y, z. */
  private static final int MOUNT_VALUES = 6;

  /** Fewer sightings than this cannot fix a mount's six values. */
  private static final int FEWEST_SIGHTINGS = 3;

  /** An alternate pose closer to the camera than this is no pose at all: there is none. */
  private static final double NO_ALTERNATE_BELOW_METERS = 1e-9;

  /**
   * How many times, at most, each tag's pose is chosen again at the mount solved so far, and the
   * mount solved again.
   */
  private static final int MOST_ROUNDS_OF_CHOOSING = 5;

  // Levenberg-Marquardt's settings.
  private static final int MOST_ITERATIONS = 100;
  private static final double FIRST_DAMPING = 1e-3;
  private static final double MOST_DAMPING = 1e12;
  private static final double LEAST_DAMPING = 1e-12;

  /**
   * The damping grows by this factor after a step that made things worse, and falls after a good
   * one.
   */
  private static final double DAMPING_FACTOR = 10;

  /** A diagonal value of the normal matrix smaller than this is damped as if it were this. */
  private static final double LEAST_DAMPED_DIAGONAL = 1e-12;

  /** The fit has converged when a step improves the cost by less than this share of it. */
  private static final double CONVERGED_COST_SHARE = 1e-14;

  /** Or when the step itself is shorter than this. */
  private static final double CONVERGED_STEP = 1e-10;

  /** How far each mount value is nudged, either way, to find the Jacobian numerically. */
  private static final double JACOBIAN_NUDGE = 1e-6;

  // Two sightings are from the same robot pose when the robot stood within this of where it stood
  // for the other, and was turned within this of how it was turned.
  private static final double SAME_POSE_WITHIN_METERS = 0.05;
  private static final double SAME_POSE_WITHIN_RADIANS = Math.toRadians(2);

  private final Transform3d configured;
  private final List<TagSighting> sightings = new ArrayList<>();

  /**
   * The solved mount.
   *
   * @param robotToCamera where the camera is, as {@code CameraConfig.robotToCamera()} gives it
   * @param residualBeforeMeters how far the tags landed from their places across the line of sight,
   *     where a camera sees them sharply, with the configured mount, on average (root mean square)
   * @param residualAfterMeters and with the solved mount
   * @param sightings how many tag sightings it rests on
   * @param stdDevs how sure it is of each value: x, y, z in meters, then rotations about x, y, z in
   *     radians
   */
  public record Result(
      Transform3d robotToCamera,
      double residualBeforeMeters,
      double residualAfterMeters,
      int sightings,
      double[] stdDevs) {}

  /**
   * @param configuredRobotToCamera where the configuration says the camera is: the start
   */
  public CameraMountCalibration(Transform3d configuredRobotToCamera) {
    configured = configuredRobotToCamera;
  }

  /**
   * A tag the camera saw. Of the two poses the vision software found for it, the solver uses the
   * one that fits the mount it's converging on: the other is the tag turned over.
   *
   * @param robot where the robot was, tilted as it was, when the image was taken
   * @param bestCameraToTag the vision software's best pose of the tag, from the camera
   * @param alternateCameraToTag its other candidate; zero when there is none
   * @param fieldToTag where the tag really is
   */
  public void add(
      Pose3d robot,
      Transform3d bestCameraToTag,
      Transform3d alternateCameraToTag,
      Pose3d fieldToTag) {
    boolean hasAlternate =
        alternateCameraToTag.getTranslation().getNorm() > NO_ALTERNATE_BELOW_METERS;
    Optional<Transform3d> alternate =
        hasAlternate ? Optional.of(alternateCameraToTag) : Optional.empty();
    sightings.add(new TagSighting(robot, bestCameraToTag, alternate, fieldToTag));
  }

  public int sightings() {
    return sightings.size();
  }

  /**
   * Solves for the mount.
   *
   * @return empty with fewer than three sightings
   */
  public Optional<Result> solve() {
    if (sightings.size() < FEWEST_SIGHTINGS) {
      return Optional.empty();
    }
    // Which of a tag's two poses is right depends on the mount, and the mount on the poses. So:
    // choose each tag's pose at the mount so far, solve for the mount, and do both again until no
    // choice changes.
    Transform3d mount = configured;
    for (int round = 0; round < MOST_ROUNDS_OF_CHOOSING; round++) {
      boolean aChoiceChanged = false;
      for (TagSighting sighting : sightings) {
        aChoiceChanged |= sighting.choose(mount);
      }
      boolean isFirstRound = round == 0;
      if (!isFirstRound && !aChoiceChanged) {
        break;
      }
      mount = minimize(mount);
    }
    double[] residuals = residuals(mount);
    return Optional.of(
        new Result(
            mount,
            rmsMissAcrossLineOfSight(configured),
            rmsMissAcrossLineOfSight(mount),
            sightings.size(),
            stdDevs(mount, residuals)));
  }

  /**
   * The mount that makes the sum of the squared residuals least, by Levenberg-Marquardt from a
   * start, with each sighting's pose as currently chosen.
   *
   * <p>Each iteration linearizes the residuals r about the mount so far, with their Jacobian J, and
   * solves the damped normal equations for a step δ:
   *
   * <pre>
   *   (J^T J + damping * diag(J^T J)) δ = -J^T r
   * </pre>
   *
   * <p>With little damping that is a Gauss-Newton step, which converges fast near the answer. With
   * much it is a short step straight downhill, which is safe far from it. A step that lowers the
   * cost is taken and the damping reduced. One that does not is thrown away and the damping raised,
   * until a step helps or the damping is as large as it may be.
   */
  private Transform3d minimize(Transform3d start) {
    Transform3d mount = start;
    double[] residuals = residuals(mount);
    double cost = sumOfSquares(residuals);
    double damping = FIRST_DAMPING;
    for (int iteration = 0; iteration < MOST_ITERATIONS; iteration++) {
      double[][] jacobian = jacobian(mount, residuals);
      double[][] normal = jacobianTransposeTimesJacobian(jacobian);
      double[] negativeGradient = negativeJacobianTransposeTimes(jacobian, residuals);

      boolean carryOn = false;
      while (damping < MOST_DAMPING) {
        Optional<double[]> step = LinearSystem.solve(damped(normal, damping), negativeGradient);
        if (step.isEmpty()) {
          damping *= DAMPING_FACTOR;
          continue;
        }
        Transform3d next = mount.plus(nudge(step.get()));
        double[] nextResiduals = residuals(next);
        double nextCost = sumOfSquares(nextResiduals);
        boolean stepHelped = nextCost < cost;
        if (!stepHelped) {
          damping *= DAMPING_FACTOR;
          continue;
        }
        boolean converged =
            cost - nextCost < CONVERGED_COST_SHARE * Math.max(1, cost)
                || norm(step.get()) < CONVERGED_STEP;
        mount = next;
        residuals = nextResiduals;
        cost = nextCost;
        damping = Math.max(damping / DAMPING_FACTOR, LEAST_DAMPING);
        carryOn = !converged;
        break;
      }
      if (!carryOn) {
        break;
      }
    }
    return mount;
  }

  /** J^T J, the normal matrix of the least-squares fit: a row and a column for each mount value. */
  private static double[][] jacobianTransposeTimesJacobian(double[][] jacobian) {
    double[][] normal = new double[MOUNT_VALUES][MOUNT_VALUES];
    for (double[] row : jacobian) {
      for (int a = 0; a < MOUNT_VALUES; a++) {
        for (int b = 0; b < MOUNT_VALUES; b++) {
          normal[a][b] += row[a] * row[b];
        }
      }
    }
    return normal;
  }

  /** -J^T r: the downhill direction of the cost, one value for each mount value. */
  private static double[] negativeJacobianTransposeTimes(double[][] jacobian, double[] residuals) {
    double[] gradient = new double[MOUNT_VALUES];
    for (int row = 0; row < residuals.length; row++) {
      for (int a = 0; a < MOUNT_VALUES; a++) {
        gradient[a] += jacobian[row][a] * residuals[row];
      }
    }
    double[] negativeGradient = new double[MOUNT_VALUES];
    for (int a = 0; a < MOUNT_VALUES; a++) {
      negativeGradient[a] = -gradient[a];
    }
    return negativeGradient;
  }

  /** A copy of the normal matrix with its diagonal raised by the damping's share of itself. */
  private static double[][] damped(double[][] normal, double damping) {
    double[][] damped = new double[MOUNT_VALUES][];
    for (int a = 0; a < MOUNT_VALUES; a++) {
      damped[a] = normal[a].clone();
      damped[a][a] += damping * Math.max(normal[a][a], LEAST_DAMPED_DIAGONAL);
    }
    return damped;
  }

  /**
   * How far each mount value could be off, as a standard deviation, from how well the solved mount
   * fits and how varied the sightings are.
   *
   * <p>This is the usual covariance of a least-squares fit: the inverse of J^T J times the variance
   * of the residuals, which is their sum of squares over the fit's spare degrees of freedom. Each
   * sighting counts six of its seven residuals toward those: its miss across the line of sight has
   * only two degrees of freedom, though it is written as three numbers. The fit itself uses up six.
   *
   * <p>Sightings from one robot pose share that pose's error, so together they count as one: the
   * variance is widened by how many sightings each distinct pose holds, on average.
   */
  private double[] stdDevs(Transform3d mount, double[] residuals) {
    double[][] normal = jacobianTransposeTimesJacobian(jacobian(mount, residuals));
    int independentResidualsPerSighting = TagSighting.RESIDUALS - 1;
    int degreesOfFreedom =
        Math.max(1, sightings.size() * independentResidualsPerSighting - MOUNT_VALUES);
    double variance = sumOfSquares(residuals) / degreesOfFreedom;
    double sightingsPerPose = (double) sightings.size() / distinctPoses();
    variance *= Math.max(1.0, sightingsPerPose);
    double[] stdDevs = new double[MOUNT_VALUES];
    for (int a = 0; a < MOUNT_VALUES; a++) {
      // Column a of the inverse of J^T J is the solution of (J^T J) x = the a-th unit vector, and
      // its a-th value is the inverse's diagonal there.
      double[] unit = new double[MOUNT_VALUES];
      unit[a] = 1;
      Optional<double[]> column = LinearSystem.solve(normal, unit);
      if (column.isEmpty()) {
        // The sightings do not fix this value at all.
        stdDevs[a] = Double.POSITIVE_INFINITY;
      } else {
        stdDevs[a] = Math.sqrt(Math.max(0, column.get()[a] * variance));
      }
    }
    return stdDevs;
  }

  /** How many different robot poses the sightings were taken from. */
  private int distinctPoses() {
    List<Pose3d> distinct = new ArrayList<>();
    for (TagSighting sighting : sightings) {
      if (!isOneOf(distinct, sighting.robot())) {
        distinct.add(sighting.robot());
      }
    }
    return distinct.size();
  }

  private static boolean isOneOf(List<Pose3d> poses, Pose3d robot) {
    for (Pose3d pose : poses) {
      double apartMeters = pose.getTranslation().getDistance(robot.getTranslation());
      if (apartMeters < SAME_POSE_WITHIN_METERS
          && pose.getRotation().relativeTo(robot.getRotation()).getAngle()
              < SAME_POSE_WITHIN_RADIANS) {
        return true;
      }
    }
    return false;
  }

  /**
   * A small change to the mount, in its own frame.
   *
   * @param delta a shift along x, y and z, then a rotation vector about x, y and z
   */
  private static Transform3d nudge(double[] delta) {
    return new Transform3d(
        new Translation3d(delta[0], delta[1], delta[2]),
        new Rotation3d(VecBuilder.fill(delta[3], delta[4], delta[5])));
  }

  /**
   * The Jacobian of the residuals at a mount: a row for each residual, a column for each mount
   * value. Each column is a central difference: the residuals with that value nudged up, less the
   * residuals with it nudged down, over twice the nudge.
   */
  private double[][] jacobian(Transform3d mount, double[] residuals) {
    double[][] jacobian = new double[residuals.length][MOUNT_VALUES];
    for (int a = 0; a < MOUNT_VALUES; a++) {
      double[] delta = new double[MOUNT_VALUES];
      delta[a] = JACOBIAN_NUDGE;
      double[] nudgedUp = residuals(mount.plus(nudge(delta)));
      delta[a] = -JACOBIAN_NUDGE;
      double[] nudgedDown = residuals(mount.plus(nudge(delta)));
      for (int row = 0; row < residuals.length; row++) {
        jacobian[row][a] = (nudgedUp[row] - nudgedDown[row]) / (2 * JACOBIAN_NUDGE);
      }
    }
    return jacobian;
  }

  /** Every sighting's residuals at a mount, one sighting after another. */
  private double[] residuals(Transform3d mount) {
    double[] residuals = new double[sightings.size() * TagSighting.RESIDUALS];
    for (int i = 0; i < sightings.size(); i++) {
      sightings.get(i).residualsInto(mount, residuals, TagSighting.RESIDUALS * i);
    }
    return residuals;
  }

  /** How far the tags land from their places across the line of sight, root mean square. */
  private double rmsMissAcrossLineOfSight(Transform3d mount) {
    double sum = 0;
    for (TagSighting sighting : sightings) {
      double across = sighting.missAcrossLineOfSight(mount);
      sum += across * across;
    }
    return Math.sqrt(sum / sightings.size());
  }

  private static double sumOfSquares(double[] values) {
    double sum = 0;
    for (double value : values) {
      sum += value * value;
    }
    return sum;
  }

  private static double norm(double[] values) {
    return Math.sqrt(sumOfSquares(values));
  }
}
