package com.michaelgrundvig.frc.localization;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * Swerve kinematics for localization. Each module's travel between two samples is a small vector
 * (how far it rolled, at the angle it pointed); the robot's motion is the one rigid motion that
 * best fits all of them (least squares, as WPILib's odometry does). How badly it fits is the
 * residual: modules that disagree with every rigid motion are slipping, lifted, or being pushed,
 * which is when odometry deserves the least trust (one top team's skid ratio, another's wheel
 * scores).
 *
 * <p>The residual is reported as how far the disagreement may have put the fitted travel off: the
 * modules' misses, spread over what the fit leaves free (two numbers a module, less the motion's
 * three), carried into the travel as least squares carries each module's error into it. One module
 * of four slipping puts the fit off by a quarter of its slip, and the residual says that much,
 * within about a tenth, whichever way it slips (exactly, for a square drivetrain's wheel slipping
 * forward or sideways).
 *
 * <p>A module that didn't report (its sample reads a value that isn't a number: its motor
 * controllers off the bus, say) is left out, and the motion fits the rest: two modules or more fix
 * it, so a robot driving on three keeps its odometry. With fewer, nothing can be fitted, and the
 * motion is none, as doubtful as the farthest a reporting wheel rolled.
 */
public final class SwerveKinematicsModel implements DriveKinematicsModel {
  // The three values of a rigid motion, in the order the fit's rows follow.
  private static final int DX = 0;
  private static final int DY = 1;
  private static final int DTHETA = 2;
  private static final int MOTION_VALUES = 3;

  /** Each module gives two numbers to the fit: how far it traveled along x, and along y. */
  private static final int TRAVELS_PER_MODULE = 2;

  /** Fewer modules than this cannot fix a rigid motion's three values. */
  private static final int FEWEST_MODULES = 2;

  /** Each set of reporting modules is one bit a module in an int, which has room for this many. */
  private static final int MOST_MODULES = 30;

  /** Where each module sits from the robot's center, in meters. */
  private final double[] moduleX;

  private final double[] moduleY;

  /**
   * The least-squares fit of a rigid motion to the travels of some of the modules.
   *
   * <p>The model is {@code travels = A motion}. For each module at (x, y), A has two rows: {@code
   * [1, 0, -y]} for its travel along x and {@code [0, 1, x]} for its travel along y, since turning
   * the robot by dθ carries that module by (-y dθ, x dθ). The motion that fits best is {@code (A^T
   * A)^-1 A^T travels}.
   *
   * @param modules which modules it is over, by their index, in increasing order
   * @param travelsToMotion (A^T A)^-1 A^T, stored row by row: a row for each of dx, dy and dθ, and
   *     two columns for each module in {@code modules}, its travel along x then along y
   * @param missToTravelVariance what the modules' squared misses, added up, are multiplied by to
   *     give the variance of the fitted travel: the travel's share of (A^T A)^-1, over the fit's
   *     spare degrees of freedom (two a module, less the motion's three)
   */
  private record Fit(int[] modules, double[] travelsToMotion, double missToTravelVariance) {
    int moduleCount() {
      return modules.length;
    }

    /** How much one module's travel along x counts toward one value of the motion. */
    double fromTravelX(int motionValue, int moduleInFit) {
      return travelsToMotion[
          motionValue * TRAVELS_PER_MODULE * modules.length + TRAVELS_PER_MODULE * moduleInFit];
    }

    double fromTravelY(int motionValue, int moduleInFit) {
      return travelsToMotion[
          motionValue * TRAVELS_PER_MODULE * modules.length + TRAVELS_PER_MODULE * moduleInFit + 1];
    }
  }

  /**
   * The fit for each set of reporting modules that has come up, by the set's bits: bit i is set
   * when module i reported. Empty for a set that cannot fix a motion. Each is made the first time
   * its set comes up, so a robot that never loses a module makes none.
   */
  private final Map<Integer, Optional<Fit>> fitsByReportingModules = new HashMap<>();

  /** The fit over every module: the one a healthy robot uses every time. */
  private final Optional<Fit> fitOverEveryModule;

  /**
   * @param modules where each module sits, from the robot's center, in the samples' order
   */
  public SwerveKinematicsModel(Translation2d... modules) {
    int count = modules.length;
    if (count < FEWEST_MODULES) {
      throw new IllegalArgumentException("swerve needs at least 2 modules: " + count);
    }
    if (count > MOST_MODULES) {
      throw new IllegalArgumentException("swerve with at most 30 modules: " + count);
    }
    moduleX = new double[count];
    moduleY = new double[count];
    int[] everyModule = new int[count];
    for (int i = 0; i < count; i++) {
      moduleX[i] = modules[i].getX();
      moduleY[i] = modules[i].getY();
      everyModule[i] = i;
    }
    fitOverEveryModule = fit(everyModule);
    if (fitOverEveryModule.isEmpty()) {
      throw new IllegalArgumentException("the modules must not all sit at one point");
    }
  }

  /**
   * The fit over some modules, or empty if they cannot fix a motion: there are fewer than two, or
   * they all sit at one point, about which the robot could turn without any of them moving.
   */
  private Optional<Fit> fit(int[] modules) {
    int count = modules.length;
    if (count < FEWEST_MODULES) {
      return Optional.empty();
    }
    double sumX = 0;
    double sumY = 0;
    double sumOfSquaredDistances = 0;
    for (int module : modules) {
      sumX += moduleX[module];
      sumY += moduleY[module];
      sumOfSquaredDistances +=
          moduleX[module] * moduleX[module] + moduleY[module] * moduleY[module];
    }
    // A^T A, the normal matrix, 3 by 3 and row by row. With A's rows [1, 0, -y] and [0, 1, x] for
    // each module, it comes to this:
    double[] normal = {count, 0, -sumY, 0, count, sumX, -sumY, sumX, sumOfSquaredDistances};
    double[] normalInverse = new double[MOTION_VALUES * MOTION_VALUES];
    if (!Matrices.invert3(normal, normalInverse)) {
      return Optional.empty();
    }
    // (A^T A)^-1 A^T. Its column for a module's travel along x is the inverse times that row of A,
    // (1, 0, -y), and its column for the travel along y is the inverse times (0, 1, x).
    int columns = TRAVELS_PER_MODULE * count;
    double[] travelsToMotion = new double[MOTION_VALUES * columns];
    for (int row = 0; row < MOTION_VALUES; row++) {
      double inverseDx = normalInverse[row * MOTION_VALUES + DX];
      double inverseDy = normalInverse[row * MOTION_VALUES + DY];
      double inverseDtheta = normalInverse[row * MOTION_VALUES + DTHETA];
      for (int k = 0; k < count; k++) {
        int module = modules[k];
        travelsToMotion[row * columns + TRAVELS_PER_MODULE * k] =
            inverseDx - inverseDtheta * moduleY[module];
        travelsToMotion[row * columns + TRAVELS_PER_MODULE * k + 1] =
            inverseDy + inverseDtheta * moduleX[module];
      }
    }
    // The variance of the fitted dx is (A^T A)^-1's dx diagonal times the misses' variance, and
    // likewise dy. Their sum is the travel's. The misses' variance is their squares added up, over
    // the fit's spare degrees of freedom.
    double travelShareOfInverse =
        normalInverse[DX * MOTION_VALUES + DX] + normalInverse[DY * MOTION_VALUES + DY];
    int spareDegreesOfFreedom = TRAVELS_PER_MODULE * count - MOTION_VALUES;
    return Optional.of(
        new Fit(modules, travelsToMotion, travelShareOfInverse / spareDegreesOfFreedom));
  }

  @Override
  public BodyMotion motion(OdometrySample from, OdometrySample to) {
    int everyModule = moduleX.length;
    if (from.wheels() != everyModule || to.wheels() != everyModule) {
      throw new IllegalArgumentException(
          "samples of "
              + from.wheels()
              + " and "
              + to.wheels()
              + " wheels, for "
              + everyModule
              + " modules");
    }
    // Which modules reported in both samples, as a set of bits, and how far the farthest rolled.
    int reportingBits = 0;
    int reportingCount = 0;
    double farthestRolledMeters = 0;
    for (int module = 0; module < everyModule; module++) {
      if (from.reported(module) && to.reported(module)) {
        reportingBits |= 1 << module;
        reportingCount++;
        farthestRolledMeters =
            Math.max(
                farthestRolledMeters,
                Math.abs(to.distanceMeters(module) - from.distanceMeters(module)));
      }
    }
    Optional<Fit> fitOverReporting =
        reportingCount == everyModule
            ? fitOverEveryModule
            : fitsByReportingModules.computeIfAbsent(reportingBits, this::fitOverModulesWithBits);
    if (fitOverReporting.isEmpty()) {
      // Nothing can be fitted. The motion is taken as none, and as doubtful as the farthest any
      // reporting wheel rolled.
      return new BodyMotion(0, 0, 0, farthestRolledMeters);
    }
    Fit fit = fitOverReporting.get();
    int count = fit.moduleCount();

    // Each module's travel, as a vector in the robot's frame: its x, then its y.
    double[] travels = new double[TRAVELS_PER_MODULE * count];
    for (int k = 0; k < count; k++) {
      int module = fit.modules()[k];
      double rolledMeters = to.distanceMeters(module) - from.distanceMeters(module);
      // The module may have steered during the interval: take the angle halfway through it.
      double angle =
          from.angleRadians(module)
              + MathUtil.angleModulus(to.angleRadians(module) - from.angleRadians(module)) / 2;
      travels[TRAVELS_PER_MODULE * k] = rolledMeters * Math.cos(angle);
      travels[TRAVELS_PER_MODULE * k + 1] = rolledMeters * Math.sin(angle);
    }

    // The motion that fits them best: (A^T A)^-1 A^T times the travels.
    double dx = 0;
    double dy = 0;
    double dtheta = 0;
    for (int k = 0; k < count; k++) {
      double travelX = travels[TRAVELS_PER_MODULE * k];
      dx += fit.fromTravelX(DX, k) * travelX;
      dy += fit.fromTravelX(DY, k) * travelX;
      dtheta += fit.fromTravelX(DTHETA, k) * travelX;
      double travelY = travels[TRAVELS_PER_MODULE * k + 1];
      dx += fit.fromTravelY(DX, k) * travelY;
      dy += fit.fromTravelY(DY, k) * travelY;
      dtheta += fit.fromTravelY(DTHETA, k) * travelY;
    }

    // How far each module's travel misses what that motion would have carried it by.
    double sumOfSquaredMisses = 0;
    for (int k = 0; k < count; k++) {
      int module = fit.modules()[k];
      double missX = travels[TRAVELS_PER_MODULE * k] - (dx - moduleY[module] * dtheta);
      double missY = travels[TRAVELS_PER_MODULE * k + 1] - (dy + moduleX[module] * dtheta);
      sumOfSquaredMisses += missX * missX + missY * missY;
    }
    double residualMeters = Math.sqrt(sumOfSquaredMisses * fit.missToTravelVariance());
    return new BodyMotion(dx, dy, dtheta, residualMeters);
  }

  /** The fit over the modules whose bits are set, or empty if they cannot fix a motion. */
  private Optional<Fit> fitOverModulesWithBits(int reportingBits) {
    int[] modules = new int[Integer.bitCount(reportingBits)];
    int next = 0;
    for (int module = 0; module < moduleX.length; module++) {
      boolean reported = (reportingBits & (1 << module)) != 0;
      if (reported) {
        modules[next++] = module;
      }
    }
    return fit(modules);
  }
}
