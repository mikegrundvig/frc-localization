package com.michaelgrundvig.frc.localization;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * How uncertain a pose is: the covariance of its position (x and y, in square meters, and how they
 * vary together) and the variance of its heading (square radians). Heading stands apart from
 * position, as in the estimator: the gyro owns heading.
 *
 * @param xx the variance along the field's x
 * @param xy the covariance of x and y
 * @param yy the variance along y
 * @param heading the variance of the heading; infinite when the pose says nothing about heading
 */
public record PoseCovariance(double xx, double xy, double yy, double heading) {
  public PoseCovariance {
    if (!Double.isFinite(xx) || !Double.isFinite(yy) || !Double.isFinite(xy) || xx < 0 || yy < 0) {
      throw new IllegalArgumentException(
          "the position variances must be finite, and zero or more: " + xx + ", " + yy);
    }
    // No real spread of positions has x and y varying together by more than their own spreads
    // allow (the matrix would not be positive semidefinite); a little slack absorbs rounding.
    if (xy * xy - xx * yy > 1e-9 * Math.max(xx * yy, xy * xy) + Double.MIN_NORMAL) {
      throw new IllegalArgumentException(
          "the covariance of x and y, " + xy + ", is more than their variances allow");
    }
    if (!(heading >= 0)) {
      throw new IllegalArgumentException(
          "the heading variance must be zero or more (infinite when unknown): " + heading);
    }
  }

  /**
   * Independent standard deviations: meters along x and along y, and radians of heading ({@code
   * Double.POSITIVE_INFINITY} when the heading is unknown).
   */
  public static PoseCovariance fromStdDevs(double x, double y, double heading) {
    return new PoseCovariance(x * x, 0, y * y, heading * heading);
  }

  public double xStdDev() {
    return Math.sqrt(xx);
  }

  public double yStdDev() {
    return Math.sqrt(yy);
  }

  public double headingStdDev() {
    return Math.sqrt(heading);
  }

  /**
   * The position's spread along its most uncertain direction, in meters. It is the square root of
   * the larger eigenvalue of the position's covariance {@code [xx xy; xy yy]}:
   *
   * <pre>largest variance = (xx + yy) / 2 + sqrt(((xx - yy) / 2)² + xy²)</pre>
   */
  public double largestPositionStdDev() {
    return Math.sqrt((xx + yy) / 2 + Math.hypot((xx - yy) / 2, xy));
  }

  /** Whether the pose says anything about heading. */
  public boolean hasHeading() {
    return Double.isFinite(heading);
  }

  /**
   * The ellipse around the estimated position that holds the true position this often, if the
   * errors are as this covariance says (normally distributed): the shape a view draws for "where
   * the robot could be". A position error measured in standard deviations along each axis, {@code
   * k}, has {@code k^2} following a chi-squared distribution with two degrees of freedom, so the
   * ellipse reaching {@code k = sqrt(-2 ln(1 - probability))} holds that share: 3.72 for 99.9%.
   *
   * @param probability the share of true positions inside, above 0 and below 1
   */
  public Ellipse ellipse(double probability) {
    if (!(probability > 0 && probability < 1)) {
      throw new IllegalArgumentException("a probability above 0 and below 1: " + probability);
    }
    double standardDeviationsOut = Math.sqrt(-2 * Math.log1p(-probability));
    // The covariance's eigenvalues are the variances along the ellipse's axes: mean + spread along
    // the longer, mean - spread along the shorter. Rounding can leave the smaller one a hair below
    // zero, which is taken as zero. The longer axis points along the first eigenvector.
    double mean = (xx + yy) / 2;
    double spread = Math.hypot((xx - yy) / 2, xy);
    return new Ellipse(
        standardDeviationsOut * Math.sqrt(mean + spread),
        standardDeviationsOut * Math.sqrt(Math.max(0, mean - spread)),
        0.5 * Math.atan2(2 * xy, xx - yy));
  }

  /**
   * An ellipse around a position, in the field's frame.
   *
   * @param major half its longer axis, meters
   * @param minor half its shorter axis, meters
   * @param angleRadians which way its longer axis points, counterclockwise from the field's x
   */
  public record Ellipse(double major, double minor, double angleRadians) {
    /**
     * A point on the ellipse, from its center: at {@code around} radians along it, counterclockwise
     * from the end of its longer axis.
     */
    public Translation2d pointAt(double around) {
      return pointAt(Math.cos(around), Math.sin(around));
    }

    /**
     * The point at an angle along the ellipse given by its cosine and sine: for a caller that keeps
     * a table of them, to draw the same points loop after loop without the trigonometry.
     */
    public Translation2d pointAt(double cosAround, double sinAround) {
      double alongMajor = major * cosAround;
      double alongMinor = minor * sinAround;
      double c = Math.cos(angleRadians);
      double s = Math.sin(angleRadians);
      return new Translation2d(c * alongMajor - s * alongMinor, s * alongMajor + c * alongMinor);
    }
  }

  /**
   * The same uncertainty in a frame turned by an angle: a camera's "less certain along my line of
   * sight", written in the robot's frame, turned into the field's by the robot's heading.
   */
  public PoseCovariance rotatedBy(Rotation2d rotation) {
    double c = rotation.getCos();
    double s = rotation.getSin();
    // The covariance C in the turned frame is R C R^T, for the rotation R = [c -s; s c]. Written
    // out, with C = [xx xy; xy yy], that is the three numbers below. Turning the frame doesn't
    // change how sure the heading is.
    return new PoseCovariance(
        c * c * xx - 2 * c * s * xy + s * s * yy,
        c * s * (xx - yy) + (c * c - s * s) * xy,
        s * s * xx + 2 * c * s * xy + c * c * yy,
        heading);
  }
}
