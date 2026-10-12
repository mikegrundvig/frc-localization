package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

class PoseCovarianceTest {
  @Test
  void standardDeviationsAreTheRootsOfTheVariances() {
    PoseCovariance covariance = PoseCovariance.fromStdDevs(0.1, 0.2, 0.05);

    assertThat(covariance.xx()).isCloseTo(0.01, within(1e-15));
    assertThat(covariance.yy()).isCloseTo(0.04, within(1e-15));
    assertThat(covariance.xy()).isZero();
    assertThat(covariance.xStdDev()).isCloseTo(0.1, within(1e-15));
    assertThat(covariance.yStdDev()).isCloseTo(0.2, within(1e-15));
    assertThat(covariance.headingStdDev()).isCloseTo(0.05, within(1e-15));
    assertThat(covariance.hasHeading()).isTrue();
  }

  /** A solve that took its heading from the gyro tells the estimator nothing about heading. */
  @Test
  void theLargestSpreadIsAlongTheWorstDirection() {
    assertThat(PoseCovariance.fromStdDevs(0.2, 0.1, 0.01).largestPositionStdDev())
        .isCloseTo(0.2, within(1e-12));
    // Spreads of 0.02 m² each, varying together by 0.01: 0.03 m² along the diagonal.
    assertThat(new PoseCovariance(0.02, 0.01, 0.02, 0.001).largestPositionStdDev())
        .isCloseTo(Math.sqrt(0.03), within(1e-12));
  }

  @Test
  void anInfiniteHeadingVarianceMeansNoHeading() {
    PoseCovariance covariance = PoseCovariance.fromStdDevs(0.1, 0.1, Double.POSITIVE_INFINITY);

    assertThat(covariance.hasHeading()).isFalse();
  }

  @Test
  void aQuarterTurnSwapsTheSpreads() {
    PoseCovariance alongX = PoseCovariance.fromStdDevs(0.3, 0.1, 0.01);

    PoseCovariance turned = alongX.rotatedBy(Rotation2d.CCW_PI_2);

    assertThat(turned.xx()).isCloseTo(0.01, within(1e-12));
    assertThat(turned.yy()).isCloseTo(0.09, within(1e-12));
    assertThat(turned.xy()).isCloseTo(0.0, within(1e-12));
    assertThat(turned.heading()).isEqualTo(alongX.heading());
  }

  @Test
  void turningKeepsTheSpreadAndTurningBackUndoesIt() {
    PoseCovariance correlated = new PoseCovariance(0.04, 0.01, 0.02, 0.001);
    Rotation2d angle = Rotation2d.fromDegrees(37);

    PoseCovariance turned = correlated.rotatedBy(angle);
    PoseCovariance back = turned.rotatedBy(angle.unaryMinus());

    assertThat(turned.xx() + turned.yy()).isCloseTo(0.06, within(1e-15));
    assertThat(turned.xx() * turned.yy() - turned.xy() * turned.xy())
        .isCloseTo(0.04 * 0.02 - 0.01 * 0.01, within(1e-15));
    assertThat(back.xx()).isCloseTo(0.04, within(1e-15));
    assertThat(back.xy()).isCloseTo(0.01, within(1e-15));
    assertThat(back.yy()).isCloseTo(0.02, within(1e-15));
  }

  /** Certain along one line and not across it: rounding in a turn mustn't make it invalid. */
  @Test
  void aSpreadAlongOneLineSurvivesTurning() {
    PoseCovariance line = new PoseCovariance(1, 2, 4, 0);

    PoseCovariance turned = line.rotatedBy(Rotation2d.fromDegrees(37));

    assertThat(turned.xx() * turned.yy() - turned.xy() * turned.xy()).isCloseTo(0.0, within(1e-12));
  }

  @Test
  void onlyRealSpreadsAreAllowed() {
    assertThatThrownBy(() -> new PoseCovariance(-0.1, 0, 0.1, 0))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> new PoseCovariance(Double.NaN, 0, 0.1, 0))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> new PoseCovariance(0.1, 0, Double.POSITIVE_INFINITY, 0))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> new PoseCovariance(0.01, 0.02, 0.01, 0))
        .hasMessageContaining("more than their variances allow");
    assertThatThrownBy(() -> new PoseCovariance(0.1, 0, 0.1, Double.NaN))
        .hasMessageContaining("heading");
    assertThatThrownBy(() -> new PoseCovariance(0.1, 0, 0.1, -1)).hasMessageContaining("heading");
  }

  /** 99.9% of a two-dimensional normal error lies within 3.72 of its standard deviations. */
  @Test
  void theEllipseReachesTheSpreadsTimesTheChiSquaredRadius() {
    PoseCovariance.Ellipse ellipse = PoseCovariance.fromStdDevs(0.2, 0.1, 0.01).ellipse(0.999);

    double k = Math.sqrt(-2 * Math.log(0.001));
    assertThat(k).isCloseTo(3.717, within(0.001));
    assertThat(ellipse.major()).isCloseTo(0.2 * k, within(1e-12));
    assertThat(ellipse.minor()).isCloseTo(0.1 * k, within(1e-12));
    assertThat(ellipse.angleRadians()).isCloseTo(0, within(1e-12));
    assertThat(ellipse.pointAt(0).getX()).isCloseTo(0.2 * k, within(1e-12));
    assertThat(ellipse.pointAt(Math.PI / 2).getY()).isCloseTo(0.1 * k, within(1e-12));
  }

  /** Every point on a turned ellipse is the same number of standard deviations out. */
  @Test
  void aCorrelatedSpreadTurnsTheEllipse() {
    PoseCovariance turned =
        PoseCovariance.fromStdDevs(0.3, 0.1, 0.01).rotatedBy(Rotation2d.fromDegrees(30));

    PoseCovariance.Ellipse ellipse = turned.ellipse(0.95);

    double k2 = -2 * Math.log(0.05);
    assertThat(Math.toDegrees(ellipse.angleRadians())).isCloseTo(30, within(1e-9));
    double determinant = turned.xx() * turned.yy() - turned.xy() * turned.xy();
    for (int i = 0; i < 12; i++) {
      Translation2d point = ellipse.pointAt(i * Math.PI / 6);
      double x = point.getX();
      double y = point.getY();
      double squared =
          (turned.yy() * x * x - 2 * turned.xy() * x * y + turned.xx() * y * y) / determinant;
      assertThat(squared).as("point " + i).isCloseTo(k2, within(1e-9));
    }
  }

  @Test
  void anEllipseNeedsAProbability() {
    PoseCovariance covariance = PoseCovariance.fromStdDevs(0.1, 0.1, 0.01);

    assertThatThrownBy(() -> covariance.ellipse(1)).isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> covariance.ellipse(0)).isInstanceOf(IllegalArgumentException.class);
    assertThat(PoseCovariance.fromStdDevs(0, 0, 0).ellipse(0.5).major()).isZero();
  }
}
