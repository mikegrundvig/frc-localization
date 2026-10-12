package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.Random;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;

/** Wheel readings in, the robot's motion (and how well the wheels agree on it) out. */
class KinematicsModelTest {
  private static final Translation2d[] MODULES = {
    new Translation2d(0.3, 0.3),
    new Translation2d(0.3, -0.3),
    new Translation2d(-0.3, 0.3),
    new Translation2d(-0.3, -0.3)
  };
  private final SwerveKinematicsModel swerve = new SwerveKinematicsModel(MODULES);

  /** The wheels' readings after a rigid motion, from readings of zero, all pointing one way. */
  private static OdometrySample afterRigidMotion(double dx, double dy, double dtheta) {
    double[] distances = new double[4];
    double[] angles = new double[4];
    for (int i = 0; i < 4; i++) {
      double travelX = dx - MODULES[i].getY() * dtheta;
      double travelY = dy + MODULES[i].getX() * dtheta;
      distances[i] = Math.hypot(travelX, travelY);
      angles[i] = Math.atan2(travelY, travelX);
    }
    return new OdometrySample(1, distances, angles);
  }

  private static OdometrySample startingFrom(OdometrySample after) {
    double[] angles = new double[4];
    for (int i = 0; i < 4; i++) {
      angles[i] = after.angleRadians(i);
    }
    return new OdometrySample(0, new double[4], angles);
  }

  private BodyMotion motionOf(double dx, double dy, double dtheta) {
    OdometrySample after = afterRigidMotion(dx, dy, dtheta);
    return swerve.motion(startingFrom(after), after);
  }

  @Test
  void aRigidMotionComesBackExactlyWithNoResidual() {
    for (double[] motion :
        new double[][] {{0.1, 0, 0}, {0.05, 0.08, 0}, {0, 0, 0.1}, {0.05, 0.02, 0.03}}) {
      BodyMotion found = motionOf(motion[0], motion[1], motion[2]);

      assertThat(found.dxMeters()).isCloseTo(motion[0], within(1e-12));
      assertThat(found.dyMeters()).isCloseTo(motion[1], within(1e-12));
      assertThat(found.dthetaRadians()).isCloseTo(motion[2], within(1e-12));
      assertThat(found.residualMeters()).isCloseTo(0.0, within(1e-12));
    }
  }

  /**
   * A wheel that spins without the robot moving to match disagrees with every rigid motion, and the
   * residual says how far that put the motion off: no more, no less.
   */
  @Test
  void aSlippingWheelShowsAsAResidualAsLargeAsTheErrorItCauses() {
    OdometrySample after = afterRigidMotion(0.1, 0, 0);
    double[] distances = new double[4];
    double[] angles = new double[4];
    for (int i = 0; i < 4; i++) {
      distances[i] = after.distanceMeters(i);
      angles[i] = after.angleRadians(i);
    }
    distances[0] += 0.04; // the front left wheel spun 4 cm more than the robot moved
    OdometrySample slipped = new OdometrySample(1, distances, angles);

    BodyMotion found = swerve.motion(startingFrom(after), slipped);

    assertThat(found.dxMeters()).as("the others outvote it").isCloseTo(0.11, within(1e-12));
    assertThat(found.residualMeters())
        .as("the centimeter it put the motion off")
        .isCloseTo(0.01, within(1e-12));
  }

  /**
   * Whichever way a wheel slips, on a rectangular drivetrain too, its residual is within about a
   * tenth of the error it causes.
   */
  @Test
  void onARectangleTheResidualIsCloseToTheError() {
    Translation2d[] rectangle = {
      new Translation2d(0.35, 0.2),
      new Translation2d(0.35, -0.2),
      new Translation2d(-0.35, 0.2),
      new Translation2d(-0.35, -0.2)
    };
    SwerveKinematicsModel rectangular = new SwerveKinematicsModel(rectangle);
    double[] angles = new double[4];
    for (int k = 0; k < 12; k++) {
      double direction = k * Math.PI / 12;
      for (int slipping = 0; slipping < 4; slipping++) {
        double[] distances = new double[4];
        double[] pointing = angles.clone();
        pointing[slipping] = direction;
        distances[slipping] = 0.04; // standing still, one wheel spins 4 cm
        BodyMotion found =
            rectangular.motion(
                new OdometrySample(0, new double[4], pointing),
                new OdometrySample(1, distances, pointing));

        double error = Math.hypot(found.dxMeters(), found.dyMeters());
        assertThat(found.residualMeters()).isCloseTo(error, within(0.11 * error));
      }
    }
  }

  /** A module steering during the interval traveled, on average, along its middle angle. */
  @Test
  void aModuleSteeringDuringTheStepTravelsAlongItsMiddleAngle() {
    double[] start = {0, 0, 0, 0};
    double[] angles0 = {0, 0, 0, 0};
    double[] angles1 = new double[4];
    double[] distances = new double[4];
    for (int i = 0; i < 4; i++) {
      angles1[i] = Math.toRadians(20);
      distances[i] = 0.1;
    }

    BodyMotion found =
        swerve.motion(
            new OdometrySample(0, start, angles0), new OdometrySample(1, distances, angles1));

    assertThat(Math.toDegrees(Math.atan2(found.dyMeters(), found.dxMeters())))
        .isCloseTo(10.0, within(1e-9));
  }

  @Test
  void itAgreesWithWpilibsOdometryWhenTheModulesHoldTheirAngles() {
    SwerveDriveKinematics wpilib = new SwerveDriveKinematics(MODULES);
    Random random = new Random(7);
    for (int trial = 0; trial < 20; trial++) {
      double[] distances = new double[4];
      double[] angles = new double[4];
      SwerveModulePosition[] start = new SwerveModulePosition[4];
      SwerveModulePosition[] end = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        distances[i] = random.nextGaussian() * 0.05;
        angles[i] = random.nextDouble() * 2 * Math.PI - Math.PI;
        start[i] = new SwerveModulePosition(0, new Rotation2d(angles[i]));
        end[i] = new SwerveModulePosition(distances[i], new Rotation2d(angles[i]));
      }

      BodyMotion ours =
          swerve.motion(
              new OdometrySample(0, new double[4], angles),
              new OdometrySample(1, distances, angles));
      Twist2d theirs = wpilib.toTwist2d(start, end);

      assertThat(ours.dxMeters()).isCloseTo(theirs.dx, within(1e-12));
      assertThat(ours.dyMeters()).isCloseTo(theirs.dy, within(1e-12));
      assertThat(ours.dthetaRadians()).isCloseTo(theirs.dtheta, within(1e-12));
    }
  }

  /**
   * A module that didn't report (its motor controllers off the bus) reads no number, and the motion
   * fits the other three exactly, however the robot moved.
   */
  @Test
  void aModuleThatDidntReportIsLeftOutAndTheRestFixTheMotion() {
    for (double[] motion : new double[][] {{0.1, 0, 0}, {0.05, 0.08, 0}, {0.05, 0.02, 0.03}}) {
      OdometrySample after = afterRigidMotion(motion[0], motion[1], motion[2]);
      OdometrySample before = startingFrom(after);
      double[] distances = new double[4];
      double[] angles = new double[4];
      for (int i = 0; i < 4; i++) {
        distances[i] = after.distanceMeters(i);
        angles[i] = after.angleRadians(i);
      }
      distances[2] = Double.NaN;
      OdometrySample silent = new OdometrySample(1, distances, angles);

      BodyMotion found = swerve.motion(before, silent);

      assertThat(found.dxMeters()).isCloseTo(motion[0], within(1e-12));
      assertThat(found.dyMeters()).isCloseTo(motion[1], within(1e-12));
      assertThat(found.dthetaRadians()).isCloseTo(motion[2], within(1e-12));
      assertThat(found.residualMeters()).isCloseTo(0.0, within(1e-12));
    }
  }

  /**
   * With fewer than two modules reporting, nothing fixes the motion: none, as doubtful as it is.
   */
  @Test
  void oneModuleAloneFixesNoMotion() {
    OdometrySample after = afterRigidMotion(0.1, 0, 0);
    double nan = Double.NaN;
    OdometrySample oneLeft =
        new OdometrySample(
            1,
            new double[] {after.distanceMeters(0), nan, nan, nan},
            new double[] {after.angleRadians(0), nan, nan, nan});

    BodyMotion found = swerve.motion(startingFrom(after), oneLeft);

    assertThat(found.dxMeters()).isZero();
    assertThat(found.dthetaRadians()).isZero();
    assertThat(found.residualMeters()).isCloseTo(0.1, within(1e-12));
  }

  @Test
  void swerveNeedsModulesThatCanFixAMotion() {
    assertThatThrownBy(() -> new SwerveKinematicsModel(new Translation2d(0.3, 0)))
        .hasMessageContaining("at least 2");
    assertThatThrownBy(
            () -> new SwerveKinematicsModel(new Translation2d(0.3, 0), new Translation2d(0.3, 0)))
        .hasMessageContaining("one point");
    OdometrySample two = new OdometrySample(0, new double[2], new double[2]);
    assertThatThrownBy(() -> swerve.motion(two, two)).hasMessageContaining("for 4 modules");
  }

  @Test
  void aSingularMatrixHasNoInverse() {
    double[] out = new double[9];

    assertThat(Matrices.invert3(new double[] {1, 2, 3, 2, 4, 6, 0, 0, 1}, out)).isFalse();
    assertThat(Matrices.invert3(new double[] {2, 0, 0, 0, 4, 0, 0, 0, 5}, out)).isTrue();
    assertThat(out[4]).isEqualTo(0.25);
  }
}
