package com.michaelgrundvig.frc.localization.calibration;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * A camera knocked off its mount, found again from the tags it sees: synthetic sightings of 2026's
 * field, with the noise a camera adds.
 */
class CameraMountCalibrationTest {
  private static final FieldModel FIELD =
      new FieldModel(
          Fields.FRC_2026_REBUILT_WELDED.loadField(),
          Symmetry.ROTATED,
          List.of(),
          List.of(),
          List.of());

  /** Forward, 12 in up, tilted up 15°: as configured. */
  private static final Transform3d CONFIGURED =
      new Transform3d(new Translation3d(0.3, 0.1, 0.3), new Rotation3d(0, Math.toRadians(-15), 0));

  /** Really 4 in lower, and turned 2° to the left. */
  private static final Transform3d TRUE =
      new Transform3d(
          new Translation3d(0.3, 0.1, 0.3 - 0.1016),
          new Rotation3d(0, Math.toRadians(-15), Math.toRadians(2)));

  /**
   * The camera's sightings of the blue hub's tags from spots between the given distances, facing
   * the hub within 20°, with a camera's noise: millimeters across the line of sight, more along it
   * (growing with distance squared), and a degree or so of the tag's orientation.
   */
  private static CameraMountCalibration sightings(
      double nearest, double farthest, int spots, double noise, long seed) {
    CameraMountCalibration calibration = new CameraMountCalibration(CONFIGURED);
    Random random = new Random(seed);
    Pose3d hub = FIELD.tag(26).orElseThrow();
    for (int spot = 0; spot < spots; spot++) {
      double distance = nearest + (farthest - nearest) * random.nextDouble();
      double across = (random.nextDouble() - 0.5) * 1.5;
      double heading = Math.toRadians((random.nextDouble() - 0.5) * 40);
      Pose3d robot =
          new Pose3d(
              new Translation3d(hub.getX() - distance, hub.getY() + across, 0),
              new Rotation3d(0, 0, heading));
      for (int id : new int[] {18, 21, 24, 25, 26, 27}) {
        Pose3d tag = FIELD.tag(id).orElseThrow();
        Transform3d view = new Transform3d(robot.transformBy(TRUE), tag);
        Translation3d t = view.getTranslation();
        if (t.getX() < 0.5 || Math.atan2(Math.hypot(t.getY(), t.getZ()), t.getX()) > 0.7) {
          continue; // behind the camera, or outside its view
        }
        double range = t.getNorm();
        Translation3d ray = t.div(range);
        Translation3d wobble =
            new Translation3d(
                    noise * 0.2 * range * random.nextGaussian(),
                    noise * 0.2 * range * random.nextGaussian(),
                    noise * 0.2 * range * random.nextGaussian())
                .plus(ray.times(noise * 0.4 * range * range * random.nextGaussian()));
        Transform3d noisy =
            new Transform3d(
                t.plus(wobble),
                view.getRotation()
                    .rotateBy(
                        new Rotation3d(
                            noise * 2 * range * random.nextGaussian(),
                            noise * 2 * range * random.nextGaussian(),
                            noise * 2 * range * random.nextGaussian())));
        calibration.add(robot, noisy, Transform3d.ZERO, tag);
      }
    }
    return calibration;
  }

  @Test
  void aCameraKnockedLowAndTurnedIsFoundFromVariedSightings() {
    CameraMountCalibration calibration = sightings(1.5, 5, 40, 0.01, 1);

    CameraMountCalibration.Result result = calibration.solve().orElseThrow();

    Transform3d found = result.robotToCamera();
    assertThat(found.getTranslation().getDistance(TRUE.getTranslation())).isLessThan(0.01);
    assertThat(Math.toDegrees(found.getRotation().getZ())).isCloseTo(2, within(0.2));
    assertThat(Math.toDegrees(found.getRotation().getY())).isCloseTo(-15, within(0.2));
    assertThat(result.residualBeforeMeters()).isGreaterThan(0.1);
    assertThat(result.residualAfterMeters()).isLessThan(0.025);
    assertThat(result.stdDevs()[2])
        .as("sure of its height, to a few millimeters")
        .isLessThan(0.005);
  }

  @Test
  void sightingsFromOneDistanceLeaveItUnsure() {
    CameraMountCalibration.Result varied = sightings(1.5, 5, 40, 0.01, 2).solve().orElseThrow();
    CameraMountCalibration.Result oneDistance =
        sightings(3.0, 3.05, 40, 0.01, 2).solve().orElseThrow();

    // Height and tilt trade off when every tag is the same distance away.
    assertThat(oneDistance.stdDevs()[2]).isGreaterThan(2 * varied.stdDevs()[2]);
  }

  @Test
  void ofATagsTwoPosesItKeepsTheOneThatFits() {
    CameraMountCalibration calibration = new CameraMountCalibration(CONFIGURED);
    Pose3d tag = FIELD.tag(26).orElseThrow();
    for (double distance : new double[] {1.5, 2.5, 3.5, 4.5}) {
      Pose3d robot =
          new Pose3d(new Translation3d(tag.getX() - distance, tag.getY(), 0), new Rotation3d());
      Transform3d right = new Transform3d(robot.transformBy(CONFIGURED), tag);
      Transform3d wrong =
          new Transform3d(
              right.getTranslation().plus(new Translation3d(0, 0.4, 0)), right.getRotation());
      calibration.add(robot, wrong, right, tag);
    }

    CameraMountCalibration.Result result = calibration.solve().orElseThrow();

    assertThat(result.residualBeforeMeters()).isLessThan(1e-6);
    assertThat(result.robotToCamera().getTranslation().getDistance(CONFIGURED.getTranslation()))
        .isLessThan(1e-6);
  }

  @Test
  void itNeedsAFewSightings() {
    CameraMountCalibration calibration = new CameraMountCalibration(CONFIGURED);
    Pose3d tag = FIELD.tag(26).orElseThrow();
    calibration.add(new Pose3d(), new Transform3d(new Pose3d(), tag), Transform3d.ZERO, tag);

    assertThat(calibration.solve()).isEmpty();
    assertThat(calibration.sightings()).isEqualTo(1);
  }
}
