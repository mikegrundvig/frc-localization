package com.michaelgrundvig.frc.localization;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.Arrays;
import java.util.List;

/**
 * Cameras for the estimator's tests, known by name, where only how far each one's mount may be off
 * matters: the estimator is fed measurements by hand, so where a camera sits and what it sees are
 * the test's own.
 */
final class TestCameras {
  private TestCameras() {}

  /** A mount known exactly: nothing it repeats from image to image is its mount's. */
  static final CameraConfig.MountTolerance EXACT =
      CameraConfig.MountTolerance.of(Meters.of(0), Degrees.of(0));

  /** Cameras by these names, each mounted exactly. */
  static List<CameraConfig> exact(String... names) {
    return mounted(EXACT, names);
  }

  /** Cameras by these names, each mount as far off as the tolerance allows. */
  static List<CameraConfig> mounted(CameraConfig.MountTolerance tolerance, String... names) {
    return Arrays.stream(names).map(name -> camera(name, tolerance)).toList();
  }

  /** A camera by this name, its mount as far off as the tolerance allows. */
  static CameraConfig camera(String name, CameraConfig.MountTolerance tolerance) {
    return CameraConfig.of(
            name,
            CameraConfig.ImageSensor.of(1280, 800, Hertz.of(30))
                .fieldOfView(Degrees.of(80), CameraConfig.Across.DIAGONAL)
                .latency(Milliseconds.of(30))
                .sightRange(Meters.of(7)))
        .at(Meters.of(0.3), Meters.of(0), Meters.of(0.3))
        .facing(Degrees.of(15), Degrees.of(0))
        .roll(Degrees.of(0))
        .noiseScale(1.0)
        .mountTolerance(tolerance);
  }
}
