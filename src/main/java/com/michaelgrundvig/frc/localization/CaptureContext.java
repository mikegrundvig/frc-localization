package com.michaelgrundvig.frc.localization;

import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The robot at the moment a camera took its image, as the vision pipeline needs it.
 *
 * @param heading the robot's heading then, if the estimator knows it: not while it's a driver's
 *     reset no camera has backed
 * @param headingStdDevRadians how sure the estimator is of that heading
 * @param pitchRadians the robot's pitch then, from the IMU: zero when level
 * @param rollRadians its roll
 * @param speedMetersPerSecond how fast it was moving
 * @param recentYawRateRadiansPerSecond the fastest it had turned lately, in either direction
 */
public record CaptureContext(
    @Nullable Rotation2d heading,
    double headingStdDevRadians,
    double pitchRadians,
    double rollRadians,
    double speedMetersPerSecond,
    double recentYawRateRadiansPerSecond) {}
