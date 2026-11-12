package com.michaelgrundvig.frc.localization;

/**
 * How unsure the filter is of the errors it considers.
 *
 * @param gyroScaleStdDev the gyro's scale error: radians of heading per radian it turns
 * @param backupScaleStdDev the backup IMU's
 * @param cameraTurnStdDevRadians each camera's turn on its mount, as unsure as it starts: and for a
 *     camera without a place, as unsure as it is in every image
 * @param cameraShiftStdDevMeters each camera's shift on its mount, forward and left
 * @param cameraShiftStdDevRadians and of its headings
 * @param tagPlacementStdDevMeters each tag's placement, along x and along y
 * @param shiftCorrelationMeters how far the robot moves before a camera's headings' shift is mostly
 *     a new one: what the filter keeps of it falls by a factor of e over this distance
 * @param shiftCorrelationRadians and over this much turning
 */
record ConsideredSizes(
    double gyroScaleStdDev,
    double backupScaleStdDev,
    double cameraTurnStdDevRadians,
    double cameraShiftStdDevMeters,
    double cameraShiftStdDevRadians,
    double tagPlacementStdDevMeters,
    double shiftCorrelationMeters,
    double shiftCorrelationRadians) {}
