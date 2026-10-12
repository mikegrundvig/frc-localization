package com.michaelgrundvig.frc.localization;

/**
 * A camera's doubt about its mount, as the robot's mount check last reported it ({@link
 * MountCheck}).
 *
 * @param meters how far the camera's measurements may be off from where its mount moved
 * @param tagDistanceMeters how far its tags were, on average, when that was judged; NaN until it
 *     has been judged
 */
record MountDoubt(double meters, double tagDistanceMeters) {}
