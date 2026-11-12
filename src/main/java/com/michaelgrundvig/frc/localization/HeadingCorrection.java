package com.michaelgrundvig.frc.localization;

/**
 * What corrects the heading beside the gyro. A camera's position solved with the gyro's heading
 * leans on it, and so says something about the heading too; whether that's let correct the heading
 * is the choice. Each is honest about its uncertainty; they differ in what they trust the cameras
 * with.
 *
 * <p>The default localizer runs {@link #EVERY_FIX}, which is what lets the cameras find the gyro's
 * errors. The others are for the consistency study, which compares the three (docs/testing.md): no
 * robot's configuration chooses one.
 */
public enum HeadingCorrection {
  /**
   * Only measurements with a heading of their own (multi-tag solves, under strict conditions), with
   * all three of their rows: a position solved with the gyro's heading carries the heading's doubt
   * but never turns it, so the gyro owns heading. Honest on a calibrated robot; a camera turned on
   * its mount turns the heading with its own headings, and the mount check then doubts the cameras
   * that disagree with it.
   */
  HEADING_MEASUREMENTS,
  /**
   * As {@link #HEADING_MEASUREMENTS}, and a camera the mount check doubts has that doubt added to
   * the errors it repeats in every image (its turn and shift), so its headings can pull the heading
   * only so far, however many there are. Only as good as the mount check's verdict: with a camera
   * turned, much of the doubt lands on its neighbour too (docs/testing.md).
   */
  HEADING_MEASUREMENTS_AND_MOUNT_DOUBT,
  /**
   * Every fix: a position fix corrects the heading too, through how far its tags are and how the
   * position's error has come to depend on the heading's. A full Kalman update: more accurate in
   * heading after turns. With a camera turned on its mount, its positions turn the heading, and the
   * mount check then doubts its healthy neighbour more than it. It leans on the images' capture
   * times while the robot turns: an image stamped a few milliseconds late shows the robot turned a
   * little less, as a heading error would, the same way in every image, so check them
   * (docs/testing.md, docs/characterization.md).
   */
  EVERY_FIX
}
