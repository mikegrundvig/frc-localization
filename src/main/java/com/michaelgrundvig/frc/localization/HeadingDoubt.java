package com.michaelgrundvig.frc.localization;

/**
 * Whether the estimate's heading rests on a person's word that no camera has backed yet, and whose.
 * Until a camera backs it, no image is solved with it ({@link Localizer#headingAt}): solved with a
 * wrong heading, every image lands off alike, and could never say the heading is wrong.
 */
public enum HeadingDoubt {
  /** Backed by a camera, worked out by one, or never given by a person. */
  NONE,
  /**
   * The robot was placed, on its start or anywhere else, and no camera has backed the heading it
   * was placed facing: a robot set down crooked or backwards believes it all the same.
   */
  PLACEMENT,
  /** A driver reset the heading, and no camera has backed it yet. */
  RESET
}
