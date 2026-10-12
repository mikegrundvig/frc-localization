package com.michaelgrundvig.frc.localization.calibration;

import org.wpilib.math.geometry.Rotation2d;

/**
 * The average of headings, taken around the circle, so that 179° and -179° average to 180° and not
 * to 0°. It gives a reference heading from many camera measurements while the robot stands still.
 *
 * <p>Each heading is taken as a point on the unit circle, and the average heading is the direction
 * of the points' sum:
 *
 * <pre>average = atan2(sum of sines, sum of cosines)</pre>
 */
public final class HeadingAverage {
  private double sumCos;
  private double sumSin;
  private int count;

  public void add(Rotation2d heading) {
    sumCos += heading.getCos();
    sumSin += heading.getSin();
    count++;
  }

  public int count() {
    return count;
  }

  /** The average heading, or zero with nothing added. */
  public Rotation2d average() {
    if (count == 0) {
      return Rotation2d.ZERO;
    }
    return new Rotation2d(Math.atan2(sumSin, sumCos));
  }

  /**
   * How tightly the headings agree, from 0 (spread evenly around) to 1 (all the same). It is the
   * length of the points' average, and 0 with nothing added.
   */
  public double agreement() {
    if (count == 0) {
      return 0;
    }
    return Math.hypot(sumCos, sumSin) / count;
  }

  public void clear() {
    sumCos = 0;
    sumSin = 0;
    count = 0;
  }
}
