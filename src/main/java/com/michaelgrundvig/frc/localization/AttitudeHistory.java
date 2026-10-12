package com.michaelgrundvig.frc.localization;

/**
 * The robot's recent pitch and roll, for looking up its attitude when a camera image was taken. On
 * a bump the attitude changes within a tenth of a second, which is about as long as an image takes
 * to arrive.
 *
 * <p>Between two readings the attitude is interpolated. Before the oldest reading kept it is the
 * oldest, and after the newest it is the newest.
 *
 * <p>The readings are kept in a ring: three arrays of a fixed length, where the newest reading
 * overwrites the oldest once the ring is full.
 */
public final class AttitudeHistory {
  private final long[] timesNanos;
  private final double[] pitchesRadians;
  private final double[] rollsRadians;

  /** The slot the next reading goes into. */
  private int next;

  /** How many readings are kept so far, which is the ring's length once it is full. */
  private int size;

  /** The robot's attitude at a moment, in radians, as {@link CaptureContext} takes it. */
  public record Attitude(double pitchRadians, double rollRadians) {
    public static final Attitude LEVEL = new Attitude(0, 0);
  }

  /**
   * @param capacity how many readings to keep. 25 is half a second at 50 Hz.
   */
  public AttitudeHistory(int capacity) {
    if (capacity < 2) {
      throw new IllegalArgumentException("keep at least two readings: " + capacity);
    }
    timesNanos = new long[capacity];
    pitchesRadians = new double[capacity];
    rollsRadians = new double[capacity];
  }

  /** A reading, newer than the last. An older or equal one is ignored. */
  public void add(long timestampNanos, double pitchRadians, double rollRadians) {
    boolean newerThanTheLast = size == 0 || timestampNanos > timesNanos[slotBack(0)];
    if (!newerThanTheLast) {
      return;
    }
    timesNanos[next] = timestampNanos;
    pitchesRadians[next] = pitchRadians;
    rollsRadians[next] = rollRadians;
    next = (next + 1) % timesNanos.length;
    size = Math.min(size + 1, timesNanos.length);
  }

  /** The attitude at a moment. It is level before any reading. */
  public Attitude at(long timestampNanos) {
    if (size == 0) {
      return Attitude.LEVEL;
    }
    // Walk back from the newest reading to the latest one taken at or before the moment. The
    // reading passed just before it is the one taken after the moment.
    int after = slotBack(0);
    for (int back = 0; back < size; back++) {
      int atOrBefore = slotBack(back);
      if (timesNanos[atOrBefore] <= timestampNanos) {
        boolean isTheNewest = back == 0;
        if (isTheNewest) {
          return reading(atOrBefore);
        }
        return between(atOrBefore, after, timestampNanos);
      }
      after = atOrBefore;
    }
    // Every reading kept was taken after the moment, and the walk ended on the oldest.
    return reading(after);
  }

  /** The slot of the reading this many readings back from the newest. */
  private int slotBack(int back) {
    return Math.floorMod(next - 1 - back, timesNanos.length);
  }

  private Attitude reading(int slot) {
    return new Attitude(pitchesRadians[slot], rollsRadians[slot]);
  }

  /**
   * The attitude at a moment between two readings, on the straight line from the earlier to the
   * later:
   *
   * <pre>
   * fraction = (moment - earlier time) / (later time - earlier time)
   * attitude = earlier + fraction × (later - earlier)
   * </pre>
   */
  private Attitude between(int earlier, int later, long timestampNanos) {
    double fraction =
        (double) (timestampNanos - timesNanos[earlier]) / (timesNanos[later] - timesNanos[earlier]);
    return new Attitude(
        pitchesRadians[earlier] + fraction * (pitchesRadians[later] - pitchesRadians[earlier]),
        rollsRadians[earlier] + fraction * (rollsRadians[later] - rollsRadians[earlier]));
  }
}
