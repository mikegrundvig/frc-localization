package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.robot.Joint;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The robot's recent joint positions, for looking up where a camera on a joint was when its image
 * was taken. A turret turns a tenth of a turn in the time an image takes to arrive.
 *
 * <p>Between two readings each position is interpolated. Before the oldest reading kept it is the
 * oldest, and after the newest it is the newest. Before any reading, each joint is at its
 * reference.
 *
 * <p>The readings are kept in a ring: arrays of a fixed length, where the newest reading overwrites
 * the oldest once the ring is full.
 */
public final class JointHistory {
  /** The joints kept, in the order their positions are held in. */
  private final List<Joint> joints;

  /** Each joint's reference, in the joints' order. */
  private final double[] references;

  private final long[] timesNanos;

  /** Each reading's positions, in the joints' order: {@code positions[slot][joint]}. */
  private final double[][] positions;

  /** The slot the next reading goes into. */
  private int next;

  /** How many readings are kept so far, which is the ring's length once it is full. */
  private int size;

  /**
   * @param references the joints kept, each at its reference. They are the robot's joints that move
   *     on their own ({@code RobotMath.references}).
   * @param capacity how many readings to keep. 25 is half a second at 50 Hz.
   */
  public JointHistory(Map<Joint, Double> references, int capacity) {
    if (capacity < 2) {
      throw new IllegalArgumentException("keep at least two readings: " + capacity);
    }
    joints = List.copyOf(references.keySet());
    this.references = new double[joints.size()];
    int j = 0;
    for (double reference : references.values()) {
      this.references[j++] = reference;
    }
    timesNanos = new long[capacity];
    positions = new double[capacity][joints.size()];
  }

  /**
   * A reading, newer than the last. An older or equal one is ignored. A joint the reading doesn't
   * name is where it was in the reading before, or at its reference if there is none before.
   */
  public void add(long timestampNanos, Map<Joint, Double> read) {
    boolean newerThanTheLast = size == 0 || timestampNanos > timesNanos[slotBack(0)];
    if (!newerThanTheLast) {
      return;
    }
    double[] before = references;
    if (size > 0) {
      before = positions[slotBack(0)];
    }
    for (int j = 0; j < joints.size(); j++) {
      // The map answers null for a joint the reading doesn't name.
      Double position = read.get(joints.get(j));
      if (position != null) {
        positions[next][j] = position;
      } else {
        positions[next][j] = before[j];
      }
    }
    timesNanos[next] = timestampNanos;
    next = (next + 1) % timesNanos.length;
    size = Math.min(size + 1, timesNanos.length);
  }

  /** Every kept joint's position at a moment. */
  public Map<Joint, Double> at(long timestampNanos) {
    if (size == 0) {
      return named(references);
    }
    // Walk back from the newest reading to the latest one taken at or before the moment. The
    // reading passed just before it is the one taken after the moment.
    int after = slotBack(0);
    for (int back = 0; back < size; back++) {
      int atOrBefore = slotBack(back);
      if (timesNanos[atOrBefore] <= timestampNanos) {
        boolean isTheNewest = back == 0;
        if (isTheNewest) {
          return named(positions[atOrBefore]);
        }
        return between(atOrBefore, after, timestampNanos);
      }
      after = atOrBefore;
    }
    // Every reading kept was taken after the moment, and the walk ended on the oldest.
    return named(positions[after]);
  }

  /** The slot of the reading this many readings back from the newest. */
  private int slotBack(int back) {
    return Math.floorMod(next - 1 - back, timesNanos.length);
  }

  /** Positions held in the joints' order, as a map by joint. */
  private Map<Joint, Double> named(double[] held) {
    return between(held, held, 0);
  }

  private Map<Joint, Double> between(int earlier, int later, long timestampNanos) {
    double fraction =
        (double) (timestampNanos - timesNanos[earlier]) / (timesNanos[later] - timesNanos[earlier]);
    return between(positions[earlier], positions[later], fraction);
  }

  /**
   * Each joint's position part of the way from one reading to a later one, on the straight line
   * between them:
   *
   * <pre>position = earlier + fraction × (later - earlier)</pre>
   */
  private Map<Joint, Double> between(double[] earlier, double[] later, double fraction) {
    Map<Joint, Double> at = new LinkedHashMap<>();
    for (int j = 0; j < joints.size(); j++) {
      at.put(joints.get(j), earlier[j] + fraction * (later[j] - earlier[j]));
    }
    return at;
  }
}
