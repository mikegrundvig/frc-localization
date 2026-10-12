package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The filter against a world that errs exactly as it claims: every error it considers drawn at the
 * spread it considers, every image's noise at the spread the image claims. Its NEES must then
 * average its number of dimensions, 2 for position and 1 for heading, in each part of the run.
 *
 * <p>It's a check on the whole, not on each step: a mistake in how the estimate relates to one
 * considered error can be too small, or average out over the worlds, to move a mean NEES, so this
 * misses some that {@link PoseFilterReferenceTest}, which checks every step against the textbook,
 * catches. What it adds is the other side: that the model the filter's arithmetic carries out, the
 * errors it considers and how each enters a measurement, is one a world can really err by.
 *
 * <p>The world: a ring of tags around the robot, each off its layout, four cameras on mounts turned
 * and shifted a centimeter or so, two of them seeing the same tags, and a gyro whose scale is off.
 * The images are quiet beside those, so what repeats is most of what's left.
 */
class PoseFilterMonteCarloTest {
  private static final double DT = 0.02;
  private static final double SCALE = 1.0 / 360;
  private static final double TURN = Math.toRadians(0.25);
  private static final double SHIFT = 0.01;
  private static final double HEADING_SHIFT = Math.toRadians(2);
  private static final double TAG = 0.01;
  private static final double SHIFT_METERS = 1.0;
  private static final double SHIFT_RADIANS = Math.toRadians(45);
  private static final double NOISE = 0.005;
  private static final double HEADING_NOISE = Math.toRadians(3);
  private static final double TRANSLATION_PER_RADIAN = 0.02;
  private static final double TRANSLATION_PER_METER = 0.03;
  private static final double HEADING_PER_RADIAN = Math.toRadians(0.1) / Math.sqrt(2 * Math.PI);

  /** Where each camera looks, from the robot's heading: the two at the back see the same tags. */
  private static final double[] LOOKS = {Math.toRadians(23), Math.toRadians(-23), Math.PI, Math.PI};

  /** How far to either side of where it looks a camera sees a tag. */
  private static final double VIEW = Math.toRadians(40);

  /** The tags: a ring about where the robot stands, every 45°, at these distances. */
  private static final double[] TAG_DISTANCES = {3.5, 2.5, 4.0, 3.0, 3.5, 2.0, 4.5, 3.0};

  private static final double CENTER_X = 3;
  private static final double CENTER_Y = 4;

  private static final PoseFilter.Considered CONSIDERED =
      new PoseFilter.Considered(
          SCALE, 2 * SCALE, TURN, SHIFT, HEADING_SHIFT, TAG, SHIFT_METERS, SHIFT_RADIANS);

  /** One world: the truth, what it drew, and the filter following it. */
  private static final class World {
    final SplittableRandom random;
    final PoseFilter filter = new PoseFilter(CONSIDERED, TAG_DISTANCES.length);
    double x;
    double y;
    double heading;
    double yaw; // the gyro's
    final double scale;
    final double[] turn = new double[LOOKS.length];
    final double[][] shift = new double[LOOKS.length][2];
    final double[] headingShift = new double[LOOKS.length];
    final double[][] tags = new double[TAG_DISTANCES.length][2]; // where each really is
    final double[][] layout = new double[TAG_DISTANCES.length][2]; // where the layout says

    World(long seed) {
      random = new SplittableRandom(seed);
      scale = SCALE * random.nextGaussian();
      for (int c = 0; c < LOOKS.length; c++) {
        turn[c] = TURN * random.nextGaussian();
        shift[c][0] = SHIFT * random.nextGaussian();
        shift[c][1] = SHIFT * random.nextGaussian();
        headingShift[c] = HEADING_SHIFT * random.nextGaussian();
      }
      for (int t = 0; t < TAG_DISTANCES.length; t++) {
        double angle = t * Math.PI / 4;
        layout[t][0] = CENTER_X + TAG_DISTANCES[t] * Math.cos(angle);
        layout[t][1] = CENTER_Y + TAG_DISTANCES[t] * Math.sin(angle);
        tags[t][0] = layout[t][0] + TAG * random.nextGaussian();
        tags[t][1] = layout[t][1] + TAG * random.nextGaussian();
      }
      double placed = 0.05;
      double placedHeading = Math.toRadians(2);
      x = CENTER_X + placed * random.nextGaussian();
      y = CENTER_Y + placed * random.nextGaussian();
      heading = placedHeading * random.nextGaussian();
      filter.set(
          new PoseReset(
              0,
              new Pose2d(CENTER_X, CENTER_Y, Rotation2d.ZERO),
              PoseCovariance.fromStdDevs(placed, placed, placedHeading),
              "placed"),
          yaw);
    }

    /**
     * One loop: driving straight ahead at this speed, or turning in place at this rate, then every
     * camera's image, with its heading only while the robot stands.
     */
    void loop(double speed, double rate) {
      double forward = speed * DT;
      double trueTurn = rate * DT;
      double read = (1 + scale) * trueTurn;
      double positionNoise =
          TRANSLATION_PER_RADIAN * TRANSLATION_PER_RADIAN * Math.abs(read)
              + TRANSLATION_PER_METER * TRANSLATION_PER_METER * Math.abs(forward);
      double headingNoise = HEADING_PER_RADIAN * HEADING_PER_RADIAN * Math.abs(read);
      double turned = read + Math.sqrt(headingNoise) * random.nextGaussian();
      filter.predict(yaw, turned, forward, 0, positionNoise, headingNoise, YawSource.GYRO);
      yaw += turned;
      // The wheels' travel, along the true heading: off by the odometry's own error.
      x += Math.cos(heading) * forward + Math.sqrt(positionNoise) * random.nextGaussian();
      y += Math.sin(heading) * forward + Math.sqrt(positionNoise) * random.nextGaussian();
      heading += trueTurn;
      // What a camera's headings share fades as it moves and turns, to a fresh one.
      double kept = Math.exp(-Math.abs(forward) / SHIFT_METERS - Math.abs(turned) / SHIFT_RADIANS);
      for (int c = 0; c < LOOKS.length; c++) {
        headingShift[c] =
            kept * headingShift[c]
                + Math.sqrt(1 - kept * kept) * HEADING_SHIFT * random.nextGaussian();
      }
      boolean standing = speed == 0 && rate == 0;
      for (int c = 0; c < LOOKS.length; c++) {
        image(c, standing);
      }
    }

    /**
     * A camera's image: each tag it sees placed through the estimate's heading, which the camera's
     * own turn turns, and through its mount's shift; the tags' positions averaged by their
     * distances, as the vision pipeline does.
     */
    private void image(int camera, boolean withHeading) {
      double estimated = yaw + filter.headingOffset();
      double sumX = 0;
      double sumY = 0;
      double leverX = 0;
      double leverY = 0;
      double weights = 0;
      List<Integer> seen = new ArrayList<>();
      List<Double> shares = new ArrayList<>();
      for (int t = 0; t < tags.length; t++) {
        double toTagX = tags[t][0] - x;
        double toTagY = tags[t][1] - y;
        double bearing = MathUtil.angleModulus(Math.atan2(toTagY, toTagX) - heading);
        if (Math.abs(MathUtil.angleModulus(bearing - LOOKS[camera])) > VIEW) {
          continue;
        }
        // The tag as the robot sees it, in its own frame, through the camera's turn.
        double robotX = Math.cos(-heading) * toTagX - Math.sin(-heading) * toTagY;
        double robotY = Math.sin(-heading) * toTagX + Math.cos(-heading) * toTagY;
        double solvedWith = estimated - turn[camera];
        double fixX =
            layout[t][0] - (Math.cos(solvedWith) * robotX - Math.sin(solvedWith) * robotY);
        double fixY =
            layout[t][1] - (Math.sin(solvedWith) * robotX + Math.cos(solvedWith) * robotY);
        double distanceSquared = toTagX * toTagX + toTagY * toTagY;
        double weight = 1 / distanceSquared;
        sumX += weight * fixX;
        sumY += weight * fixY;
        leverX += weight * (layout[t][0] - fixX);
        leverY += weight * (layout[t][1] - fixY);
        weights += weight;
        seen.add(t);
        shares.add(weight);
      }
      if (seen.isEmpty()) {
        return;
      }
      // The mount's shift moves the camera's positions along the robot's forward and left.
      double fixX =
          sumX / weights
              + Math.cos(heading) * shift[camera][0]
              - Math.sin(heading) * shift[camera][1];
      double fixY =
          sumY / weights
              + Math.sin(heading) * shift[camera][0]
              + Math.cos(heading) * shift[camera][1];
      fixX += NOISE * random.nextGaussian();
      fixY += NOISE * random.nextGaussian();
      double measuredHeading =
          withHeading
              ? heading
                  + turn[camera]
                  + headingShift[camera]
                  + HEADING_NOISE * random.nextGaussian()
              : estimated;
      VisionMeasurement measurement =
          new VisionMeasurement(
              1,
              new Pose2d(fixX, fixY, new Rotation2d(measuredHeading)),
              PoseCovariance.fromStdDevs(
                  NOISE, NOISE, withHeading ? HEADING_NOISE : Double.POSITIVE_INFINITY),
              new VisionMeasurement.Lever(
                  new Rotation2d(estimated), new Translation2d(leverX / weights, leverY / weights)),
              "camera " + camera,
              seen,
              shares,
              VisionMeasurement.Solver.HEADING_CONSTRAINED);
      int[] slots = seen.stream().mapToInt(Integer::intValue).toArray();
      filter.correct(Objects.requireNonNull(filter.innovation(measurement, yaw, camera, slots)));
    }

    double positionNees() {
      PoseCovariance p = filter.covariance();
      double ex = filter.x() - x;
      double ey = filter.y() - y;
      double determinant = p.xx() * p.yy() - p.xy() * p.xy();
      return (p.yy() * ex * ex - 2 * p.xy() * ex * ey + p.xx() * ey * ey) / determinant;
    }

    double headingNees() {
      double error = MathUtil.angleModulus(yaw + filter.headingOffset() - heading);
      return error * error / filter.covariance().heading();
    }
  }

  /**
   * Standing, driving out a meter and a half and back, standing, two turns in place on the gyro,
   * and standing again: each camera's turn and shifts, each tag's placement, and the gyro's scale,
   * as considered.
   */
  @Test
  void standingDrivingAndTurningItsNeesAveragesItsDimensions() {
    // Each world's draws last all its loops, so the worlds are the sample: a thousand put the mean
    // NEES within about 0.06 of 2, and 0.05 of 1.
    int worlds = 1000;
    int[] phases = {150, 75, 75, 100, 300, 150};
    double[] speeds = {0, 1, -1, 0, 0, 0};
    double[] rates = {0, 0, 0, 0, 4 * Math.PI / (300 * DT), 0};
    double[][] position = new double[phases.length][2];
    double[][] heading = new double[phases.length][2];
    for (int w = 0; w < worlds; w++) {
      World world = new World(w * 31L + 7);
      for (int phase = 0; phase < phases.length; phase++) {
        for (int i = 0; i < phases[phase]; i++) {
          world.loop(speeds[phase], rates[phase]);
          position[phase][0] += world.positionNees();
          position[phase][1]++;
          heading[phase][0] += world.headingNees();
          heading[phase][1]++;
        }
      }
    }
    for (int phase = 0; phase < phases.length; phase++) {
      double p = position[phase][0] / position[phase][1];
      double h = heading[phase][0] / heading[phase][1];
      assertThat(p).as("position NEES, phase %d", phase).isBetween(1.7, 2.3);
      assertThat(h).as("heading NEES, phase %d", phase).isBetween(0.8, 1.2);
    }
  }
}
