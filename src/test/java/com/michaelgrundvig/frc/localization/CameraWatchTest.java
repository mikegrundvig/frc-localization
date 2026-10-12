package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;

/**
 * The watch on the cameras, fed by hand: misses in the robot's frame, as a robot turning on the
 * spot would see them, an image from every camera each 40 ms.
 */
class CameraWatchTest {
  /** Each image's own noise along each axis: half a centimeter, as a standard deviation. */
  private static final double NOISE = 0.005;

  private static final double TAGS_AT_METERS = 2.5;

  /** What each image claims of its position, as a variance along each axis: 2 cm. */
  private static final double CLAIMED = 0.02 * 0.02;

  private static final long IMAGE_NANOS = 40_000_000L;

  /** A watch being fed, and the time on its robot's clock. */
  private static final class Watched {
    // A calibrated camera's mount is allowed 0.02° of turn, about 1 mm at these tags, and 1 mm of
    // shift: a camera is doubted for what is past three times those together, 4 mm.
    final CameraWatch watch = new CameraWatch(50, Math.toRadians(0.02), 0.001, 0.001);
    final SplittableRandom random = new SplittableRandom(5);

    /** How unsure odometry's place is, as a variance: as sure as one camera, unless set. */
    double odometryVariance = CLAIMED;

    long nanos = 1_000_000_000L;
    int image;

    /**
     * Feeds some cameras one image each, the robot turning a little between images.
     *
     * @param misses for each camera, by place: its steady miss forward and to the left, in the
     *     robot's frame, in meters; {@link #BLIND} for a camera that sees no tag
     */
    Watched see(double[]... misses) {
      nanos += IMAGE_NANOS;
      double heading = 0.1 * image;
      image++;
      for (int camera = 0; camera < misses.length; camera++) {
        if (misses[camera].length != 0) {
          double forward = misses[camera][0] + NOISE * random.nextGaussian();
          double left = misses[camera][1] + NOISE * random.nextGaussian();
          double x = Math.cos(heading) * forward - Math.sin(heading) * left;
          double y = Math.sin(heading) * forward + Math.cos(heading) * left;
          watch.accepted(
              camera, nanos, x, y, heading, TAGS_AT_METERS, CLAIMED, odometryVariance, 0, 0);
        }
      }
      return this;
    }

    Watched see(int images, double[]... misses) {
      for (int i = 0; i < images; i++) {
        see(misses);
      }
      return this;
    }
  }

  private static final double[] NONE = {0, 0};

  /** A camera that sees no tag. */
  private static final double[] BLIND = {};

  /**
   * One row: the cameras' steady misses over 100 images, and how far each ends in doubt.
   *
   * @param misses each camera's miss in the robot's frame, forward and left, in meters
   * @param doubts how far each camera is doubted, in meters, to within 5 mm
   */
  private record Row(String what, double[][] misses, double[] doubts) {}

  @Test
  void aCameraIsDoubtedByHowFarItSitsFromTheMiddleOfThemAll() {
    double[] slipping = {0.08, 0};
    Row[] rows = {
      new Row(
          "four that agree", new double[][] {NONE, NONE, NONE, NONE}, new double[] {0, 0, 0, 0}),
      new Row(
          "one of four 5 cm to the left: it alone is doubted",
          new double[][] {{0, 0.05}, NONE, NONE, NONE},
          new double[] {0.05, 0, 0, 0}),
      new Row(
          "one of three 4 cm back and 3 cm left",
          new double[][] {NONE, {-0.04, 0.03}, NONE},
          new double[] {0, 0.05, 0}),
      new Row(
          "every camera off alike, as wheels that slip put them: odometry is outvoted, and none"
              + " is doubted",
          new double[][] {slipping, slipping, slipping, slipping},
          new double[] {0, 0, 0, 0}),
      new Row(
          "two that disagree by 6 cm: the one that agrees with odometry is the sound one",
          new double[][] {{0.06, 0}, NONE},
          new double[] {0.06, 0}),
      new Row(
          "two that disagree while the wheels slip 8 cm: the two outvote odometry on the slip,"
              + " and the one 6 cm from the other is 3 cm from their middle, as the other is",
          new double[][] {{0.14, 0}, slipping},
          new double[] {0.06, 0}),
      new Row(
          "one camera alone has nothing to be judged against",
          new double[][] {{0.2, 0}},
          new double[] {0}),
      new Row(
          "a miss a sound camera could show: 3 mm, where its mount is allowed 4",
          new double[][] {{0.003, 0}, NONE, NONE, NONE},
          new double[] {0, 0, 0, 0}),
    };
    for (Row row : rows) {
      Watched watched = new Watched().see(100, row.misses());
      for (int camera = 0; camera < row.doubts().length; camera++) {
        assertThat(watched.watch.doubtMeters(camera))
            .as("%s: camera %d", row.what(), camera)
            .isCloseTo(row.doubts()[camera], within(0.005));
      }
    }
  }

  /** A knocked camera is doubted within half a second of images, and by most of its miss. */
  @Test
  void aKnockedCameraIsDoubtedWithinHalfASecond() {
    Watched watched = new Watched().see(100, NONE, NONE, NONE);
    assertThat(watched.watch.doubtMeters(0)).isZero();

    watched.see(12, new double[] {0, 0.08}, NONE, NONE);

    // Twelve images of fifty: a fifth of the average is the new miss, 1.7 cm of the 8, less what
    // the scatter of old and new misses together could explain.
    assertThat(watched.watch.doubtMeters(0)).isGreaterThan(0.005);
    assertThat(watched.watch.distanceMeters(0)).isCloseTo(TAGS_AT_METERS, within(1e-9));
    watched.see(100, new double[] {0, 0.08}, NONE, NONE);
    assertThat(watched.watch.doubtMeters(0)).isCloseTo(0.08, within(0.01));
  }

  /** A camera pushed back where it belongs is trusted again, once its latest images say so. */
  @Test
  void aCameraWhoseMissesStopIsNoLongerDoubted() {
    Watched watched = new Watched().see(100, new double[] {0, 0.05}, NONE, NONE);
    assertThat(watched.watch.doubtMeters(0)).isGreaterThan(0.04);

    int imagesUntilTrusted = 0;
    while (watched.watch.doubtMeters(0) > 0) {
      watched.see(NONE, NONE, NONE);
      imagesUntilTrusted++;
    }

    // Each image takes a fiftieth off the old misses' weight, and old and new misses together
    // scatter widely, so the 5 cm is down among the noise after a few dozen images: a second or
    // two of them.
    assertThat(imagesUntilTrusted).isBetween(20, 160);
  }

  /**
   * A camera that stops seeing tags keeps the doubt it had: nothing new says it was pushed back.
   * And it has no voice in a moment it saw nothing of, so its old misses cannot outvote the cameras
   * that still see.
   */
  @Test
  void aCameraThatSeesNothingKeepsItsDoubtAndLosesItsVote() {
    Watched watched = new Watched().see(100, new double[] {0, 0.05}, NONE, NONE, NONE);
    double doubted = watched.watch.doubtMeters(0);
    assertThat(doubted).isCloseTo(0.05, within(0.005));

    // Two seconds in which only the last two cameras see tags, and both miss 4 cm forward, as
    // they would with the wheels slipping: with the others' old misses still voting, they would
    // be the odd ones out.
    double[] slipping = {0.04, 0};
    watched.see(50, BLIND, BLIND, slipping, slipping);

    assertThat(watched.watch.doubtMeters(0)).as("unseen, still doubted").isEqualTo(doubted);
    // Of the two that see, the one a little farther off than the other is that little apart
    // from their middle: a few millimeters, the size of the images' noise.
    assertThat(watched.watch.doubtMeters(2)).as("the two that see agree").isLessThan(0.005);
    assertThat(watched.watch.doubtMeters(3)).isLessThan(0.005);
  }

  /**
   * Each voice counts by how sure it claims to be. Two cameras that each claim little (20 cm, as
   * far from their tags) and are each off their own way do not outvote an odometry that is sure:
   * both are doubted. With the wheels in doubt, odometry counts for little, and the same two
   * cameras off alike are taken at their word.
   */
  @Test
  void theVoicesCountByHowSureTheyClaimToBe() {
    double farCamera = 0.2 * 0.2;
    double[][] eachOffItsOwnWay = {{0.30, 0}, {0, -0.12}};
    double[][] bothOffAlike = {{0.30, 0}, {0.30, 0}};

    CameraWatch sureOdometry = watchingFarCameras(eachOffItsOwnWay, farCamera, 0.03 * 0.03);
    assertThat(sureOdometry.doubtMeters(0)).isCloseTo(0.30, within(0.01));
    assertThat(sureOdometry.doubtMeters(1)).isCloseTo(0.12, within(0.01));

    CameraWatch slippingWheels = watchingFarCameras(bothOffAlike, farCamera, 1.0);
    assertThat(slippingWheels.doubtMeters(0)).isLessThan(0.01);
    assertThat(slippingWheels.doubtMeters(1)).isLessThan(0.01);

    // Odometry as sure as it can be, against the same two: it is the middle, and both are off it.
    CameraWatch atRest = watchingFarCameras(bothOffAlike, farCamera, 0);
    assertThat(atRest.doubtMeters(0)).isCloseTo(0.30, within(0.01));
    assertThat(atRest.doubtMeters(1)).isCloseTo(0.30, within(0.01));
  }

  /** A watch fed 100 images from each of two cameras, each claiming and missing as given. */
  private static CameraWatch watchingFarCameras(
      double[][] misses, double claimedVariance, double odometryVariance) {
    CameraWatch watch = new CameraWatch(50, 0, 0, 0);
    SplittableRandom random = new SplittableRandom(13);
    long nanos = 1_000_000_000L;
    for (int image = 0; image < 100; image++) {
      nanos += IMAGE_NANOS;
      for (int camera = 0; camera < misses.length; camera++) {
        watch.accepted(
            camera,
            nanos,
            misses[camera][0] + NOISE * random.nextGaussian(),
            misses[camera][1] + NOISE * random.nextGaussian(),
            0,
            4,
            claimedVariance,
            odometryVariance,
            0,
            0);
      }
    }
    return watch;
  }

  /**
   * A robot that drives a figure and turns as it goes, three cameras each seeing tags 3 m off its
   * own way. One camera's images are stamped a time late, so each of them misses by that time times
   * its late arm. The others' are stamped rightly.
   *
   * @param lateSeconds how late the first camera's images are stamped
   * @param images how many images each camera takes
   */
  private static CameraWatch watchingARobotThatMoves(double lateSeconds, int images) {
    CameraWatch watch = new CameraWatch(50, Math.toRadians(0.02), 0.001, 0.001);
    SplittableRandom random = new SplittableRandom(9);
    long nanos = 1_000_000_000L;
    for (int image = 0; image < images; image++) {
      nanos += IMAGE_NANOS;
      double heading = 0.05 * image;
      double velocityX = 2 * Math.cos(0.07 * image);
      double velocityY = 2 * Math.sin(0.11 * image);
      double yawRate = 1.5 * Math.sin(0.09 * image);
      for (int camera = 0; camera < 3; camera++) {
        double bearing = heading + 2.1 * camera;
        double leverX = 3 * Math.cos(bearing);
        double leverY = 3 * Math.sin(bearing);
        double lateArmX = CameraWatch.lateArmX(leverY, velocityX, yawRate);
        double lateArmY = CameraWatch.lateArmY(leverX, velocityY, yawRate);
        double late = camera == 0 ? lateSeconds : 0;
        watch.accepted(
            camera,
            nanos,
            late * lateArmX + NOISE * random.nextGaussian(),
            late * lateArmY + NOISE * random.nextGaussian(),
            heading,
            3,
            CLAIMED,
            CLAIMED,
            lateArmX,
            lateArmY);
      }
    }
    return watch;
  }

  /**
   * A camera whose images are stamped 20 ms late is found out by how its misses follow the robot's
   * motion, within a few seconds of driving, and what its lateness puts a measurement off by is
   * what is taken off it. Its mount is not doubted for it.
   */
  @Test
  void aCameraStampedLateIsFoundOutByHowItsMissesFollowTheRobotsMotion() {
    CameraWatch watch = watchingARobotThatMoves(0.020, 150);

    assertThat(watch.lateSeconds(0)).isCloseTo(0.020, within(0.002));
    assertThat(watch.lateSeconds(1)).as("stamped rightly").isCloseTo(0, within(0.001));
    assertThat(watch.lateSeconds(2)).isCloseTo(0, within(0.001));
    assertThat(watch.doubtMeters(0)).as("its mount").isLessThan(0.005);
    // A measurement taken at 2 m/s along x, turning 1 rad/s, its tags 3 m along y: a second of
    // lateness puts it 1 m off along x (the turn's 3, less the travel's 2) and nothing along y.
    double lateArmX = CameraWatch.lateArmX(3, 2, 1);
    double lateArmY = CameraWatch.lateArmY(0, 0, 1);
    assertThat(lateArmX).isCloseTo(1.0, within(1e-12));
    assertThat(lateArmY).isCloseTo(0.0, within(1e-12));
    assertThat(watch.offX(0, 0.4, lateArmX)).isCloseTo(0.020, within(0.004));
    assertThat(watch.offY(0, 0.4, lateArmY)).isCloseTo(0, within(0.004));
    assertThat(watch.offX(1, 0.4, lateArmX))
        .as("a camera stamped rightly")
        .isCloseTo(0, within(0.002));
    assertThat(watch.lateSeconds(ConsideredErrors.NO_PLACE)).isZero();
  }

  /** Stamped early, the lateness is negative; and a robot at rest says nothing of any clock. */
  @Test
  void latenessHasASignAndNeedsMotionToShow() {
    assertThat(watchingARobotThatMoves(-0.030, 150).lateSeconds(0))
        .isCloseTo(-0.030, within(0.003));

    Watched atRest = new Watched().see(150, new double[] {0, 0.05}, NONE, NONE);
    assertThat(atRest.watch.lateSeconds(0)).as("a mount's miss is not a clock's").isZero();
    assertThat(atRest.watch.doubtMeters(0)).isCloseTo(0.05, within(0.005));
  }

  @Test
  void aCameraWithoutAPlaceOrEnoughImagesIsNeverDoubted() {
    Watched watched = new Watched();
    watched.watch.accepted(
        ConsideredErrors.NO_PLACE, watched.nanos, 1, 1, 0, 2, CLAIMED, CLAIMED, 0, 0);
    assertThat(watched.watch.doubtMeters(ConsideredErrors.NO_PLACE)).isZero();
    assertThat(watched.watch.distanceMeters(0)).as("no image yet").isNaN();
    assertThat(watched.watch.distanceMeters(ConsideredErrors.NO_PLACE)).isNaN();

    // Nine images are too few to judge a camera's scatter by; three more, and their fading
    // weights add up to ten.
    watched.see(30, BLIND, NONE, NONE);
    watched.see(9, new double[] {0, 0.5}, NONE, NONE);
    assertThat(watched.watch.doubtMeters(0)).as("nine images").isZero();
    watched.see(3, new double[] {0, 0.5}, NONE, NONE);
    assertThat(watched.watch.doubtMeters(0)).as("twelve images").isCloseTo(0.5, within(0.01));
    watched.see(30, new double[] {0, 0.5});
    assertThat(watched.watch.doubtMeters(0))
        .as("seen alone, it is judged against nothing, and keeps the doubt it had")
        .isCloseTo(0.5, within(0.01));
    assertThatThrownBy(() -> new CameraWatch(19, 0, 0, 0)).hasMessageContaining("at least 20");
  }

  /**
   * The watch's working written out plainly, from every measurement it was ever given: each sum is
   * taken over the whole list, with each measurement's weight the share kept raised to its age. The
   * watch keeps running sums instead, and must come to the same.
   */
  private static final class Plain {
    static final int CAMERAS = 3;
    static final double CLEAR_OF_NOISE = 3;
    static final double FEWEST_TO_JUDGE = 10;
    static final long SAME_MOMENT_NANOS = 100_000_000L;
    static final double LEAST_VARIANCE = 1e-8;
    static final double LATE_VARIANCE_BEFORE = 0.05 * 0.05;

    /** What is kept of one measurement that had another camera to be judged against. */
    record Kept(
        double apartForward,
        double apartLeft,
        double distance,
        double armForward,
        double armLeft) {}

    interface Of {
      double of(Kept kept);
    }

    final double kept;
    final double turnVariance;
    final double shiftVariance;
    final double lateVariance;
    final List<List<Kept>> measurements = new ArrayList<>();
    final long[] heard = new long[CAMERAS];
    final double[] missForward = new double[CAMERAS];
    final double[] missLeft = new double[CAMERAS];
    final double[] missWeight = new double[CAMERAS];
    final double[] fitted = new double[CAMERAS];
    final double[] fittedVariance = new double[CAMERAS];
    final double[] late = new double[CAMERAS];
    final double[] offForward = new double[CAMERAS];
    final double[] offLeft = new double[CAMERAS];

    Plain(int memory, double turn, double shift, double lateStdDev) {
      kept = 1 - 1.0 / memory;
      turnVariance = turn * turn;
      shiftVariance = shift * shift;
      lateVariance = lateStdDev * lateStdDev;
      Arrays.fill(heard, Long.MIN_VALUE / 2);
      for (int camera = 0; camera < CAMERAS; camera++) {
        measurements.add(new ArrayList<>());
      }
    }

    /** The weighted sum of something over a camera's measurements, the newest counting whole. */
    double sum(int camera, Of what) {
      List<Kept> list = measurements.get(camera);
      double sum = 0;
      for (int i = 0; i < list.size(); i++) {
        sum += Math.pow(kept, list.size() - 1 - i) * what.of(list.get(i));
      }
      return sum;
    }

    double weight(int camera) {
      return sum(camera, each -> 1);
    }

    /** The weighted median of the voices of a moment along one axis, odometry's zero among them. */
    double middle(double[] miss, long taken, double odometryWeight) {
      List<double[]> voices = new ArrayList<>();
      voices.add(new double[] {0, odometryWeight});
      for (int camera = 0; camera < CAMERAS; camera++) {
        if (Math.abs(taken - heard[camera]) <= SAME_MOMENT_NANOS) {
          voices.add(new double[] {miss[camera], missWeight[camera]});
        }
      }
      voices.sort((a, b) -> Double.compare(a[0], b[0]));
      double half = voices.stream().mapToDouble(voice -> voice[1]).sum() / 2;
      int lower = 0;
      double below = voices.get(0)[1];
      while (below < half) {
        below += voices.get(++lower)[1];
      }
      int upper = voices.size() - 1;
      double above = voices.get(upper)[1];
      while (above < half) {
        above += voices.get(--upper)[1];
      }
      return (voices.get(lower)[0] + voices.get(upper)[0]) / 2;
    }

    void accepted(
        int camera,
        long taken,
        double missX,
        double missY,
        double heading,
        double distance,
        double claimed,
        double odometryVariance,
        double armX,
        double armY) {
      double cos = Math.cos(heading);
      double sin = Math.sin(heading);
      missForward[camera] = cos * missX + sin * missY;
      missLeft[camera] = cos * missY - sin * missX;
      missWeight[camera] = 1 / Math.max(claimed, LEAST_VARIANCE);
      heard[camera] = taken;
      long voices =
          Arrays.stream(heard).filter(at -> Math.abs(taken - at) <= SAME_MOMENT_NANOS).count();
      if (voices < 2) {
        return;
      }
      double odometryWeight = 1 / Math.max(odometryVariance, LEAST_VARIANCE);
      measurements
          .get(camera)
          .add(
              new Kept(
                  missForward[camera] - middle(missForward, taken, odometryWeight),
                  missLeft[camera] - middle(missLeft, taken, odometryWeight),
                  distance,
                  cos * armX + sin * armY,
                  cos * armY - sin * armX));
      judge(camera);
    }

    void judge(int camera) {
      double n = weight(camera);
      if (n < FEWEST_TO_JUDGE) {
        return;
      }
      double meanForward = sum(camera, Kept::apartForward) / n;
      double meanLeft = sum(camera, Kept::apartLeft) / n;
      double armForward = sum(camera, Kept::armForward) / n;
      double armLeft = sum(camera, Kept::armLeft) / n;
      double armSquared = armForward * armForward + armLeft * armLeft;
      double spread =
          Math.max(
              0,
              sum(camera, k -> k.armForward() * k.armForward() + k.armLeft() * k.armLeft()) / n
                  - armSquared);
      double armWithApart =
          sum(camera, k -> k.armForward() * k.apartForward() + k.armLeft() * k.apartLeft()) / n
              - armForward * meanForward
              - armLeft * meanLeft;
      double line = spread > 0 ? armWithApart / spread : 0;
      double leftOver =
          sum(camera, k -> k.apartForward() * k.apartForward() + k.apartLeft() * k.apartLeft()) / n
              - meanForward * meanForward
              - meanLeft * meanLeft
              - line * line * spread;
      double noise = Math.max(0, leftOver) / 2;
      double before = 1 / LATE_VARIANCE_BEFORE;
      double fromMisses = 0;
      if (n * spread > 0) {
        fromMisses = noise > 0 ? n * spread / noise : Double.POSITIVE_INFINITY;
      }
      fitted[camera] = fromMisses > 0 ? line / (1 + before / fromMisses) : 0;
      fittedVariance[camera] = 1 / (before + fromMisses);
      judgeLateness();

      double forward = meanForward - fitted[camera] * armForward;
      double left = meanLeft - fitted[camera] * armLeft;
      double distance = sum(camera, Kept::distance) / n;
      double sound = noise / n + shiftVariance + turnVariance * distance * distance;
      double armLength = Math.sqrt(armSquared);
      double alongForward = armLength > 0 ? armForward / armLength : 1;
      double alongLeft = armLength > 0 ? armLeft / armLength : 0;
      double along = forward * alongForward + left * alongLeft;
      double acrossForward = forward - along * alongForward;
      double acrossLeft = left - along * alongLeft;
      double alongKept =
          along * clearOf(along * along, sound + fittedVariance[camera] * armSquared);
      double acrossShare = clearOf(acrossForward * acrossForward + acrossLeft * acrossLeft, sound);
      offForward[camera] = alongKept * alongForward + acrossShare * acrossForward;
      offLeft[camera] = alongKept * alongLeft + acrossShare * acrossLeft;
    }

    void judgeLateness() {
      List<Double> judged = new ArrayList<>();
      for (int camera = 0; camera < CAMERAS; camera++) {
        if (weight(camera) >= FEWEST_TO_JUDGE) {
          judged.add(fitted[camera]);
        }
      }
      judged.sort(Double::compare);
      int size = judged.size();
      double middle =
          size % 2 == 1
              ? judged.get(size / 2)
              : (judged.get(size / 2 - 1) + judged.get(size / 2)) / 2;
      for (int camera = 0; camera < CAMERAS; camera++) {
        if (weight(camera) >= FEWEST_TO_JUDGE) {
          double own = fitted[camera] - middle;
          late[camera] = own * clearOf(own * own, fittedVariance[camera] + lateVariance);
        }
      }
    }

    static double clearOf(double squared, double variance) {
      double sound = CLEAR_OF_NOISE * CLEAR_OF_NOISE * variance;
      return squared > sound ? Math.sqrt(1 - sound / squared) : 0;
    }
  }

  /**
   * Three cameras over two minutes of a robot that drives and turns: one knocked 6 cm to its left a
   * third of the way in, one stamped 30 ms late, each missing an image now and then, the wheels
   * sure and unsure by turns. After every measurement the watch says of every camera exactly what
   * its working written out plainly says: how far it is off, how late, how doubted, how far its
   * tags.
   */
  @Test
  void theWatchSaysWhatItsWorkingWrittenOutPlainlySays() {
    double turn = Math.toRadians(0.25);
    CameraWatch watch = new CameraWatch(25, turn, 0.003, 0.003);
    Plain plain = new Plain(25, turn, 0.003, 0.003);
    SplittableRandom random = new SplittableRandom(11);
    long nanos = 1_000_000_000L;
    double heading = 0;
    double mostDoubt = 0;
    double mostLate = 0;
    int compared = 0;

    for (int moment = 0; moment < 900; moment++) {
      // Now and then every camera is blind for a while, and the next images are of a new moment.
      nanos += random.nextInt(20) == 0 ? 400_000_000L : IMAGE_NANOS;
      heading += 0.2 * random.nextGaussian();
      double velocityX = 3 * random.nextGaussian();
      double velocityY = 3 * random.nextGaussian();
      double odometryVariance = Math.pow(10, -6 + 4 * random.nextDouble());
      for (int camera = 0; camera < Plain.CAMERAS; camera++) {
        if (random.nextInt(6) == 0) {
          continue;
        }
        double armX = -velocityX + random.nextGaussian();
        double armY = -velocityY + random.nextGaussian();
        double lateBy = camera == 2 ? 0.030 : 0;
        double knockedLeft = camera == 1 && moment > 300 ? 0.06 : 0;
        double missX =
            -Math.sin(heading) * knockedLeft + lateBy * armX + NOISE * random.nextGaussian();
        double missY =
            Math.cos(heading) * knockedLeft + lateBy * armY + NOISE * random.nextGaussian();
        double distance = 1.5 + 2.5 * random.nextDouble();
        double claimed = 1e-4 + 9e-4 * random.nextDouble();
        long taken = nanos + camera * 1_000_000L;
        watch.accepted(
            camera, taken, missX, missY, heading, distance, claimed, odometryVariance, armX, armY);
        plain.accepted(
            camera, taken, missX, missY, heading, distance, claimed, odometryVariance, armX, armY);

        for (int each = 0; each < Plain.CAMERAS; each++) {
          double cos = Math.cos(heading);
          double sin = Math.sin(heading);
          double offX = cos * plain.offForward[each] - sin * plain.offLeft[each];
          double offY = sin * plain.offForward[each] + cos * plain.offLeft[each];
          assertThat(watch.offX(each, heading, armX))
              .as("moment %d, camera %d: off along x", moment, each)
              .isCloseTo(offX + plain.late[each] * armX, within(1e-9));
          assertThat(watch.offY(each, heading, armY))
              .as("moment %d, camera %d: off along y", moment, each)
              .isCloseTo(offY + plain.late[each] * armY, within(1e-9));
          assertThat(watch.lateSeconds(each)).isCloseTo(plain.late[each], within(1e-10));
          assertThat(watch.doubtMeters(each))
              .isCloseTo(Math.hypot(plain.offForward[each], plain.offLeft[each]), within(1e-9));
          double weight = plain.weight(each);
          if (weight > 0) {
            assertThat(watch.distanceMeters(each))
                .isCloseTo(plain.sum(each, Plain.Kept::distance) / weight, within(1e-9));
          }
          compared++;
        }
        mostDoubt = Math.max(mostDoubt, watch.doubtMeters(1));
        mostLate = Math.max(mostLate, Math.abs(watch.lateSeconds(2)));
      }
    }

    assertThat(compared).isGreaterThan(6000);
    assertThat(mostDoubt).as("the knocked camera was doubted").isGreaterThan(0.04);
    assertThat(mostLate).as("the late camera was found late").isGreaterThan(0.01);
  }

  /** A second of lateness puts a measurement off by (w ly - vx, -w lx - vy). */
  @Test
  void theLateArmIsTheVelocityBackwardAndTheLeverAQuarterTurnBack() {
    assertThat(CameraWatch.lateArmX(2.0, 0.5, 3.0)).isEqualTo(5.5);
    assertThat(CameraWatch.lateArmY(2.0, 0.5, 3.0)).isEqualTo(-6.5);
  }

  /**
   * Cameras are of the same moment when their images were taken within a tenth of a second, and a
   * camera never heard from is of no moment at all, however early the clock.
   */
  @Test
  void camerasAreOfTheSameMomentWithinATenthOfASecond() {
    CameraWatch early = new CameraWatch(20, 0, 0, 0.001);
    early.accepted(0, 50_000_000L, 0.1, 0, 0, 2, CLAIMED, CLAIMED, 0, 0);
    assertThat(early.distanceMeters(0)).as("alone, it is judged against nothing").isNaN();

    CameraWatch within = new CameraWatch(20, 0, 0, 0.001);
    within.accepted(0, 1_000_000_000L, 0.1, 0, 0, 2, CLAIMED, CLAIMED, 0, 0);
    within.accepted(1, 1_100_000_000L, 0.1, 0, 0, 3, CLAIMED, CLAIMED, 0, 0);
    assertThat(within.distanceMeters(1)).as("a tenth of a second on").isEqualTo(3);
    within.accepted(2, 1_200_000_001L, 0.1, 0, 0, 3, CLAIMED, CLAIMED, 0, 0);
    assertThat(within.distanceMeters(2)).as("just past a tenth after the last").isNaN();
    assertThat(within.offY(ConsideredErrors.NO_PLACE, 0, 1)).isZero();
  }
}
