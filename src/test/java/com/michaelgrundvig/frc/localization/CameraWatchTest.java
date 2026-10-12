package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

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
    final CameraWatch watch = new CameraWatch(50, Math.toRadians(0.02), 0.001);
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
          watch.accepted(camera, nanos, x, y, heading, TAGS_AT_METERS, CLAIMED, odometryVariance);
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
    CameraWatch watch = new CameraWatch(50, 0, 0);
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
            odometryVariance);
      }
    }
    return watch;
  }

  @Test
  void aCameraWithoutAPlaceOrEnoughImagesIsNeverDoubted() {
    Watched watched = new Watched();
    watched.watch.accepted(ConsideredErrors.NO_PLACE, watched.nanos, 1, 1, 0, 2, CLAIMED, CLAIMED);
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
    assertThatThrownBy(() -> new CameraWatch(19, 0, 0)).hasMessageContaining("at least 20");
  }
}
