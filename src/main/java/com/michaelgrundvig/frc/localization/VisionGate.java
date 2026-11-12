package com.michaelgrundvig.frc.localization;

/**
 * The gate a camera measurement must pass before the estimator applies it. It judges and keeps no
 * books: the estimator counts what it says.
 *
 * <p>The gate is a chi-squared test: the innovation's squared distance from zero, measured in its
 * own uncertainty, may not exceed the configured limit for a measurement of its kind (a position
 * alone, or a pose). A measurement with a heading of its own must also back a person's word on the
 * heading while one is in doubt ({@link HeadingDoubt}).
 */
final class VisionGate {
  /**
   * A camera's heading within this many spreads of the estimate's doubt of it agrees with the
   * estimate: three, as far as an honest estimate's error goes in all but a few hundredths of the
   * time.
   */
  private static final double HEADING_AGREEMENT_SPREADS = 3;

  /** What a measurement's own heading says of the estimate's. */
  enum OwnHeading {
    /** It has no heading of its own: it was solved with the estimate's, or without one. */
    NONE,
    /** Its heading agrees with the estimate's, so it backs a person's word on the heading. */
    AGREES,
    /** Its heading is further from the estimate's than the estimate's doubt allows. */
    DISAGREES
  }

  /**
   * What the gate made of a measurement.
   *
   * @param verdict {@link Verdict#ACCEPTED}, {@link Verdict#GATED}, or {@link
   *     Verdict#HEADING_UNBACKED}
   * @param ownHeading what its own heading says of the estimate's
   * @param chiSquared its innovation's chi-squared
   * @param threshold the limit that chi-squared was held to
   */
  record Judgment(Verdict verdict, OwnHeading ownHeading, double chiSquared, double threshold) {}

  private final double gateChiSquaredPosition;
  private final double gateChiSquaredPose;

  /** A camera's own heading within this of the estimate's agrees with it, however sure both are. */
  private final double headingAgreementRadians;

  private final PoseFilter filter;
  private final ErrorPlaces places;

  VisionGate(EstimatorConfig config, PoseFilter filter, ErrorPlaces places) {
    gateChiSquaredPosition = config.gateChiSquaredPosition();
    gateChiSquaredPose = config.gateChiSquaredPose();
    headingAgreementRadians = Math.toRadians(config.recoveryAgreementDegrees());
    this.filter = filter;
    this.places = places;
  }

  /**
   * Judges a measurement by its innovation against the estimate.
   *
   * @param camera the camera's place
   * @param headingDoubt whose word the heading rests on, as it stands
   */
  Judgment judge(Innovation innovation, int camera, HeadingDoubt headingDoubt) {
    double threshold = innovation.withHeading() ? gateChiSquaredPose : gateChiSquaredPosition;
    double chiSquared = innovation.chiSquared();
    OwnHeading ownHeading = ownHeading(innovation, camera);
    Verdict verdict = verdict(chiSquared, threshold, ownHeading, headingDoubt);
    return new Judgment(verdict, ownHeading, chiSquared, threshold);
  }

  /**
   * A chi-squared over the limit fails the gate, and so does one that isn't a number. A heading of
   * its own that doesn't back a person's word fails too, however wide its spread lets it pass.
   * Taken, it would back the placement ({@link MotionEvidence}) and pull the heading only a share
   * of the way; rejected, three that agree turn the heading there.
   */
  private static Verdict verdict(
      double chiSquared, double threshold, OwnHeading ownHeading, HeadingDoubt headingDoubt) {
    if (Double.isNaN(chiSquared)) {
      return Verdict.GATED;
    }
    if (chiSquared > threshold) {
      return Verdict.GATED;
    }
    boolean unbacked = ownHeading == OwnHeading.DISAGREES && headingDoubt != HeadingDoubt.NONE;
    if (unbacked) {
      return Verdict.HEADING_UNBACKED;
    }
    return Verdict.ACCEPTED;
  }

  private OwnHeading ownHeading(Innovation innovation, int camera) {
    if (!innovation.withHeading()) {
      return OwnHeading.NONE;
    }
    if (Math.abs(innovation.headingRadians()) <= headingAllowance(camera)) {
      return OwnHeading.AGREES;
    }
    return OwnHeading.DISAGREES;
  }

  /**
   * What a rejected measurement's position says of the estimate's. One solved with a heading in
   * doubt says nothing: it lands off by that heading's error however right it is.
   *
   * @param headingInDoubt whether the heading is a person's word no camera has backed yet
   */
  Recovery.PositionSays positionSays(
      Innovation innovation, VisionMeasurement measurement, boolean headingInDoubt) {
    if (innovation.positionChiSquared() <= gateChiSquaredPosition) {
      return Recovery.PositionSays.AGREES;
    }
    if (measurement.leansOnTheHeading() && headingInDoubt) {
      return Recovery.PositionSays.NOT_COUNTED;
    }
    return Recovery.PositionSays.WRONG;
  }

  /**
   * How far a camera's own heading may be from the heading the estimate has it seeing and still
   * agree. It is the agreement angle ({@link EstimatorConfig#recoveryAgreementDegrees}) or {@value
   * #HEADING_AGREEMENT_SPREADS} spreads of the estimate's doubt of that heading, whichever is
   * wider. The doubt is the robot's heading's and the camera's turn on its mount's together, so a
   * placement sure to 2° and a mount sure to 1° are not told wrong by a camera 5° off.
   */
  double headingAllowance(int camera) {
    double spread = Math.sqrt(filter.cameraHeadingVariance(camera));
    return Math.max(headingAgreementRadians, HEADING_AGREEMENT_SPREADS * spread);
  }

  /** The widest of the cameras' heading allowances: what a turn on their word must be past. */
  double widestHeadingAllowance() {
    double widest = headingAgreementRadians;
    for (int camera = 0; camera < places.placedCameraCount(); camera++) {
      widest = Math.max(widest, headingAllowance(camera));
    }
    return widest;
  }
}
