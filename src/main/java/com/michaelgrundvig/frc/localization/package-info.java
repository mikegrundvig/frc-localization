/**
 * Localization: where the robot is, from its wheels, its IMUs, its cameras, and the field's
 * terrain. Hardware (or its simulation) reports through an IO implementation outside this package,
 * in the types here, which no vendor's library leaks into; new hardware means a new IO
 * implementation, and nothing here changes.
 *
 * <p>Where to start reading:
 *
 * <ul>
 *   <li><b>The seam:</b> {@link com.michaelgrundvig.frc.localization.Localizer} is what works out
 *       where the robot is, behind one interface: odometry and camera measurements in, a pose out.
 *       Its Javadoc is its contract, clause by clause. There are two here, {@link
 *       com.michaelgrundvig.frc.localization.DefaultLocalizer} (this library's own estimator, which
 *       a robot gets unless it chooses otherwise) and {@link
 *       com.michaelgrundvig.frc.localization.WpilibLocalizer} (WPILib's pose estimator), and a team
 *       may write its own. A {@link com.michaelgrundvig.frc.localization.LocalizerMaker} is how a
 *       robot's configuration names one. The conformance suite in the test fixtures ({@code
 *       LocalizerConformance}) checks any localizer against the contract.
 *   <li><b>The service:</b> {@link com.michaelgrundvig.frc.localization.LocalizationService} is
 *       localization as the robot program uses it, one loop at a time, around whichever localizer
 *       the robot chose: the drivetrain's readings ({@link
 *       com.michaelgrundvig.frc.localization.DrivetrainReadings}), each camera image, and a
 *       placement in, stamped with their times, and each loop's {@link
 *       com.michaelgrundvig.frc.localization.LocalizationOutput} out. It solves each image into the
 *       measurement the localizer is given, which no localizer replaces. The loop's time comes in a
 *       {@link com.michaelgrundvig.frc.localization.LoopContext}. The robot program's {@code
 *       Localization} only feeds it, logs it, and raises its alerts.
 *   <li><b>What it says:</b> the estimate; how far each source is trusted ({@link
 *       com.michaelgrundvig.frc.localization.Trust}: each camera's {@link
 *       com.michaelgrundvig.frc.localization.CameraTrust}, the wheels' {@link
 *       com.michaelgrundvig.frc.localization.WheelTrust}); and what became of each camera image, by
 *       tag ({@link com.michaelgrundvig.frc.localization.FrameVerdict}, {@link
 *       com.michaelgrundvig.frc.localization.TagVerdict}). Every refusal is a {@link
 *       com.michaelgrundvig.frc.localization.Verdict}, whose words the log and the page show, so
 *       nothing reads a reason back out of a sentence.
 *   <li><b>The default localizer's estimator:</b> {@link
 *       com.michaelgrundvig.frc.localization.PoseEstimator} takes every measurement and keeps the
 *       estimate, tuned by {@link com.michaelgrundvig.frc.localization.EstimatorConfig}. What it
 *       reports is a {@link com.michaelgrundvig.frc.localization.LocalizationEstimate}; {@link
 *       com.michaelgrundvig.frc.localization.LocalizationGate} says whether that's good enough to
 *       score on. Inside it, {@code OdometryHistory} keeps the steps a late measurement is replayed
 *       through, {@code PoseFilter} does the filter's math (and keeps the errors that repeat,
 *       below) with {@code CameraCorrection} working each camera measurement, {@code WheelDoubt}
 *       says how far each step's wheels are doubted, and {@code Recovery} judges when the cameras
 *       agree that odometry is wrong.
 *   <li><b>Wheels:</b> an {@link com.michaelgrundvig.frc.localization.OdometrySample} of every
 *       wheel, turned into the robot's {@link com.michaelgrundvig.frc.localization.BodyMotion} by
 *       the drivetrain's {@link com.michaelgrundvig.frc.localization.DriveKinematicsModel} (swerve
 *       or differential), whose residual says how far the wheels' disagreement with each other may
 *       have put it off. {@link com.michaelgrundvig.frc.localization.DriveConditions} says how far
 *       to trust them right now, and {@link com.michaelgrundvig.frc.localization.SlipCheck} whether
 *       the IMU sees them sliding together.
 *   <li><b>IMUs:</b> {@link com.michaelgrundvig.frc.localization.YawTrack} places yaw readings in
 *       time, and {@link com.michaelgrundvig.frc.localization.ImuCrossCheck} compares two IMUs; a
 *       tilt the field's terrain explains keeps the wheels trusted. The service picks which IMU
 *       turns odometry: the gyro, the backup while the gyro is lost, or the wheels.
 *   <li><b>Cameras:</b> the raw {@link com.michaelgrundvig.frc.localization.TagDetections} of one
 *       image go through the {@link com.michaelgrundvig.frc.localization.VisionPipeline}, which
 *       solves them (with {@link com.michaelgrundvig.frc.localization.TagSolver}) into a {@link
 *       com.michaelgrundvig.frc.localization.VisionMeasurement}, or says why not; the estimator's
 *       {@link com.michaelgrundvig.frc.localization.VisionDecision} says what became of it. The
 *       service solves each image as the robot was when it was taken ({@link
 *       com.michaelgrundvig.frc.localization.CaptureContext}), against the field's tag poses. The
 *       robot program's {@code Vision} still holds a field model of its own, to place each tag it
 *       saw for its camera checks, until venues (the architecture's migration step 5) make the
 *       service the one owner of the tags' poses. {@link
 *       com.michaelgrundvig.frc.localization.MountCheck} judges, from where a camera's tags land,
 *       whether it still sits where its configuration says, and how much less to trust it if not; a
 *       {@link com.michaelgrundvig.frc.localization.TagAllowlist} keeps a task to the tags it
 *       trusts.
 * </ul>
 *
 * <p>Conventions:
 *
 * <ul>
 *   <li><b>Time:</b> every measurement carries the moment it was captured, not when it arrived, in
 *       nanoseconds on the robot's clock ({@code RobotController.getTime()}, WPILib 2027's integer
 *       clock). Vendors report seconds: convert with {@link
 *       com.michaelgrundvig.frc.localization.Timestamps}. Nothing here reads a clock: the loop's
 *       own time comes in its {@link com.michaelgrundvig.frc.localization.LoopContext}.
 *   <li><b>Threads:</b> every call is made on the robot loop's thread, in the service's fixed
 *       order. The drivetrain's 250 Hz thread only samples and queues; its readings come in once a
 *       loop.
 *   <li><b>Frames:</b> as written in {@code com.michaelgrundvig.frc.world.field}: poses in the
 *       field frame, the robot's own motion in the robot frame.
 *   <li><b>Uncertainty:</b> a {@link com.michaelgrundvig.frc.localization.PoseCovariance}, in
 *       square meters and square radians. An infinite heading variance means the measurement says
 *       nothing about heading, which is how a solve that took its heading from the gyro reports it;
 *       such a measurement also says how far its tags were ({@link
 *       com.michaelgrundvig.frc.localization.VisionMeasurement#lever}), since the gyro's heading
 *       error moves its position by that distance times the error.
 * </ul>
 *
 * <p><b>An honest uncertainty.</b> Gates, χ² tests, and any bound on the robot's error trust the
 * covariance the estimator reports, so it must not claim more than it knows: the error should go
 * past 3.72 times the claimed spread no more than once in a thousand loops. The consistency study
 * checks that in the simulator, where the truth is known (docs/testing.md), and a test keeps it.
 * What makes an estimator overconfident is an error that repeats, averaged as if each instance were
 * new. So the estimator never does that with the ones it knows:
 *
 * <ul>
 *   <li>a position a camera solved with the gyro's heading leans on that heading, in every image:
 *       it carries the heading's doubt, and by default doesn't correct the heading (the gyro owns
 *       it; a measurement with a heading of its own corrects its drift, with all three of its rows:
 *       its position too, through how the position's error has come to depend on the heading's).
 *       {@link com.michaelgrundvig.frc.localization.EstimatorConfig#headingCorrection} can let
 *       every fix correct it;
 *   <li>a gyro reading its turns a little long or short is off the same way every turn;
 *   <li>each camera's turn and shift on its mount are in every image it takes, and what its
 *       multi-tag headings share, until it moves on and sees other tags;
 *   <li>each tag's placement off the field's layout is in every image any camera takes of it: two
 *       cameras seeing the same tags share its error, and turning away and back finds it again.
 * </ul>
 *
 * <p>The filter keeps these as "consider" parameters: errors it accounts for but never estimates.
 * Their sizes are in {@link com.michaelgrundvig.frc.localization.EstimatorConfig}, and
 * docs/values.md says how to measure each.
 */
@NullMarked
package com.michaelgrundvig.frc.localization;

import org.jspecify.annotations.NullMarked;
