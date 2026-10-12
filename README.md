# FRC localization

Where an FRC robot is on the field, worked out from its wheels, its IMUs and its cameras. It is a
Java library on the [robot model](https://github.com/mikegrundvig/frc-robot-model) and WPILib's
math, and nothing else of a robot program. It has no HAL, no NetworkTables and no clock, so a
robot, a simulator and a replay of a log all get the same answers from it.

It is unreleased. There are no versions yet, and anything may change. It has run in simulation
only, never on a real robot.

## What is in it

Everything is in one package, `com.michaelgrundvig.frc.localization`.

| Part | What it is |
|---|---|
| `Localizer` | The contract. Odometry and camera measurements go in, and a pose comes out. |
| `DefaultLocalizer` | The library's own estimator, as a localizer. |
| `WpilibLocalizer` | WPILib's `SwerveDrivePoseEstimator`, as a localizer. |
| `LocalizationService` | One robot loop around whichever localizer the robot chose. It also solves each camera image into the measurement a localizer is given. |
| `VisionPipeline` | The solving and filtering of AprilTag detections into measurements. |
| `calibration` | The math of the calibration procedures: a camera's mount, the gyro's scale, the wheels' radius. |
| `LocalizerConformance` | The contract as a test suite, in the test fixtures, for checking a localizer of your own. |

## The contract

A localizer is told three things each loop, in this order:

```java
void odometry(DrivetrainReadings readings, LoopContext loop);
void place(PoseReset placed);
void measurements(List<VisionMeasurement> taken);
```

`odometry` comes every loop, with every wheel reading since the last loop and both IMUs. `place`
comes only when the drive team placed the robot. `measurements` comes every loop, and its list is
empty when no camera saw a tag.

It must answer three things:

```java
Pose2d pose();
Optional<Pose2d> poseAt(long timestampNanos);
long timestampNanos();
```

The whole contract is in `Localizer`'s Javadoc, as numbered clauses, L1 to L23. It is written to
be enough to implement a localizer without reading either of the two here. In short:

- Poses are on the field, in meters and radians. Times are nanoseconds on the robot's clock. A
  localizer reads no clock of its own.
- `pose()` always answers. `poseAt(time)` reaches back half a second at least.
- With odometry alone the pose moves as the wheels and the gyro say. A lost gyro, a wheel that
  did not report and a loop with no readings are all ordinary.
- After `place`, the pose is the placed pose.
- A measurement that agrees is used, and never overshot. It is applied at its capture time. One
  with no heading does not turn the estimate. Bad data costs one measurement, never the loop.
- A loop costs the same after two minutes as after two seconds.

The contract says what a localizer must do to be driven by. It says nothing about accuracy.

### The options

A localizer may also report more. Each option is one method that returns an `Optional`, and
returns empty unless you override it: `covariance()`, `mode()`, `health()`, `decisions()`,
`fieldVelocities()`, `wheels()`, `trust()` and `yawSource()`.

Report an option only where your localizer really knows the answer. Whoever reads an empty one
says "not reported" in its place. Never fill one with a guess.

One option changes how images are solved. Without `covariance()`, nothing says the heading is
sure, so no image is solved using the pose's heading. Every image is then solved from its tags
alone.

### What stays in front of every localizer

The solving of camera images is not part of the contract, and a localizer does not replace it.
`LocalizationService` hands each camera's tag detections to the `VisionPipeline`, which solves and
filters them into `VisionMeasurement`s: a pose, the time the image was taken, and a covariance.
Every localizer is handed the same measurements. None sees an image.

## The two localizers

**`DefaultLocalizer`** is the library's own estimator. It gates every measurement against the
estimate, recovers when the cameras agree against odometry, lowers the wheels' trust when they
slip, and keeps an honest uncertainty. It reports every option.

**`WpilibLocalizer`** is WPILib's `SwerveDrivePoseEstimator`, fed as the default one is: every
wheel reading, and the pipeline's measurements with their standard deviations. It keeps the
required contract and reports none of the options, because WPILib's estimator keeps no
uncertainty, no mode and no record of what it did with a measurement. Since it reports no
covariance, its images are solved from their tags alone, never with the gyro's heading.

WPILib's estimator leaves some things to the code that calls it. `WpilibLocalizer` does them, and
its Javadoc says how: which sensor turns it when the gyro is lost, what happens to a placement's
uncertainty, the order measurements are given in, and how a measurement with no heading is
passed.

## Using it from the runtime

The [robot runtime](https://github.com/mikegrundvig/frc-robot-runtime) runs a
`LocalizationService` every loop and makes the localizer the robot's configuration names. A robot
that names none gets the default localizer. One line in a robot's block chooses another:

```java
.localizer(WpilibLocalizer::new)
```

The runtime's `docs/localizers.md` says what the robot program does without each option, and how
its pit page says so. Where this library's Javadoc points at `docs/`, it means the runtime's
guides.

A build includes this library as a git submodule, built as part of its own build (a Gradle
composite build), so there is no version to bump:

```
git submodule add https://github.com/mikegrundvig/frc-localization.git localization
```

```groovy
// settings.gradle
includeBuild("localization")

// build.gradle
dependencies {
    implementation "com.michaelgrundvig.frc:frc-localization"
    testImplementation testFixtures("com.michaelgrundvig.frc:frc-localization")
}
```

The build also includes the robot model, its own `includeBuild("model")`. This library includes
its `model/` submodule only when it is the build that runs, because Gradle refuses two builds
that provide one module. So a build that checks this library out does not need to initialize its
submodule.

## Writing and testing your own

Implement `Localizer`. Six methods have no default, and they are the six above. Then check it
against the contract. `LocalizerConformance` drives a scripted robot whose wheels and gyro are
exact, and checks each clause. A test class of a few lines runs it. This is the one for WPILib's
estimator:

```java
class WpilibLocalizerConformanceTest extends LocalizerConformance {
  WpilibLocalizerConformanceTest() {
    super(LocalizerBench.of("WPILib's pose estimator", WpilibLocalizer::new));
  }
}
```

Name the options your localizer reports after the maker, such as `Option.COVARIANCE`. The suite
runs the clauses for those, and checks that the rest really are empty.

Both localizers here pass it. The default one passes all 21 scenarios. WPILib's passes the 16
that every localizer must, and the 5 for options it does not report are skipped.

Your localizer picks its own gyro. `DrivetrainReadings` has the gyro and the backup IMU, and
either may be lost in any loop. Each IMU has its own zero, so switch between them by how far each
turned, not by what each reads.

## Building and testing it alone

```bash
git clone --recurse-submodules https://github.com/mikegrundvig/frc-localization.git
cd frc-localization
./gradlew test -Pquick     # the tests, fast
./gradlew ci               # everything CI runs: formatting, Error Prone and NullAway, tests, coverage, Javadoc
./gradlew pitest           # mutation testing, which takes minutes and is not part of ci
```

Gradle downloads the JDK it needs. The tests need no robot, no simulator and no native libraries,
and they run in seconds.

`./gradlew pitest -PpitClasses='com.michaelgrundvig.frc.localization.SlipCheck*'` runs mutation
testing on some classes only.

## What the comparison found

The robot template has a study that runs both localizers on the same simulated robot and scores
each against the simulator's true pose. Every contender is run through the `Localizer` interface.

What it found, in simulation only:

- In clean driving the two are roughly equal. Both hold the pose to about a centimeter.
- The default localizer is better against bad camera data, such as a camera that keeps misreading
  a tag, and against wheels that slip.
- WPILib's estimator, behind the kind of vision filter strong teams write for it, is better when
  the gyro reads wrong or a camera has been knocked on its mount. The default localizer leans on
  the gyro for heading, and that costs it there.
- Most of each difference comes from how images are solved and filtered, not from the estimator.
  Fed the very same measurements, the two estimators are close.
- The study did not measure a robot that chose `WpilibLocalizer`. Such a robot's images are solved
  without the gyro's heading, which none of the study's contenders did with this pipeline.

No real robot was involved. Do not quote a number from it as a fact about your robot.

## License

BSD 3-Clause. See [LICENSE](LICENSE).
