# FRC localization

Where an FRC robot is on the field, worked out from its wheels, its IMUs and its cameras. It is a
Java library on the [robot model](https://github.com/mikegrundvig/frc-robot-model) and WPILib's
math, and nothing else of a robot program. It has no HAL, no NetworkTables and no clock, so a
robot, a simulator and a replay of a log all get the same answers from it.

It is pre-release: versions are dated releases with no promise of compatibility from one to the
next, and anything may change. It has run in simulation only, never on a real robot.

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
estimate, recovers when the cameras agree against odometry (two cameras, two tags, or a camera and
a sign from the IMU that the robot was moved, never one camera repeating one tag), lowers the
wheels' trust when they slip, and keeps an honest uncertainty. It estimates the gyro's scale and
drift, learns each camera's turn on its mount as the robot drives, and watches each camera against
the others for one knocked on its mount or stamped late. It reports every option.

**Cameras' turns on their mounts.** A camera turned a degree from where the robot's description
says puts every tag, and so the robot, a degree round: 5 cm at 3 m, the same way in every image,
and every heading it gives a degree off. The estimator keeps each camera's turn in its state,
starting as unsure as the camera's `mountTolerance` (5 mm and 1° unless the robot says otherwise),
and learns it from the camera's images:

- No single image tells a camera's turn from a heading error: the image sees both alike. The gyro
  carries the heading from one camera's images to the next's, so how each camera's turn differs
  from the others' is learned as the robot drives and turns.
- What every camera's turn shares, only the wheels could tell from the heading, by how the robot
  travels against where the cameras put it, and a wheel zeroed a few degrees off says the same
  thing wrongly. So that is carried, never corrected: its doubt, as wide as the mounts' tolerances
  allow together, stays in the heading's.
- A camera whose headings turn all at once, as a knocked camera's do, beyond what its noise and the
  estimate's doubt explain, has its turn opened up again and relearned.
- A shove, a wheel's slip, a misread tag or a tag off the layout moves where a camera puts the
  robot, not the camera's own heading, and is not learned as a turn: the scenario tests
  (`MountTurnTest`) check each.

It learns the turn about the vertical only. A camera shifted or tilted on its mount is carried as
before, as an error it repeats in every image.

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

It is published to the FRC libraries' Maven repository,
[mikegrundvig.github.io/frc-maven](https://mikegrundvig.github.io/frc-maven/), as
`com.michaelgrundvig.frc:frc-localization`, with the conformance suite in its test fixtures. The
runtime brings it, so a robot program built on the runtime has it already. Another build adds:

```groovy
// build.gradle
repositories {
    exclusiveContent {
        forRepository {
            maven { url = "https://mikegrundvig.github.io/frc-maven/" }
        }
        filter { includeGroup "com.michaelgrundvig.frc" }
    }
}
dependencies {
    implementation "com.michaelgrundvig.frc:frc-localization:2026.10.22"
    testImplementation testFixtures("com.michaelgrundvig.frc:frc-localization:2026.10.22")
}
```

It reads the robot model at the same version. WPILib's libraries are compiled against and never
brought along: a robot program has them already, from GradleRIO, at its own version.

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
git clone https://github.com/mikegrundvig/frc-localization.git
cd frc-localization
./gradlew test -Pquick     # the tests, fast
./gradlew ci               # everything CI runs: formatting, Error Prone and NullAway, tests, coverage, Javadoc
./gradlew pitest           # mutation testing, which takes minutes and is not part of ci
```

Gradle downloads the JDK it needs. The tests need no robot, no simulator and no native libraries,
and they run in seconds.

`./gradlew pitest -PpitClasses='com.michaelgrundvig.frc.localization.SlipCheck*'` runs mutation
testing on some classes only.

The robot model comes from the FRC libraries' repository, at the version the catalog's `frc-libs`
names (`gradle/libs.versions.toml`). Two switches change where libraries come from, both off by
default:

- `-PlocalLibs=<folder>` builds the model from a clone of it in `<folder>/model`, from source, in
  place of the published version, to try a change to the model here before it's published.
- `-PfrcMaven=<url>` reads the published libraries from another copy of the repository, such as a
  local clone holding a release not pushed yet: `-PfrcMaven=file:///home/you/src/frc-maven`.

## Publishing it

`./gradlew publishToFrcMaven` runs every check, then writes this release (the catalog's `frc-libs`)
into `../frc-maven`, a clone of [frc-maven](https://github.com/mikegrundvig/frc-maven) beside this
one (`-PfrcMavenDir=<folder>` names another). The model must be published at that version first;
frc-maven's `release.sh` publishes every library in order and commits, and its README says how a
release goes.

## What the comparison found

The robot template has a study that runs both localizers on the same simulated robot and scores
each against the simulator's true pose. Every contender is run through the `Localizer` interface.

What it found, in simulation only:

- In clean driving the two are roughly equal. Both hold the pose to about a centimeter.
- The default localizer is better against bad camera data, such as a camera that keeps misreading
  a tag, and against wheels that slip.
- With the gyro reading wrong, or a camera knocked or turned on its mount or stamped late, the two
  are within a centimeter of each other. The default localizer used to lose there. It now
  estimates the gyro's scale and drift and each camera's turn on its mount, and takes off what a
  late camera puts its measurements off by. A camera turned 6° left it 16.9 cm off, against 2.4 cm
  for WPILib's; it is now 1.7 cm against 2.3. The simulator's gyro and cameras are simpler than
  real ones, so this is the result to trust least.
- With worse cameras than the simulator's (corners twice as noisy, tags seen to 5 m, each mount up
  to 5 mm and 1° off), the default localizer is ahead in clean driving, 1.1 cm against 1.3, and its
  heading's uncertainty stays honest. Before it learned the cameras' turns it was half a centimeter
  behind there, and claimed about half the heading doubt a camera turned a degree costs.
- WPILib's estimator, behind the kind of vision filter strong teams write for it, moves more
  smoothly. The default localizer corrects a wrong estimate in one step, which can be tens of
  centimeters after a shove.
- Most of each difference comes from how images are solved and filtered, not from the estimator.
  Fed the very same measurements, the two estimators are close in most scenarios.
- The study did not measure a robot that chose `WpilibLocalizer`. Such a robot's images are solved
  without the gyro's heading, which none of the study's contenders did with this pipeline.

No real robot was involved. Do not quote a number from it as a fact about your robot.

## License

BSD 3-Clause. See [LICENSE](LICENSE).
