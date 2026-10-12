package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.EnumSet;
import java.util.Optional;
import java.util.Set;

/**
 * A test bench for one localizer: it makes a fresh one for each scenario of the conformance suite
 * ({@link LocalizerConformance}), and says which options that localizer does not report. The suite
 * knows only {@link Localizer} and this.
 *
 * <p>To check a localizer of your own, write a test class of a few lines:
 *
 * <pre>{@code
 * class MyLocalizerConformanceTest extends LocalizerConformance {
 *   MyLocalizerConformanceTest() {
 *     super(LocalizerBench.of("mine", MyLocalizer::new, Option.COVARIANCE));
 *   }
 * }
 * }</pre>
 */
public interface LocalizerBench {
  /** What the bench is called in a test's message, such as "WPILib". */
  String name();

  /**
   * A new localizer for a robot on a field, as the robot program makes one when it starts.
   *
   * @param robot the suite's robot: a swerve drive with one camera
   * @param field the suite's field
   */
  Localizer make(RobotDefinition robot, FieldModel field);

  /**
   * Why the localizer does not report an option, or empty if it reports it. The suite skips the
   * clauses that need the option, and checks that the localizer really reports nothing for it.
   */
  Optional<String> lacks(Option option);

  /** An option of the contract: one thing a localizer may report, or not. */
  enum Option {
    /** {@link Localizer#covariance}. */
    COVARIANCE,
    /** {@link Localizer#mode}. */
    MODE,
    /** {@link Localizer#health}. */
    HEALTH,
    /** {@link Localizer#decisions}. */
    DECISIONS,
    /** {@link Localizer#fieldVelocities}. */
    FIELD_VELOCITIES,
    /** {@link Localizer#wheels}. */
    WHEELS,
    /** {@link Localizer#trust}. */
    TRUST,
    /** {@link Localizer#yawSource}. */
    YAW_SOURCE
  }

  /**
   * A bench for a localizer made by a maker, as a robot's configuration names it.
   *
   * @param name what the bench is called in a test's message
   * @param maker makes the localizer
   * @param reported the options the localizer reports; it is taken to lack every other
   */
  static LocalizerBench of(String name, LocalizerMaker maker, Option... reported) {
    Set<Option> has = EnumSet.noneOf(Option.class);
    has.addAll(Set.of(reported));
    return new LocalizerBench() {
      @Override
      public String name() {
        return name;
      }

      @Override
      public Localizer make(RobotDefinition robot, FieldModel field) {
        return maker.make(robot, field);
      }

      @Override
      public Optional<String> lacks(Option option) {
        if (has.contains(option)) {
          return Optional.empty();
        }
        return Optional.of("it does not report it");
      }
    };
  }
}
