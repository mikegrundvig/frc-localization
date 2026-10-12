package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;

/**
 * Makes a robot's {@link Localizer} as the robot program starts. A robot's configuration names one
 * to choose its localizer, in the robot's block:
 *
 * <pre>{@code
 * .localizer(WpilibLocalizer::new)
 * }</pre>
 *
 * <p>A robot that names none gets the {@link DefaultLocalizer}, weighing its sources as the
 * configuration's {@code estimator} says.
 */
@FunctionalInterface
public interface LocalizerMaker {
  /**
   * Makes the localizer for a robot on a field.
   *
   * @param robot what the robot is: its wheels, its IMUs, and its cameras
   * @param field the field the robot plays on, as the season has it
   */
  Localizer make(RobotDefinition robot, FieldModel field);
}
