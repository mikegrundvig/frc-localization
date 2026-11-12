package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.RevoluteJoint;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.util.LinkedHashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

class JointHistoryTest {
  private static RevoluteJoint joint(String label, double reference) {
    return Joint.pivot(label, Part.CHASSIS, new Vector3(0, 0, 1))
        .at(Meters.of(0), Meters.of(0), Meters.of(0))
        .rest(Radians.of(reference));
  }

  private static final RevoluteJoint TURRET = joint("turret", 0.5);
  private static final RevoluteJoint HOOD = joint("hood", 0.1);

  private static JointHistory history() {
    Map<Joint, Double> references = new LinkedHashMap<>();
    references.put(TURRET, RobotMath.restPosition(TURRET));
    references.put(HOOD, RobotMath.restPosition(HOOD));
    return new JointHistory(references, 3);
  }

  @Test
  void beforeAnyReadingEachJointIsAtItsReference() {
    assertThat(history().at(0)).containsEntry(TURRET, 0.5).containsEntry(HOOD, 0.1);
  }

  @Test
  void betweenReadingsItInterpolatesAndOutsideThemHoldsTheNearest() {
    JointHistory history = history();
    history.add(100, Map.of(TURRET, 1.0, HOOD, 0.2));
    history.add(200, Map.of(TURRET, 2.0, HOOD, 0.4));

    assertThat(history.at(150).get(TURRET)).isCloseTo(1.5, within(1e-12));
    assertThat(history.at(150).get(HOOD)).isCloseTo(0.3, within(1e-12));
    assertThat(history.at(50).get(TURRET)).isEqualTo(1.0);
    assertThat(history.at(250).get(TURRET)).isEqualTo(2.0);
  }

  @Test
  void aJointNotReadStaysWhereItWasAndAnOldReadingIsIgnored() {
    JointHistory history = history();
    history.add(100, Map.of(TURRET, 1.0));
    history.add(200, Map.of(HOOD, 0.4));
    history.add(150, Map.of(TURRET, 9.0));

    assertThat(history.at(100)).containsEntry(TURRET, 1.0).containsEntry(HOOD, 0.1);
    assertThat(history.at(200)).containsEntry(TURRET, 1.0).containsEntry(HOOD, 0.4);
  }

  @Test
  void itKeepsOnlyItsCapacity() {
    JointHistory history = history();
    for (int k = 1; k <= 5; k++) {
      history.add(k * 100L, Map.of(TURRET, (double) k));
    }

    assertThat(history.at(0).get(TURRET)).as("the oldest kept, the third").isEqualTo(3.0);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new JointHistory(Map.of(), 1))
        .withMessage("keep at least two readings: 1");
  }
}
