package com.michaelgrundvig.frc.localization;

import java.util.Locale;

/**
 * What the estimator's latest recovery did, and on what evidence: the cameras, rejected one after
 * another, agreed with each other for independent reasons, so the estimate went to them.
 *
 * @param correction whether the estimate turned to the heading they agreed on, or moved to where
 *     they put the robot
 * @param amount by how much: degrees, counterclockwise, for a turn; meters for a move
 * @param evidence what made the rejected measurements independent of each other
 * @param measurements how many rejected measurements agreed
 */
public record Recovered(Correction correction, double amount, Evidence evidence, int measurements) {
  /** What a recovery changed. */
  public enum Correction {
    /** The heading, to the one the cameras' own headings agreed on. */
    TURNED,
    /** The position, to where the cameras put the robot. */
    MOVED
  }

  /** What made the cameras' agreement more than one source repeating itself. */
  public enum Evidence {
    /** They came from two cameras or more. */
    TWO_CAMERAS("from two cameras"),
    /** They rested on two tags or more. */
    TWO_TAGS("on two tags"),
    /** The robot had been placed, or its heading reset, and no camera had backed it. */
    PLACED("after a placement no camera had backed"),
    /** The IMU felt a jolt the wheels' grip can't explain. */
    HIT("after a hit the wheels can't explain"),
    /** The IMU saw the wheels slide over the floor. */
    SLID("after the wheels slid"),
    /** The robot was tilted past where its wheels' travel counts in full. */
    LIFTED("after the robot was lifted or tilted"),
    /** The robot was disabled, when people may carry it. */
    DISABLED("after the robot was disabled");

    private final String words;

    Evidence(String words) {
      this.words = words;
    }

    /** In a few words: "from two cameras". */
    public String words() {
      return words;
    }
  }

  /** In words, for the log: "turned 31.2° on 3 rejected measurements from two cameras". */
  public String describe() {
    String what =
        switch (correction) {
          case TURNED -> String.format(Locale.ROOT, "turned %.1f°", amount);
          case MOVED -> String.format(Locale.ROOT, "moved %.2f m", amount);
        };
    return what + " on " + measurements + " rejected measurements " + evidence.words();
  }
}
