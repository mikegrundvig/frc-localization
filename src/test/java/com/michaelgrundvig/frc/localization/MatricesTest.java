package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The small matrices' one tolerance: a 2x2 spread and the same spread as a 3x3 with a heading of
 * its own beside it are singular, or not, alike. Before, the 2x2 asked only for a positive
 * determinant, so a block a hair from singular passed it and failed the 3x3's test.
 */
class MatricesTest {
  /** A symmetric 2x2 spread {@code [a b; b c]}, and whether it has an inverse to measure by. */
  private record Spread(String name, double a, double b, double c, boolean invertible) {}

  @Test
  void aSpreadIsInvertibleAsTwoByTwoExactlyWhenItIsAsThreeByThree() {
    List<Spread> rows =
        List.of(
            new Spread("round", 0.01, 0, 0.01, true),
            new Spread("long and leaning", 0.04, 0.019, 0.01, true),
            new Spread("a line: x and y vary as one", 0.01, 0.01, 0.01, false),
            new Spread("a hair from a line", 0.01, 0.01 * (1 - 1e-14), 0.01, false),
            new Spread("none at all", 0, 0, 0, false),
            new Spread("not a number", Double.NaN, 0, 0.01, false));
    for (Spread row : rows) {
      // The same spread, and beside it a heading of the same size, varying with neither.
      double[] three = {row.a(), row.b(), 0, row.b(), row.c(), 0, 0, 0, row.a()};
      boolean asThree = Matrices.invert3(three, new double[9]);

      assertThat(Matrices.invertible2(row.a(), row.b(), row.c()))
          .as(row.name())
          .isEqualTo(row.invertible());
      assertThat(asThree).as("%s, as 3x3", row.name()).isEqualTo(row.invertible());
    }
  }
}
