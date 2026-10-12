package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.util.List;
import org.junit.jupiter.api.Test;

/** The tags a task trusts. */
class TagAllowlistTest {
  @Test
  void itTrustsItsTagsAndNoOthers() {
    TagAllowlist hub = TagAllowlist.of("Hub", List.of(26, 25, 26));

    assertThat(hub.allows(25)).isTrue();
    assertThat(hub.allows(26)).isTrue();
    assertThat(hub.allows(10)).isFalse();
    assertThat(hub.describe()).isEqualTo("Hub: 25, 26");
  }

  @Test
  void itNamesItsTaskAndTrustsAtLeastOneTag() {
    assertThatThrownBy(() -> TagAllowlist.of(" ", List.of(1)))
        .hasMessage("an allowlist names its task");
    assertThatThrownBy(() -> TagAllowlist.of("Start", List.of()))
        .hasMessage("Start: an allowlist trusts at least one tag");
  }
}
