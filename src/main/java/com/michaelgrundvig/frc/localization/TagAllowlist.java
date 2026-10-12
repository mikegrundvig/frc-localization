package com.michaelgrundvig.frc.localization;

import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;

/**
 * The tags a task trusts, and no others: while a robot lines up on a target, its cameras measure it
 * only from that target's tags. At an event the tags stand where the field's crew put them,
 * centimeters from the layout's numbers and from each other, and what the robot needs is to be
 * right relative to the target's own tags, not an average with tags across the field.
 *
 * <p>Others did the same more loosely. A top team (2025) kept one fixed list, the reef's tags, for
 * its single-tag fixes; and while its auto-align drove to a branch, it took a camera's whole
 * solution only if the branch's tag was among those it used. Another team (2026) gave its drivers a
 * "hub tags only" switch. This is stricter than theirs: an image is solved from the trusted tags
 * alone, and the vision software's multi-tag solution is used only if it used no others. The cost
 * is heading: while a task runs, an image that also saw other tags corrects only position, from the
 * trusted tags each on its own, and a robot not yet placed solves the trusted ones without the
 * others' help.
 *
 * <p>Name the tags through the field model rather than by number, so the task works on any field
 * and for either alliance: a task lining up on a place takes the place's tags with {@code
 * vision.trustingOnly(hub, task)}, which finds them for the robot's alliance when the task starts
 * (bindings are made before the alliance is known).
 *
 * @param task what the tags are for, as the log names it: "Hub"
 * @param tagIds the tags it trusts: at least one
 */
public record TagAllowlist(String task, Set<Integer> tagIds) {
  public TagAllowlist {
    if (task.isBlank()) {
      throw new IllegalArgumentException("an allowlist names its task");
    }
    if (tagIds.isEmpty()) {
      throw new IllegalArgumentException(task + ": an allowlist trusts at least one tag");
    }
    tagIds = Set.copyOf(tagIds);
  }

  /** The tags a task trusts, in any order. */
  public static TagAllowlist of(String task, List<Integer> tagIds) {
    return new TagAllowlist(task, Set.copyOf(tagIds));
  }

  /** Whether the task trusts a tag. */
  public boolean allows(int tagId) {
    return tagIds.contains(tagId);
  }

  /** For the log: "Hub: 25, 26". */
  public String describe() {
    return task
        + ": "
        + new TreeSet<>(tagIds).stream().map(String::valueOf).collect(Collectors.joining(", "));
  }
}
