package com.michaelgrundvig.frc.localization;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * How far localization trusts each of its sources as of an update: the one place its checks report,
 * each adding what it finds here rather than a channel of its own.
 *
 * @param cameras each camera's trust, by name: those the estimator has weighed a measurement from,
 *     in the order it first did, then the robot's others
 * @param wheels the drivetrain's wheels' trust this loop, from the IMU and the slip check
 * @param tagsWithoutPlaces the tags measurements named that the field's layout doesn't have, in the
 *     order they came: the estimator can't keep their placement apart, so it takes each image's as
 *     new, and their images together claim more than they know. None, normally
 */
@OutputRecord
public record Trust(
    Map<String, CameraTrust> cameras, WheelTrust wheels, Set<Integer> tagsWithoutPlaces) {
  public Trust {
    cameras = Collections.unmodifiableMap(new LinkedHashMap<>(cameras));
    Objects.requireNonNull(wheels, "the wheels' trust");
    tagsWithoutPlaces = Collections.unmodifiableSet(new LinkedHashSet<>(tagsWithoutPlaces));
  }

  /** Builds one by name: no cameras, and every tag kept apart, unless said. */
  public static TrustBuilder builder() {
    return TrustBuilder.builder();
  }
}
