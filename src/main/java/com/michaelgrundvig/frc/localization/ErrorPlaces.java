package com.michaelgrundvig.frc.localization;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

/**
 * Gives each camera and each tag its place among the errors the {@link PoseFilter} keeps apart
 * ({@link ConsideredErrors}), and remembers the ones it had no place for.
 *
 * <p>A camera is known by its name and gets the next free place the first time it is asked about,
 * until the filter's {@link ConsideredErrors#MAX_CAMERAS} are taken. A tag is known by its number:
 * every tag of the field has a place from the start, in the order of their numbers, and no other
 * tag ever gets one.
 *
 * <p>A camera or tag without a place still gives measurements. The filter just cannot tell that its
 * error is the same from image to image, so it takes each image's as new, and those images,
 * averaged, grow surer than they should. That is why the ones without places are reported.
 */
final class ErrorPlaces {
  /** Each camera's place, by name, in the order cameras first appeared. */
  private final Map<String, Integer> cameraPlaces = new LinkedHashMap<>();

  /** The cameras that came after every place was taken, in the order they came. */
  private final Set<String> camerasWithoutPlaces = new LinkedHashSet<>();

  /** Each of the field's tags' place, by its number. */
  private final Map<Integer, Integer> tagPlaces = new HashMap<>();

  /** The tags measurements named that the field does not have, in the order they came. */
  private final Set<Integer> tagsWithoutPlaces = new LinkedHashSet<>();

  /**
   * One measurement's tags' places, handed to the filter. It is reused from one measurement to the
   * next, and replaced by a longer one only when a measurement names more tags than any before.
   */
  private int[] placesOfOneMeasurementsTags = new int[8];

  /**
   * @param fieldTags the numbers of the field's tags
   */
  ErrorPlaces(Collection<Integer> fieldTags) {
    for (int id : new TreeSet<>(fieldTags)) {
      tagPlaces.put(id, tagPlaces.size());
    }
  }

  /** How many tags have places: the field's. */
  int tagCount() {
    return tagPlaces.size();
  }

  /**
   * A camera's place, giving it the next free one if this is the first time it is asked about.
   *
   * @return its place, or {@link ConsideredErrors#NO_PLACE} if every place was taken before it came
   */
  int cameraPlace(String camera) {
    Integer place = cameraPlaces.get(camera);
    if (place != null) {
      return place;
    }
    if (cameraPlaces.size() >= ConsideredErrors.MAX_CAMERAS) {
      camerasWithoutPlaces.add(camera);
      return ConsideredErrors.NO_PLACE;
    }
    int next = cameraPlaces.size();
    cameraPlaces.put(camera, next);
    return next;
  }

  /**
   * The places of a measurement's tags, in the measurement's order: {@link
   * ConsideredErrors#NO_PLACE} for a tag the field does not have, which is also noted as one
   * without a place.
   *
   * @return an array at least as long as the list, whose entries past the list's length mean
   *     nothing. It is this object's own, good until the next call.
   */
  int[] tagPlaces(List<Integer> tagIds) {
    if (placesOfOneMeasurementsTags.length < tagIds.size()) {
      placesOfOneMeasurementsTags = new int[tagIds.size()];
    }
    for (int index = 0; index < tagIds.size(); index++) {
      placesOfOneMeasurementsTags[index] = tagPlace(tagIds.get(index));
    }
    return placesOfOneMeasurementsTags;
  }

  private int tagPlace(int id) {
    Integer place = tagPlaces.get(id);
    if (place != null) {
      return place;
    }
    tagsWithoutPlaces.add(id);
    return ConsideredErrors.NO_PLACE;
  }

  /** Every camera asked about, those with places first, each group in the order they came. */
  List<String> cameras() {
    List<String> cameras = new ArrayList<>(cameraPlaces.keySet());
    cameras.addAll(camerasWithoutPlaces);
    return List.copyOf(cameras);
  }

  List<String> camerasWithoutPlaces() {
    return List.copyOf(camerasWithoutPlaces);
  }

  List<Integer> tagsWithoutPlaces() {
    return List.copyOf(tagsWithoutPlaces);
  }
}
