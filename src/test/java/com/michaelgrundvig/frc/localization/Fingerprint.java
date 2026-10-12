package com.michaelgrundvig.frc.localization;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.RecordComponent;
import java.util.Collection;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Every number a value holds, bit for bit, in words: two values with the same fingerprint are the
 * same to the last bit, where WPILib's geometry calls two poses equal within 1e-9. Records are
 * taken apart component by component; a type it doesn't know fails, so nothing is skipped unseen.
 */
final class Fingerprint {
  private Fingerprint() {}

  static String of(@Nullable Object value) {
    StringBuilder out = new StringBuilder();
    add(out, value);
    return out.toString();
  }

  private static void add(StringBuilder out, @Nullable Object value) {
    switch (value) {
      case null -> out.append("null");
      case Double number -> bits(out, number);
      case Number number -> out.append(number);
      case Boolean flag -> out.append(flag);
      case String words -> out.append('"').append(words).append('"');
      case Enum<?> constant -> out.append(constant.name());
      case Optional<?> maybe -> {
        out.append("Optional[");
        maybe.ifPresent(present -> add(out, present));
        out.append(']');
      }
      case Collection<?> elements -> {
        out.append('[');
        for (Object element : elements) {
          add(out, element);
          out.append(',');
        }
        out.append(']');
      }
      case Map<?, ?> entries -> {
        out.append('{');
        for (Map.Entry<?, ?> entry : entries.entrySet()) {
          add(out, entry.getKey());
          out.append('=');
          add(out, entry.getValue());
          out.append(',');
        }
        out.append('}');
      }
      case Rotation2d turn ->
          numbers(out, "Rotation2d", turn.getRadians(), turn.getCos(), turn.getSin());
      case Translation2d point -> numbers(out, "Translation2d", point.getX(), point.getY());
      case Pose2d pose ->
          numbers(
              out,
              "Pose2d",
              pose.getX(),
              pose.getY(),
              pose.getRotation().getRadians(),
              pose.getRotation().getCos(),
              pose.getRotation().getSin());
      case Translation3d point ->
          numbers(out, "Translation3d", point.getX(), point.getY(), point.getZ());
      case Rotation3d turn -> quaternion(out, turn.getQuaternion());
      case Pose3d pose -> {
        add(out, pose.getTranslation());
        add(out, pose.getRotation());
      }
      case Transform3d transform -> {
        add(out, transform.getTranslation());
        add(out, transform.getRotation());
      }
      case Record record -> components(out, record);
      default ->
          throw new IllegalArgumentException(
              "no fingerprint for a " + value.getClass().getName() + ": teach Fingerprint it");
    }
  }

  private static void components(StringBuilder out, Record record) {
    out.append(record.getClass().getSimpleName()).append('(');
    for (RecordComponent component : record.getClass().getRecordComponents()) {
      out.append(component.getName()).append('=');
      try {
        add(out, component.getAccessor().invoke(record));
      } catch (IllegalAccessException | InvocationTargetException e) {
        throw new IllegalStateException(e);
      }
      out.append(';');
    }
    out.append(')');
  }

  private static void quaternion(StringBuilder out, Quaternion turn) {
    numbers(out, "Quaternion", turn.getW(), turn.getX(), turn.getY(), turn.getZ());
  }

  private static void numbers(StringBuilder out, String type, double... numbers) {
    out.append(type).append('(');
    for (double number : numbers) {
      bits(out, number);
      out.append(',');
    }
    out.append(')');
  }

  private static void bits(StringBuilder out, double number) {
    out.append(Long.toHexString(Double.doubleToRawLongBits(number)));
  }
}
