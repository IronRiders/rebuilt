package org.ironriders.lib.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;

import org.ironriders.lib.Utils;
import org.ironriders.vision.VisionSubsystem;

/**
 * Representation of an element on the field.
 *
 * <p>Includes pose, function, and various utility routines for retrieving.
 */
public class FieldElement {

  /** The type of elements on the field. */
  public enum ElementType {
    OUTPOST,
    HUB,
    TOWER,
    DEPOT,
    BUMP,
    TRENCH
  }

  /** Generic (alliance-independent) element identifiers. */
  public enum Position {
    LEFT_OUTPOST(0, ElementType.OUTPOST),
    RIGHT_OUTPOST(1, ElementType.OUTPOST),
    HUB_FRONT(2, ElementType.HUB),
    HUB_BACK(3, ElementType.HUB),
    RIGHT_TOWER(4, ElementType.TOWER),
    LEFT_TOWER(5, ElementType.TOWER),
    RIGHT_DEPOT(6, ElementType.DEPOT),
    LEFT_DEPOT(7, ElementType.DEPOT),
    BUMP_FRONT_RIGHT(8, ElementType.BUMP),
    BUMP_FRONT_LEFT(9, ElementType.BUMP),
    BUMP_BACK_RIGHT(10, ElementType.BUMP),
    BUMP_BACK_LEFT(11, ElementType.BUMP),
    TRENCH_FRONT_RIGHT(12, ElementType.TRENCH),
    TRENCH_FRONT_LEFT(13, ElementType.TOWER),
    TRENCH_BACK_RIGHT(14, ElementType.TRENCH),
    TRENCH_BACK_LEFT(15, ElementType.TRENCH);

    public final int id;
    public final ElementType type;

    Position(int id, ElementType type) {
      this.id = id;
      this.type = type;
    }
  }

  public final Position position;
  public final ElementType type;
  public final Pose3d pose;
  public final String name;

  private static int[] BLUE_TAGS = Utils.everyIntInRange(17, 32);

  private static int[] RED_TAGS = Utils.everyIntInRange(1, 16);

  private static List<FieldElement> BLUE_ELEMENTS =
      loadElements(DriverStation.Alliance.Blue, BLUE_TAGS);
  private static List<FieldElement> RED_ELEMENTS =
      loadElements(DriverStation.Alliance.Red, RED_TAGS);

  private FieldElement(Position element, Pose3d pose) {
    this.position = element;
    this.type = element.type;
    this.name = element.name();
    this.pose = pose;
  }

  /** Retrieve all elements for an alliance. */
  public static Collection<FieldElement> of(Optional<DriverStation.Alliance> alliance) {
    if (alliance.isEmpty()) {
      return new ArrayList<FieldElement>();
    }

    if (alliance.get().equals(DriverStation.Alliance.Red)) {
      return RED_ELEMENTS;
    }

    return BLUE_ELEMENTS;
  }

  /** Retrieve a specific alliance element. */
  public static Optional<FieldElement> of(Position element) {
    for (var allianceElement : of(DriverStation.getAlliance())) {
      if (allianceElement.position == element) {
        return Optional.of(allianceElement);
      }
    }
    return Optional.empty();
  }

  /** Retrieve the closest alliance element of a desired type. */
  public static Optional<FieldElement> nearestTo(Pose2d pose, ElementType type) {
    return findNearest(pose, Optional.of(type));
  }

  /** Find a special alliance element. */
  public static Optional<FieldElement> nearestTo(Pose2d pose) {
    return findNearest(pose, Optional.empty());
  }

  private static List<FieldElement> loadElements(DriverStation.Alliance alliance, int[] tags) {
    return Stream.of(Position.values())
        .map(
            element -> {
              var pose = VisionSubsystem.fieldLayout.getTagPose(tags[element.id]);
              if (pose.isEmpty()) {
                Optional.empty();
              }
              return Optional.of(new FieldElement(element, pose.get()));
            })
        .filter(Optional::isPresent)
        .map(Optional::get)
        .toList();
  }

  private static Optional<FieldElement> findNearest(Pose2d pose, Optional<ElementType> type) {
    double distance = -1;
    Optional<FieldElement> found = Optional.empty();

    for (var element : of(DriverStation.getAlliance())) {
      if (type.isPresent() && element.type != type.get()) {
        continue;
      }

      double thisDistance =
          pose.getTranslation().getDistance(element.pose.toPose2d().getTranslation());
      if (found.isEmpty() || distance > thisDistance) {
        distance = thisDistance;
        found = Optional.of(element);
      }
    }

    return found;
  }
}