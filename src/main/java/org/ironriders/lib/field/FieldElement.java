package org.ironriders.lib.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Stream;

// TODO: Update to rebuilt

/**
 * Representation of an element on the field.
 * <hr />
 * Includes:
 * <ul>
 * <li>Position</li>
 * <li>Function</li>
 * <li>Utility routines for retrieving.</li>
 * </ul>
 */
public class FieldElement {

  /** The type of elements on the field. */
  public enum ElementType {
    HUB,
    BUMP,
    TRENCH,
    TOWER,
    OUTPOST,
    DEPOT;
  }

  /**
   * Generic (alliance-independent) game element identifiers. There should be 1
   * for each place with an april tag
   * <hr />
   * Each of these has an ID and an
   * {@linkplain org.ironriders.lib.field.FieldElement.ElementType ElementType}.
   */
  public enum Position {
    OUTPOST(0, ElementType.OUTPOST),
    DEPOT(1, ElementType.DEPOT),
    TOWER_OUTPOST_SIDE(2, ElementType.TOWER),
    TOWER_DEPOT_SIDE(3, ElementType.TOWER),
    TOWER_CENTER(4, ElementType.TOWER),
    BUMP_OUTPOST_SIDE(5, ElementType.BUMP),
    BUMP_DEPOT_SIDE(6, ElementType.BUMP),
    TRENCH_OUTPOST_SIDE(7, ElementType.TRENCH),
    TRENCH_DEPOT_SIDE(8, ElementType.TRENCH);

    public final int ID;
    public final ElementType TYPE;

    Position(int id, ElementType type) {
      this.ID = id;
      this.TYPE = type;
    }
  }

  public final Position position;
  public final ElementType TYPE;
  public final Pose3d pose;
  public final String name;

  // TODO: update element tag ID's for rebuilt
  private static HashMap<Position, Integer> BLUE_TAG_POSITIONS = new HashMap<>(Map.of(
      Position.OUTPOST, 0,
      Position.DEPOT, 0,
      Position.TOWER_OUTPOST_SIDE, 0,
      Position.TOWER_DEPOT_SIDE, 0,
      Position.TOWER_CENTER, 0,
      Position.BUMP_OUTPOST_SIDE, 0,
      Position.BUMP_DEPOT_SIDE, 0,
      Position.TRENCH_OUTPOST_SIDE, 0,
      Position.TRENCH_DEPOT_SIDE, 0)); // Integer is the April Tag ID for that position
  private static HashMap<Position, Integer> RED_TAG_POSITIONS = new HashMap<>(Map.of(
      Position.OUTPOST, 0,
      Position.DEPOT, 0,
      Position.TOWER_OUTPOST_SIDE, 0,
      Position.TOWER_DEPOT_SIDE, 0,
      Position.TOWER_CENTER, 0,
      Position.BUMP_OUTPOST_SIDE, 0,
      Position.BUMP_DEPOT_SIDE, 0,
      Position.TRENCH_OUTPOST_SIDE, 0,
      Position.TRENCH_DEPOT_SIDE, 0)); // Integer is the April Tag ID for that position

  private static List<FieldElement> BLUE_ELEMENTS = loadElements(DriverStation.Alliance.Blue, BLUE_TAG_POSITIONS);
  private static List<FieldElement> RED_ELEMENTS = loadElements(DriverStation.Alliance.Red, RED_TAG_POSITIONS);

  private FieldElement(Position element, Pose3d pose) {
    this.position = element;
    this.TYPE = element.TYPE;
    this.name = element.name();
    this.pose = pose;
  }

  /**
   * Retrieve all elements for an alliance.
   * 
   * @param alliance The alliance to retrieve elements for. If this is empty, will
   *                 return an empty list.
   * @return A collection of field elements for the given alliance. Uses
   *         {@linkplain org.ironriders.lib.field.FieldElement#loadElements(edu.wpi.first.wpilibj.DriverStation.Alliance, int[])
   *         loadElements} to generate.
   */
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

  /**
   * Retrieve the closest alliance element of a desired type.
   * 
   * @param pose the position to check from
   * @param type the type of element to find, from
   *             {@linkplain org.ironriders.lib.field.FieldElement.ElementType
   *             ElementType} enum
   * @return the nearest element of the given type to the given position
   */
  public static Optional<FieldElement> nearestTo(Pose2d pose, ElementType type) {
    return findNearest(pose, Optional.of(type));
  }

  /** Find a special alliance element. */
  public static Optional<FieldElement> nearestTo(Pose2d pose) {
    return findNearest(pose, Optional.empty());
  }

  /**
   * Load elements for an alliance from their AprilTag IDs.
   * 
   * @param alliance the {@linkplain edu.wpi.first.wpilibj.DriverStation.Alliance
   *                 Alliance} to load for
   * @param tags     A map of
   *                 {@linkplain org.ironriders.lib.field.FieldElement.Position
   *                 Position's}, and the value is the AprilTag at that position
   *                 Position
   * @return a list of loaded field elements
   */
  private static List<FieldElement> loadElements(DriverStation.Alliance alliance, HashMap<Position, Integer> tags) {
    return Stream.of(Position.values())
        .map(
            element -> {
              var pose = FieldUtils.FIELD_LAYOUT.getTagPose(tags.get(element));
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
      if (type.isPresent() && element.TYPE != type.get()) { // Skips the element if its not the right type
        continue;
      }

      double thisDistance = pose.getTranslation().getDistance(element.pose.toPose2d().getTranslation());
      if (found.isEmpty() || distance > thisDistance) { // if its the first itoration and found hasn't been set, will
                                                        // set found to the first element
        distance = thisDistance;
        found = Optional.of(element);
      }
    }

    return found;
  }
}
