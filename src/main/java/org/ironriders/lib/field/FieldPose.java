/*package org.ironriders.lib.field;
*
*import edu.wpi.first.math.geometry.Pose2d;
*import edu.wpi.first.math.geometry.Rotation2d;
*import edu.wpi.first.math.geometry.Translation2d;
*import edu.wpi.first.units.measure.Units;
*import edu.wpi.first.units.measure.Distance;
*
* // TODO: Update to rebuilt
*
* 
* public class FieldPose {
*
* 
*  public static class Station extends FieldPose {
*
* 
*    public final int slot;
*
*    public Station(FieldElement element, int slot) {
*      super(element);
*      this.slot = slot;
*    }
*
*    @Override
*    protected Distance getYOffset() {
*      return STATION_SLOT_SPACING.times(slot - STATION_SLOT_COUNT / 2).minus(INTAKE_OFFSET.div(2));
*    }
*  }
*
*  
*  public static class Reef extends FieldPose {
*
* 
*    public final Side pole;
*
*   
*    public final Level level;
*
*    public Reef(FieldElement element, Side pole, Level level) {
*      super(element);
*      this.pole = pole;
*      this.level = level;
*    }
*
*    @Override
*    protected Distance getYOffset() {
*      return this.pole == Side.Left ? Units.Inches.of(0) : REEF_POLE_SPACING;
*    }
*
*    @Override
*    protected Distance getXOffset() {
*      return super.getXOffset().plus(Units.Inches.of(-7));
*    }
*  }
*
*  static final Distance ROBOT_LENGTH = Units.Inches.of(37);
*  static final Distance INTAKE_OFFSET = Units.Inches.of(7);
*  static final Distance STATION_SLOT_SPACING = Units.Inches.of(8);
*  public static final int STATION_SLOT_COUNT = 9;
*  static final Distance REEF_POLE_SPACING = Units.Inches.of(-12.94);
*
*  
*  public final FieldElement element;
*
* 
*  public enum Side {
*    Left,
*    Right,
*  }
*
*
*  public enum Level {
*    L1,
*    L2,
*    L3,
*    L4,
*  }
*
*  public FieldPose(FieldElement element) {
*    this.element = element;
*  }
*
* 
*  public Pose2d toPose2d() {
*    final var elementPose = this.element.pose.toPose2d();
*
*    final var robotRotation = elementPose.getRotation().rotateBy(Rotation2d.k180deg);
*
*    final var zeroAngleRelativeTranslation = new Translation2d(getXOffset(), getYOffset());
*
*    final var relativeTranslation = zeroAngleRelativeTranslation.rotateBy(robotRotation);
*
*    final var robotTranslation = elementPose.getTranslation().plus(relativeTranslation);
*
*    return new Pose2d(robotTranslation, robotRotation);
*  }
*
*  protected Distance getYOffset() {
*    return Units.Inches.of(0);
*  }
*
*  protected Distance getXOffset() {
*    return ROBOT_LENGTH.div(-2);
*  }
*}
*/