package org.ironriders.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Optional;
import java.util.function.Supplier;
import org.ironriders.core.ElevatorWristControl.ElevatorLevel;
import org.ironriders.lib.field.FieldPose;

/** Current robot state required by multiple subsystems. TODO: this is practicly unused now */
public class GameState {

  public static boolean controlInverted;

  private static Field2d field = new Field2d();
  private static Supplier<Optional<Pose2d>> robotPose = () -> Optional.empty();
  private static Supplier<Optional<FieldPose>> targetRobotPose = () -> Optional.empty();

  // this represents our current elevator target
  private static ElevatorLevel target = ElevatorLevel.DOWN;

  private GameState() {}

  public static Field2d getField() {
    return field;
  }

  public static void setField(Field2d field) {
    GameState.field = field;
  }

  public static Optional<Pose2d> getRobotPose() {
    return robotPose.get();
  }

  public static void setRobotPose(Supplier<Optional<Pose2d>> robotPose) {
    GameState.robotPose = robotPose;
  }

  public static Optional<FieldPose> getTargetRobotPose() {
    return targetRobotPose.get();
  }

  public static void setTargetRobotPose(Supplier<Optional<FieldPose>> robotPose) {
    GameState.targetRobotPose = robotPose;
  }

  public static ElevatorLevel getTarget() {
    return target;
  }

  public static void setTarget(ElevatorLevel target) {
    GameState.target = target;
  }

  public static boolean getInvertControl() {
    return controlInverted;
  }

  public static void invertControl() {
    GameState.controlInverted = !GameState.controlInverted;
  }
}
