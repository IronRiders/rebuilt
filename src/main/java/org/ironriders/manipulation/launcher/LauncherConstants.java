package org.ironriders.manipulation.launcher;

import org.ironriders.drive.DriveConstants;
import org.ironriders.lib.field.FieldPositions;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/** Constants for the {@link LauncherSubsystem} */
public class LauncherConstants {
  public static final double LAUNCHER_HIGHT = Units.inchesToMeters(20.0);

  public static final double G = 9.8; // m/s

  public static final double HEIGHT_DIFFERENCE_HUB_TO_LAUNCHER = Units.inchesToMeters(FieldPositions.Hub.HUB_TOP.getZ())
      - LAUNCHER_HIGHT; // m

  public static final double TARGET_BALL_VELOCITY = 11.4; // m/s (see https://www.reca.lc/flywheel)

  public static final double FLYWHEEL_MAX_VEL = 6065; // (see https://www.reca.lc/flywheel)
  public static final double FLYWHEEL_MAX_ACC = FLYWHEEL_MAX_VEL / 2;

  public static final double LAUNCHER_HOOD_MAX_VEL = 180; // TODO
  public static final double LAUNCHER_HOOD_MAX_ACC = LAUNCHER_HOOD_MAX_VEL / 2; // TODO

  public static final double FLYWHEEL_P = 0.5;
  public static final double FLYWHEEL_I = 0.0;
  public static final double FLYWHEEL_D = 0.0;

  public static final double FLYWHEEL_TOLERANCE = 40; // rpm

  public static final double LAUNCHER_P = 0.5;
  public static final double LAUNCHER_I = 0.0;
  public static final double LAUNCHER_D = 0.0;

  public static final double LAUNCHER_TOLERANCE = 0.1;

  public static final double MIN_ROTATION = Math.toRadians(30);
  public static final double MAX_ROTATION = Math.toRadians(90);

  public static final double LAUNCHER_STOW_POSITION = MIN_ROTATION;

  public static final double SPINDOWN_TIME = 2; // time to automatically go into idle mode after, currently TODO

  public enum State {
    READY,
    IDLE,
    STOW;
  }

  public enum TargetingMode {
    OUT_OF_RANGE,
    ROTATE_TOWARDS,
    FULL_CONTROL;
  }
}
