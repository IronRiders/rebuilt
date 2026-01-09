package org.ironriders.drive;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;

/** Constants for the drive subsystem. */
public class DriveConstants {

  public static final double ROBOT_WIDTH = 37; // Front-back
  public static final double ROBOT_LENGTH = 37; // Side-side

  // Ports, IDs, Configs, etc.
  public static final int PRIMARY_CONTROLLER_PORT = 0;
  public static final int KEYPAD_CONTROLLER_PORT = 2;
  public static final int TERTIARY_CONTROLLER_PORT = 1;

  public static final double PATHFIND_CANCEL_THRESHOLD = 0.3; // 0-1, controller input

  public static final File SWERVE_JSON_DIRECTORY =
      new File(Filesystem.getDeployDirectory(), "swerve");

  public static final PPHolonomicDriveController HOLONOMIC_CONFIG =
      new PPHolonomicDriveController(
          new PIDConstants(2.5, 0.05, 0.0), // Translation PID
          new PIDConstants(10.0, 0.2, 0.0) // Rotation PID
          );

  // Mathematical Constants
  public static final double TRANSLATION_CONTROL_EXPONENT = 3.0;
  public static final double TRANSLATION_CONTROL_DEADBAND = 0.8;
  public static final double ROTATION_CONTROL_EXPONENT = 3.0;
  public static final double ROTATION_CONTROL_DEADBAND = 0.8;

  public static final double SWERVE_DRIVE_MAX_SPEED = 6; // m/s
  public static final double SWERVE_MAXIMUM_ANGULAR_VELOCITY = Math.PI * 2.5; // rad/s

  public static final double SWERVE_MAXIMUM_SPEED_AUTO = 1.0; // m/s
  public static final double SWERVE_MAXIMUM_ACCELERATION_AUTO = // TODO: This does nothing!!
      SWERVE_MAXIMUM_SPEED_AUTO / 2; // m/s^2
  public static final double SWERVE_MAXIMUM_ANGULAR_VELOCITY_AUTO = Math.PI * 4; // rad/s
  public static final double SWERVE_MAXIMUM_ANGULAR_ACCELERATION_AUTO =
      SWERVE_MAXIMUM_ANGULAR_VELOCITY_AUTO / 2; // rad/s^2

  public static final double JOG_DISTANCE_INCHES = 0.5;
  public static final double JOG_SPEED = .25;
  public static final double VISION_P = 0.1;

  public static final double VISION_I = 0.05;
  public static final double VISION_D = 0;
  public static final double VISION_ROTATION_MAX_SPEED =
      2; // rad/s (2 is about a third of a rotation i think)
  public static final String VISION_CAMERA = "main";
}
