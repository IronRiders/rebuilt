package org.ironriders.drive;

import java.io.File;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.Filesystem;

/** Constants for the drive subsystem. */
public class DriveConstants {
    public enum Controller {
        DRIVER(),
        VISION();

        Controller() {
        }
    }

    public static final File SWERVE_JSON_DIRECTORY = new File(Filesystem.getDeployDirectory(), "swerve");

    public static final PPHolonomicDriveController HOLONOMIC_CONFIG = new PPHolonomicDriveController(
            new PIDConstants(1.0, 0.0, 0.0), // Translation PID
            new PIDConstants(1.0, 0.0, 0.0) // Rotation PID
    );

    public static final double TRANSLATION_CONTROL_EXPONENT = 3.0;
    public static final double TRANSLATION_CONTROL_DEADBAND = 0.8;
    public static final double ROTATION_CONTROL_EXPONENT = 3.0;
    public static final double ROTATION_CONTROL_DEADBAND = 0.8;

    public static final double SWERVE_MAX_TRANSLATION_TELEOP = 0.6; // m/s
    public static final double SWERVE_MAX_ANGULAR_TELEOP = Math.PI / 3; // rad/s

    public static final double SWERVE_MAX_TRANSLATION_PATHFIND = 0.3; // m/s
    public static final double SWERVE_MAX_ANGULAR_PATHFIND = Math.PI / 3; // rad/s

    public static final double SWERVE_MAX_TRANSLATION_ACCEL_PATHFIND = SWERVE_MAX_TRANSLATION_PATHFIND / 2;
    public static final double SWERVE_MAX_ANGULAR_ACCEL_PATHFIND = SWERVE_MAX_ANGULAR_PATHFIND / 2;

    public static final PathConstraints PATHFIND_CONSTRAINTS = new PathConstraints(SWERVE_MAX_TRANSLATION_PATHFIND,
            SWERVE_MAX_TRANSLATION_ACCEL_PATHFIND, SWERVE_MAX_ANGULAR_PATHFIND,
            SWERVE_MAX_ANGULAR_ACCEL_PATHFIND);

    public static final double DRIVE_OVERRIDE_THRESHOLD = 0.3; // Input threshold to override vision and drive
                                                               // anyway.

    
    public static final int CONTROLLER_PRIMARY_PORT = 0;
    public static final int CONTROLLER_SECONDARY_PORT = 1;
}
