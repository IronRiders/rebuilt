package org.ironriders.drive;

import java.io.File;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Filesystem;

/** Constants for the drive subsystem. */
public class DriveConstants {
    public static final File SWERVE_JSON_DIRECTORY = new File(Filesystem.getDeployDirectory(), "swerve");

    public static final PPHolonomicDriveController HOLONOMIC_CONFIG = new PPHolonomicDriveController(
            new PIDConstants(10.0, 0.0, 0.0), // Translation PID
            new PIDConstants(0.1, 0.0, 0.0) // Rotation PID
    );

    public static final double ROTATE_TO_TARGET_P = 10;
    public static final double ROTATE_TO_TARGET_I = 0;
    public static final double ROTATE_TO_TARGET_D = 0;

    public static final double SIM_ROTATE_TO_TARGET_P = 3;
    public static final double SIM_ROTATE_TO_TARGET_I = 0;
    public static final double SIM_ROTATE_TO_TARGET_D = 0;

    public static final Constraints ROTATION_CONSTRAINTS = new Constraints(Math.PI, Math.PI / 1.2); // Radians

    public static final double TRANSLATION_CONTROL_EXPONENT = 3.0;
    public static final double TRANSLATION_CONTROL_DEADBAND = 0.8;
    public static final double ROTATION_CONTROL_EXPONENT = 3.0;
    public static final double ROTATION_CONTROL_DEADBAND = 0.8;

    public static final double SWERVE_MAX_TRANSLATION_TELEOP = 7; // m/s
    public static final double SWERVE_MAX_ANGULAR_TELEOP = Math.PI / 1; // rad/s

    public static final double SWERVE_MAX_TRANSLATION_PATHFIND = 4; // m/s
    public static final double SWERVE_MAX_ANGULAR_PATHFIND = Math.PI / 1.2; // rad/s

    public static final double SWERVE_MAX_TRANSLATION_ACCEL_PATHFIND = SWERVE_MAX_TRANSLATION_PATHFIND / 1.2;
    public static final double SWERVE_MAX_ANGULAR_ACCEL_PATHFIND = SWERVE_MAX_ANGULAR_PATHFIND / 1.2;

    public static final PathConstraints PATHFIND_CONSTRAINTS = new PathConstraints(SWERVE_MAX_TRANSLATION_PATHFIND,
            SWERVE_MAX_TRANSLATION_ACCEL_PATHFIND, SWERVE_MAX_ANGULAR_PATHFIND, SWERVE_MAX_ANGULAR_ACCEL_PATHFIND);

    public static final double DRIVE_OVERRIDE_THRESHOLD = 0.3;

    public static final int CONTROLLER_PRIMARY_PORT = 0;
}
