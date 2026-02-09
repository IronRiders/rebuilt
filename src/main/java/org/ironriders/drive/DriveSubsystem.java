package org.ironriders.drive;

import static edu.wpi.first.units.Units.Radians;

import java.io.IOException;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.vision.VisionSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The DriveSubsystem encompasses everything that the Swerve Drive needs to
 * function. It keeps track
 * of the robot's position and angle, and uses the controller input to figure
 * out how the individual
 * modules need to turn and be angled.
 */
public class DriveSubsystem extends IronSubsystem {
    private final DriveCommands commands;

    private static SwerveDrive swerveDrive;
    private static boolean rotationInvert = false;
    private static boolean driveInvert = false;

    public static boolean PIDAlign = false;

    public static AtomicBoolean isDriving = new AtomicBoolean(false);

    private static Command pathfindCommand;

    private static ProfiledPIDController rotationPid = new ProfiledPIDController(DriveConstants.ROTATE_TO_TARGET_P, DriveConstants.ROTATE_TO_TARGET_I,
            DriveConstants.ROTATE_TO_TARGET_D, DriveConstants.ROTATION_CONSTRAINTS);

    public DriveSubsystem() throws RuntimeException {
        try {
            swerveDrive = new SwerveParser(DriveConstants.SWERVE_JSON_DIRECTORY) // YAGSL reads from the deploy/swerve
                    // directory.
                    .createSwerveDrive(DriveConstants.SWERVE_MAX_TRANSLATION_TELEOP);
        } catch (IOException e) { // instancing SwerveDrive can throw an error, so we need to catch that.
            throw new RuntimeException("Error configuring swerve drive", e);
        }

        commands = new DriveCommands(this);

        swerveDrive.setHeadingCorrection(false);
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        RobotConfig robotConfig = null;
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Could not load path planner config", e);
        }

        AutoBuilder.configure(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity,
                (speeds, feedforwards) -> swerveDrive.drive(speeds),
                DriveConstants.HOLONOMIC_CONFIG,
                robotConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        rotationPid.reset(0);
        rotationPid.enableContinuousInput(0, Math.PI * 2);
        rotationPid.setTolerance(5);

        // debug setRotationGoal(180);
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();

        if (!isDriving.get() && PIDAlign) {
            drivePID(new Translation2d());
        }
    }

    /**
     * Vrrrrooooooooom brrrrrrrrr BRRRRRR wheeee BRRR brrrr VRRRRROOOOOOM ZOOOOOOM
     * ZOOOOM WAHOOOOOOOOO
     * WAHAHAHHA (Drives given a desired translation and rotation.)
     *
     * @param translation   Desired translation in meters per second.
     * @param rotation      Desired rotation in radians per second.
     * @param fieldRelative If not field relative, the robot will move relative to
     *                      its own rotation.
     */
    public static void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        isDriving.getAndSet(true);

        if (PIDAlign) {
            drivePID(translation);
        } else {
            swerveDrive.drive(
                    translation.times(driveInvert ? -1 : 1),
                    rotation * (rotationInvert ? -1 : 1),
                    fieldRelative,
                    false);
        }

        isDriving.getAndSet(false);
    }

    public static void drivePID(Translation2d translation) {
        swerveDrive.drive(translation.times(driveInvert ? -1 : 1),
                rotationPid.calculate(getRotation().in(Radians)),
                true,
                true);
    }

    /**
     * @return The robot's current rotation.
     */
    public static Angle getRotation() {
        return swerveDrive.getGyroRotation3d().toRotation2d().getMeasure();
    }

    /** Where is the robot? */
    public static Pose2d getPose() {
        return DriveSubsystem.swerveDrive.getPose();
    }

    /*
     * Enable and disable PID rotation control. Set goal using {@link #setRotationGoal()}
     */
    public static void setPIDControl(boolean PIDControl) {
        PIDAlign = PIDControl;

        if (!PIDControl) {
            rotationPid.reset(getRotation().in(Radians));
        }
    }

    /**
     * Sets the PID rotation goal in degrees.
     */
    public static void setRotationGoal(double goal) {
        rotationPid.setGoal(Math.toRadians(goal));
    }

    /**
     * Sets the PID rotation goal in radians.
     */
    public static void setRotationGoalRad(double goal) {
        rotationPid.setGoal(goal);
    }

    /**
     * Pathfinds to a given pose using PathPlanner's pathfinding. See
     * {@link #pathfindToPose()} for more information.
     * 
     * @param target      The {@link Pose2d} to pathfind to.
     * @param constraints The {@link PathConstraints} to pathfind with.
     */
    public static Command pathfindToPose(Pose2d target, PathConstraints constraints) {
        pathfindCommand = AutoBuilder.pathfindToPose(target, constraints);
        return pathfindCommand.withName("Pathfind to " + target.getX() + ", " + target.getY());
    }

    /**
     * Same as {@link #pathfindToPose(Pose2d, PathConstraints) pathfindToPose} but
     * with default constraints.
     * 
     * @param target The {@link Pose2d} to pathfind to.
     * @return A {@link Command} to do the above.
     */
    public static Command pathfindToPose(Pose2d target) {
        pathfindCommand = AutoBuilder.pathfindToPose(target, DriveConstants.PATHFIND_CONSTRAINTS);
        return pathfindCommand.withName("Pathfind to " + target.getX() + ", " + target.getY());
    }

    /**
     * Pathfinds to a given path using PathPlanner's pathfinding.
     * 
     * @param path        The {@link PathPlannerPath} to pathfind with
     * @param constraints The {@link PathConstraints} for pathfinding
     * @return A {@link Command} to do the above with the name "Pathfind to " +
     *         path.name
     */
    public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints constraints) {
        pathfindCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints);

        return pathfindCommand.withName("Pathfind to " + path.name);
    }

    /**
     * Same as {@link #pathfindThenFollowPath(PathPlannerPath, PathConstraints)
     * pathfindThenFollowPath} but with default constraints.
     * 
     * @param path The {@link PathPlannerPath} to follow.
     * @return A {@link Command} to do the above with the name "Pathfind to " +
     *         path.name
     */
    public static Command pathfindThenFollowPath(PathPlannerPath path) {
        pathfindCommand = AutoBuilder.pathfindThenFollowPath(
                path, DriveConstants.PATHFIND_CONSTRAINTS);

        return pathfindCommand.withName("Pathfind to " + path.name);
    }

    /**
     * Pathfinds to a given AprilTag using PathPlanner's pathfinding. Uses
     * {@link #pathfindToPose(Pose2d) pathfindToPose} on the flattened
     * {@link Pose3d} of the {@code tag id}.
     * 
     * @param id The ID of the AprilTag to pathfind to.
     * @return An {@link Optional} containing a {@link Command} to do the above if
     *         the tag exists, or an empty {@link Optional} if it does not.
     */
    public static Optional<Command> pathfindToTag(int id) {
        var tag = VisionSubsystem.fieldLayout.getTagPose(id).orElse(null);
        if (tag == null) {
            return Optional.empty();
        }

        return Optional.of(pathfindToPose(Utils.flattenPose3d(tag)));
    }

    /**
     * Schedules the current pathfind command, if it exists.
     */
    public static void startPathfind() {
        if (pathfindCommand != null) {
            CommandScheduler.getInstance().schedule(pathfindCommand);
        }
    }

    /**
     * Stops the currently running pathfind.
     */
    public static void cancelPathfind() {
        if (pathfindCommand != null) {
            pathfindCommand.cancel();
        }
    }

    /** Fetch the DriveCommands instance */
    public DriveCommands getCommands() {
        return commands;
    }

    /** Fetch the SwerveDrive instance */
    public static SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Sets the maximum translation speed for the swerve drive.
     * 
     * @param max The maximum translation speed in meters per second.
     */
    public static void setSpeedMax(double max) {
        swerveDrive.setMaximumAllowableSpeeds(max, DriveConstants.SWERVE_MAX_ANGULAR_TELEOP);
    }

    /**
     * Opens a {@link Pidgeon2} sensor and gets yaw, waits 1 second, then gets that
     * value as double.
     */
    public void resetRotation() {
        Pigeon2 pigeon2 = new Pigeon2(9);
        swerveDrive.resetOdometry(
                new Pose2d(
                        swerveDrive.getPose().getTranslation(),
                        new Rotation2d(
                                pigeon2.getYaw(true).waitForUpdate(1).getValueAsDouble() * (Math.PI / 180f))));
        pigeon2.close();
    }

    /**
     * Sets the robot's odometry to a given pose with rotation at 0.
     * 
     * @param pose2d The pose to reset the odometry to.
     */
    public void resetOdometry(Pose2d pose2d) {
        swerveDrive.resetOdometry(new Pose2d(pose2d.getTranslation(), new Rotation2d(0)));
    }

    /**
     * Inverts the rotation controls.
     */
    public void switchRotation() {
        rotationInvert = !rotationInvert;
    }

    /**
     * Inverts the drive controls.
     */
    public void switchDrive() {
        driveInvert = !driveInvert;
    }
}