package org.ironriders.drive;

import java.io.IOException;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

import org.ironriders.core.RobotContainer;
import org.ironriders.core.TargetingControl;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    public static boolean PIDRotation = false;

    public static AtomicBoolean isDriving = new AtomicBoolean(false);

    public static Command pathfindingCommand = new InstantCommand();

    private static ProfiledPIDController rotationPid;

    public static Pigeon2 pigeon = new Pigeon2(11);

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

        if (SwerveDriveTelemetry.isSimulation) {
            rotationPid = new ProfiledPIDController(
                    DriveConstants.SIM_ROTATE_TO_TARGET_P,
                    DriveConstants.SIM_ROTATE_TO_TARGET_I,
                    DriveConstants.SIM_ROTATE_TO_TARGET_D, DriveConstants.ROTATION_CONSTRAINTS);
        } else {
            rotationPid = new ProfiledPIDController(
                    DriveConstants.ROTATE_TO_TARGET_P,
                    DriveConstants.ROTATE_TO_TARGET_I,
                    DriveConstants.ROTATE_TO_TARGET_D, DriveConstants.ROTATION_CONSTRAINTS);
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

        rotationPid.reset(getRotation());

        rotationPid.enableContinuousInput(0, Math.PI * 2);

        rotationPid.setTolerance(0.05);
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();

        TargetingControl.update();

        if (Math.abs(RobotContainer.primaryController.getRightX()) > DriveConstants.DRIVE_OVERRIDE_THRESHOLD) {
            RobotContainer.revertToSafeDefaults();
        }

        double leftMag = Math.hypot(RobotContainer.primaryController.getLeftX(),
                RobotContainer.primaryController.getLeftY());
        if (leftMag > DriveConstants.DRIVE_OVERRIDE_THRESHOLD) {
            cancelPathfind();
        }

        publish("PID", rotationPid);
        publish("Yaw", getRotation());
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

        if (PIDRotation) {
            swerveDrive.drive(translation.times(driveInvert ? -1 : 1),
                    rotationPid.calculate(getRotation()) * (SwerveDriveTelemetry.isSimulation ? 1 : -1),
                    fieldRelative,
                    false);
        } else {
            swerveDrive.drive(
                    translation.times(driveInvert ? -1 : 1),
                    rotation * (rotationInvert ? -1 : 1),
                    fieldRelative,
                    false);
        }

        isDriving.getAndSet(false);
    }

    /**
     * @return The robot's current rotation in radians.
     */
    public static double getRotation() {
        return getPose().getRotation().getRadians();
    }

    /** Where is the robot? */
    public static Pose2d getPose() {
        return DriveSubsystem.swerveDrive.getPose();
    }

    /** Where is the robot in 3d? */
    public static Pose3d getPose3d() {
        return Utils.expandPose2d(getPose());
    }

    public static void setPIDRotationControl(boolean shouldPIDRotate) {
        PIDRotation = shouldPIDRotate;
        if (!PIDRotation) {
            rotationPid.reset(getRotation());
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
     * Command to pathfind to a given pose.
     *
     * @param pose The pose to pathfind to.
     *
     */
    public static Command pathfindToPose(Pose2d pose) {
        pathfindingCommand = AutoBuilder.pathfindToPose(pose, DriveConstants.PATHFIND_CONSTRAINTS)
                .andThen(Commands.runOnce(() -> rotationPid.reset(getRotation())));
        return pathfindingCommand;
    }

    /**
     * Command to pathfind to a given pose and face a target.
     * 
     * @param pose   The pose to pathfind to.
     * @param target The target to face.
     */
    public static Command pathfindToPoseAndAimAt(Pose2d pose, Pose2d target) {
        pathfindingCommand = AutoBuilder.pathfindToPose(
                new Pose2d(pose.getTranslation(), new Rotation2d(Utils.getAngleToPointRadians(pose, target) + Math.PI)),
                DriveConstants.PATHFIND_CONSTRAINTS).andThen(Commands.runOnce(() -> rotationPid.reset(getRotation())));

        return pathfindingCommand;
    }

    /**
     * Command to pathfind to the start of a given path then follow that path.
     */
    public static Command pathfindThenFollowPath(PathPlannerPath path) {
        pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, DriveConstants.PATHFIND_CONSTRAINTS)
                .andThen(Commands.runOnce(() -> rotationPid.reset(getRotation())));
        return pathfindingCommand;
    }

    /**
     * Command to pathfind to the start of a given path then follow the flipped
     * version of that path.
     */
    public static Command pathfindThenFollowFlippedPath(PathPlannerPath path) {
        path = path.flipPath();
        pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, DriveConstants.PATHFIND_CONSTRAINTS)
                .andThen(Commands.runOnce(() -> rotationPid.reset(getRotation())));
        return pathfindingCommand;
    }

    /**
     * Command to figure out if the distance to the start point of the flipped
     * version of the provided path is closer than the normal version, and if so
     * follow the flipped version.
     * 
     * TODO: !Uses distance as the crow flies, not path distance to start point!
     */
    public static Command pathfindThenFlipPathIfBetterThenFollow(PathPlannerPath path) {
        if (Utils.distanceToPose2d(path.getPathPoses().get(0), getPose()) < Utils
                .distanceToPose2d(path.flipPath().getPathPoses().get(0), getPose())) {
            path = path.flipPath();
        }

        pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, DriveConstants.PATHFIND_CONSTRAINTS)
                .andThen(Commands.runOnce(() -> rotationPid.reset(getRotation())));
        return pathfindingCommand;
    }

    /*
     * Cancel the current pathfinding operation.
     */
    public static void cancelPathfind() {
        if (pathfindingCommand.isScheduled()) {
            pathfindingCommand.cancel();
            rotationPid.reset(getRotation());
        }
    }

    public static Optional<PathPlannerPath> loadPath(String fileName) {
        try {
            return Optional.of(PathPlannerPath.fromPathFile(fileName));
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.printf("Error loading path %s: ", fileName);
            e.printStackTrace();
            return Optional.empty();
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

    public static void resetRotation() {
        pigeon.reset();
        resetOdometry(swerveDrive.getPose());
        rotationPid.reset(0);
    }

    /**
     * Sets the robot's odometry to a given pose with rotation at 0.
     * 
     * @param pose2d The pose to reset the odometry to.
     */
    public static void resetOdometry(Pose2d pose2d) {
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