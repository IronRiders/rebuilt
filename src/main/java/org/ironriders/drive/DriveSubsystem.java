package org.ironriders.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static org.ironriders.manipulation.launcher.LauncherConstants.ROTATE_TO_TARGET_D;
import static org.ironriders.manipulation.launcher.LauncherConstants.ROTATE_TO_TARGET_I;
import static org.ironriders.manipulation.launcher.LauncherConstants.ROTATE_TO_TARGET_P;
import static org.ironriders.manipulation.launcher.LauncherConstants.ROTATION_CONSTRAINTS;

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

    public static boolean isFacingLauncher = true;

    public static AtomicBoolean isDriving = new AtomicBoolean(false);

    private static Command pathfindCommand;

    private static ProfiledPIDController rotationPid = new ProfiledPIDController(ROTATE_TO_TARGET_P, ROTATE_TO_TARGET_I,
            ROTATE_TO_TARGET_D, ROTATION_CONSTRAINTS);

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
        setRotationGoal(180);
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();

        if (!isDriving.get() && isFacingLauncher) {
            driveRotate(new Translation2d());
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
        if (isFacingLauncher) {
            driveRotate(translation);
        } else {
            swerveDrive.drive(
                    translation.times(driveInvert ? -1 : 1),
                    rotation * (rotationInvert ? -1 : 1),
                    fieldRelative,
                    false);
        }

        isDriving.getAndSet(false);
    }

    public static void driveRotate(Translation2d translation) {
        swerveDrive.drive(translation.times(driveInvert ? -1 : 1),
                rotationPid.calculate(getRotation().in(Radians)) * (rotationInvert ? -1 : 1),
                false,
                true);
    }

    public static Angle getRotation() {
        return swerveDrive.getGyroRotation3d().toRotation2d().getMeasure();
    }

    public static Command pathfindToPose(Pose2d target, PathConstraints constraints) {
        pathfindCommand = AutoBuilder.pathfindToPose(target, constraints);
        return pathfindCommand.withName("Pathfind to " + target.getX() + ", " + target.getY());
    }

    public static Command pathfindToPose(Pose2d target) {
        pathfindCommand = AutoBuilder.pathfindToPose(target, DriveConstants.PATHFIND_CONSTRAINTS);
        return pathfindCommand.withName("Pathfind to " + target.getX() + ", " + target.getY());
    }

    public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints constraints) {
        pathfindCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints);

        return pathfindCommand.withName("Pathfind to " + path.name);
    }

    public static Command pathfindThenFollowPath(PathPlannerPath path) {
        pathfindCommand = AutoBuilder.pathfindThenFollowPath(
                path, DriveConstants.PATHFIND_CONSTRAINTS);

        return pathfindCommand.withName("Pathfind to " + path.name);
    }

    public static Optional<Command> pathfindToTag(int id) {
        var tag = VisionSubsystem.fieldLayout.getTagPose(id).orElse(null);
        if (tag == null) {
            return Optional.empty();
        }

        return Optional.of(pathfindToPose(Utils.flattenPose3d(tag)));
    }

    public static void startPathfind() {
        if (pathfindCommand != null) {
            CommandScheduler.getInstance().schedule(pathfindCommand);
        }
    }

    public static void cancelPathfind() {
        if (pathfindCommand != null) {
            pathfindCommand.cancel();
        }
    }

    public void setRotationGoal(double goal) {
        rotationPid.setGoal(Math.toRadians(goal));
    }

    /** Fetch the DriveCommands instance */
    public DriveCommands getCommands() {
        return commands;
    }

    /** Fetch the SwerveDrive instance */
    public static SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public static void setSpeedMax(double max) {
        swerveDrive.setMaximumAllowableSpeeds(max, DriveConstants.SWERVE_MAX_ANGULAR_TELEOP);
    }

    /** Where is the robot? */
    public Pose2d getPose() {
        return DriveSubsystem.swerveDrive.getPose();
    }

    public void resetRotation() {
        Pigeon2 pigeon2 = new Pigeon2(9);
        swerveDrive.resetOdometry(
                new Pose2d(
                        swerveDrive.getPose().getTranslation(),
                        new Rotation2d(
                                pigeon2.getYaw(true).waitForUpdate(1).getValueAsDouble() * (Math.PI / 180f))));
        pigeon2.close();
    }

    public void resetOdometry(Pose2d pose2d) {
        swerveDrive.resetOdometry(new Pose2d(pose2d.getTranslation(), new Rotation2d(0)));
    }

    public void switchRotation() {
        rotationInvert = !rotationInvert;
    }

    public void switchDrive() {
        driveInvert = !driveInvert;
    }
}