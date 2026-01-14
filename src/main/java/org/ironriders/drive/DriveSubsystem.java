package org.ironriders.drive;

import java.io.IOException;

import static org.ironriders.drive.DriveConstants.GYRO_PORT;
import static org.ironriders.drive.DriveConstants.SWERVE_DRIVE_MAX_SPEED;
import static org.ironriders.drive.DriveConstants.SWERVE_JSON_DIRECTORY;
import static org.ironriders.drive.DriveConstants.SWERVE_MAXIMUM_ANGULAR_VELOCITY;
import static org.ironriders.drive.DriveConstants.VISION_CAMERA;
import static org.ironriders.drive.DriveConstants.VISION_D;
import static org.ironriders.drive.DriveConstants.VISION_I;
import static org.ironriders.drive.DriveConstants.VISION_P;
import static org.ironriders.drive.DriveConstants.VISION_ROTATION_MAX_SPEED;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.RobotUtils;
import org.json.simple.parser.ParseException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The DriveSubsystem encompasses everything that the Swerve Drive needs to function. It keeps track
 * of the robot's position and angle, and uses the controller input to figure out how the individual
 * modules need to turn and be angled.
 */
public class DriveSubsystem extends IronSubsystem {

    private DriveCommands commands;

    private SwerveDrive swerveDrive;
    private boolean rotationInvert = false;
    private boolean driveInvert = false;

    public Command pathfindCommand;
    public double controlSpeedMultipler = 1;
    private boolean enableVision = false;
    private PhotonCamera camera = new PhotonCamera(VISION_CAMERA);
    private PIDController visPidController = new PIDController(VISION_P, VISION_I, VISION_D);
    private double distance = 0;

    /** Initalize drive subsystem. */
    public DriveSubsystem() throws RuntimeException {
        try {
            swerveDrive = new SwerveParser(SWERVE_JSON_DIRECTORY) // YAGSL reads from the
                                                                  // deploy/swerve directory
                    .createSwerveDrive(SWERVE_DRIVE_MAX_SPEED);
        } catch (IOException e) {
            // instancing SwerveDrive can throw an error, so we need to catch that.
            throw new RuntimeException("Error configuring swerve drive", e);
        }

        commands = new DriveCommands(this);

        swerveDrive.setHeadingCorrection(false);
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        RobotConfig robotConfig = null;
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Could not load path planner config", e);
        }

        AutoBuilder.configure(swerveDrive::getPose, swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity,
                (speeds, feedforwards) -> swerveDrive.setChassisSpeeds(new ChassisSpeeds(
                        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                        RobotUtils.clamp(-SWERVE_MAXIMUM_ANGULAR_VELOCITY,
                                SWERVE_MAXIMUM_ANGULAR_VELOCITY, -speeds.omegaRadiansPerSecond))),
                DriveConstants.HOLONOMIC_CONFIG, robotConfig, () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, this);

        visionInit();
    }

    @Override
    public void periodic() {
        visionPeriodic(enableVision);

        // debugPublish("vision has pose", hasPose);
        publish("Drive Inverted?", driveInvert);
        publish("Rotation Inverted?", rotationInvert);
    }

    /**
     * Vrrrrooooooooom brrrrrrrrr BRRRRRR wheeee BRRR brrrr VRRRRROOOOOOM ZOOOOOOM ZOOOOM
     * WAHOOOOOOOOO WAHAHAHHA (Drives given a desired translation and rotation.)
     *
     * @param translation Desired translation in meters per second.
     * @param rotation Desired rotation in radians per second.
     * @param fieldRelative If not field relative, the robot will move relative to its own rotation.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation.times(driveInvert ? -1 : 1),
                rotation * (rotationInvert ? -1 : 1), fieldRelative, false);
    }

    /** Fetch the DriveCommands instance. */
    public DriveCommands getCommands() {
        return commands;
    }

    /** Reset the gyro to 0 degrees. Good for correcting drift */
    public void resetRotation() {
        Pigeon2 pigeon2 = new Pigeon2(GYRO_PORT);
        pigeon2.reset();
        swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPose().getTranslation(), new Rotation2d(
                pigeon2.getYaw(true).waitForUpdate(1).getValueAsDouble() * (Math.PI / 180f))));
        pigeon2.close();
    }

    /** Fetch the SwerveDrive instance. */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public Pose2d getPose() {
        return this.swerveDrive.getPose();
    }

    /** Resets the odometry to the given position. */
    public void resetOdometry(Pose2d pose2d) {
        swerveDrive.resetOdometry(new Pose2d(pose2d.getTranslation(), new Rotation2d(0)));
    }

    public void switchRotation() {
        rotationInvert = !rotationInvert;
    }

    public void switchDrive() {
        driveInvert = !driveInvert;
    }

    public void setSpeed(double speed) {
        controlSpeedMultipler = speed;
    }

    /**
     * Vision Main loop.
     *
     * @param controlsDrive Am I allowed to move?
     */
    private void visionPeriodic(boolean controlsDrive) {
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();

        visPidController.setSetpoint(0);

        if (!results.isEmpty()) {
            // Camera processed a new frame since last time we checked
            // Get the latest frame
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        targetYaw = target.getYaw();
                        // We assume the camera and tag are both at a meter of height, but this is a
                        // very bad
                        // idea as the differance is important. Real nums tbd
                        distance = PhotonUtils.calculateDistanceToTargetMeters(1, 1, 0,
                                target.getPitch());
                        targetVisible = true;
                    }
                }
            }
        }

        publish("Camera sees target", targetVisible);
        publish("Vision can drive", controlsDrive);

        if (targetVisible) {
            // We found our favorite toy! (tag #7)
            publish("Yaw offset", targetYaw);
            double requestedmovement = visPidController.calculate(targetYaw);
            publish("Requested movement", requestedmovement);
            publish("Distance to target", distance);

            if (controlsDrive) {

                if (requestedmovement > VISION_ROTATION_MAX_SPEED) {
                    requestedmovement = VISION_ROTATION_MAX_SPEED;
                }
                if (requestedmovement < -VISION_ROTATION_MAX_SPEED) {
                    requestedmovement = -VISION_ROTATION_MAX_SPEED;
                }

                swerveDrive.drive(new Translation2d(0, 0), requestedmovement * -1, false, true);
            }
        } else {

            if (controlsDrive) {
                // Saftey measure, if vision control is requested but we lose the tag, stop moving.
                // Otherwise we will just keep moving in the previously commanded direction forever
                swerveDrive.drive(new Translation2d(0, 0), 0, false, true);
            }
        }
    }

    /** Initalize vision system. Disables anyone elses control */
    private void visionInit() {
        ;
        publish("Requested movement", "Unknown");
        publish("Distance to target", "Unknown");
        publish("Yaw offset", "Unknown");
    }

    /** Set if vision is allowed to drive. */
    public void setVisionControl(boolean state) {
        this.enableVision = state;
    }
}
