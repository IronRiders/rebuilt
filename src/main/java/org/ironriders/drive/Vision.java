package org.ironriders.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

import org.ironriders.lib.LimelightHelpers;

import swervelib.SwerveDrive;

/**
 * Vision is not a subsystem. This class is a utility class for the
 * DriveSubsystem and controls all
 * of the apriltag processing and pose estimation.
 *
 * <p>
 * (Why is it not a subsystem? Because it doesn't need to be.)
 */
public class Vision {

    private SwerveDrive swerveDrive = null;
    private Pigeon2 pigeon = new Pigeon2(9);
    public boolean hasPose = false;

    public Vision(SwerveDrive drive) {
        this.swerveDrive = drive;
    }

    /**
     * Call on each perodic tick to update the swerve drive with any available
     * vision data
     *
     * @returns true on success false on failure
     */
    public boolean updatePose() {

        // First, tell Limelight your robot's current orientation
        double robotYaw = pigeon.getYaw().getValueAsDouble();

        // Apparently these zeros are ignored?
        LimelightHelpers.SetRobotOrientation(
                DriveConstants.LIMELIGHT_NAME, robotYaw, swerveDrive.angularVelocityCoefficient, 0.0, 0.0, 0.0, 0.0);
        boolean rejectUpdate = false;

        // Set the right coordinate system and get the vision measurement
        Optional<Alliance> alliance = DriverStation.getAlliance();
        LimelightHelpers.PoseEstimate limelightMeasurement = new LimelightHelpers.PoseEstimate();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed(DriveConstants.LIMELIGHT_NAME);
            }
            if (alliance.get() == Alliance.Blue) {
                limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(DriveConstants.LIMELIGHT_NAME);
            }
        } else {
            // FMS not connected
            rejectUpdate = true;
        }

        if (Math.abs(pigeon.getAngularVelocityZWorld().getValueAsDouble()) > 360) {
            // Rotating too fast for a valid measurement
            rejectUpdate = true;
        }
        if (limelightMeasurement.tagCount == 0) {
            // There's nothing to update off of
            rejectUpdate = true;
        }
        if (!rejectUpdate) {
            // Add it to your pose estimator
            swerveDrive.setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            DriveConstants.VISION_X_TRUST,
                            DriveConstants.VISION_Y_TRUST,
                            DriveConstants.VISION_ANGLE_TRUST));
            swerveDrive.addVisionMeasurement(
                    limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
            this.hasPose = true;
            return true;
        } else {
            return false;
        }
    }
}
