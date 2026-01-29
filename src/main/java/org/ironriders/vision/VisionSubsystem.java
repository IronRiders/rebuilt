package org.ironriders.vision;

import java.io.UncheckedIOException;
import java.util.List;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.drive.DriveConstants.Controller;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

public class VisionSubsystem extends IronSubsystem {
    private final VisionCommands commands = new VisionCommands(this);

    private PhotonCamera camera = new PhotonCamera(VisionConstants.VISION_CAMERA);
    private PIDController visPidController = new PIDController(VisionConstants.VISION_P, VisionConstants.VISION_I, VisionConstants.VISION_D);
    private final PhotonPoseEstimator poseEstimator;

    private List<PhotonTrackedTarget> targets;
    private List<PhotonPipelineResult> results;

    double skew;
    double lastSkew = -9999;

    public static AprilTagFieldLayout fieldLayout;

    public VisionSubsystem() {
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        } catch (UncheckedIOException e) {
            reportError("Could not load apriltag layout!");
            e.printStackTrace();
        }
        poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                VisionConstants.CAMERA_OFFSET);

        for (var tag : fieldLayout.getTags()) {
            System.out.printf("tag %d, %s.\n", tag.ID, tag.pose.getTranslation().toString());
        }
    }

    public Vector<N3> estimateStdDevVector(EstimatedRobotPose pose, List<PhotonTrackedTarget> targets) {
        double xyStdDev;
        double thetaStdDev;

        double avgDistance = pose.targetsUsed.stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(-1);

        // TODO: These numbers are mostly arbitrary.
        if (pose.targetsUsed.size() > 1) { // Multi target
            xyStdDev = 0.02 + (avgDistance * 0.03);
            thetaStdDev = Math.toRadians(1 + avgDistance);
        } else { // Single Target
            xyStdDev = 0.5 + (avgDistance * 0.1);
            thetaStdDev = Math.toRadians(10 + avgDistance * 5); // Really don't trust single tag rotation
        }

        double ambiguity = targets.stream()
                .mapToDouble(t -> t.getPoseAmbiguity())
                .average()
                .orElse(Double.POSITIVE_INFINITY) + 1d; // Make sure we really don't like this pose if the optional is
                                                        // null.

        // if we have high ambiguity, remove some trust.
        xyStdDev *= (ambiguity);
        thetaStdDev *= (ambiguity * 2.0);

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    public void estimateRobotPose(PhotonPipelineResult result) {
        EstimatedRobotPose newPose;

        if (result.getTargets().size() > 1) {
            newPose = poseEstimator.estimateCoprocMultiTagPose(result).orElse(null);
        } else {
            newPose = poseEstimator.estimateLowestAmbiguityPose(result).orElse(null);
        }

        if (newPose == null) {
            // Something has gone wrong, give up and try again next tick.
            reportWarning("Giving up in pose estimation!");
            return;
        }

        skew = result.getBestTarget().getBestCameraToTarget().getRotation().getZ() * 180.0 / Math.PI;
        skew -= 90;

        publish("skew", skew);
        publish("last skew", lastSkew);

        if (Math.abs(lastSkew - skew) > 50 && lastSkew != -9999) {
            reportWarning("Skew Jump");
            return;
        }

        // Throwaway the pose if it is too normal to us or is too far away.
        if (Math.abs(skew) < VisionConstants.SKEW_THROWAWAY_THRESHOLD
        /*
         * || Utils.getPoseDifference(Utils.flattenPose3d(newPose.estimatedPose),
         * DriveSubsystem.getSwerveDrive().getPose()).getNorm() >
         * Vision.DISTANCE_THROWAWAY_THRESHOLD
         */) {
            reportWarning("Skew Throwaway");
            return;
        }

        lastSkew = skew;

        // Actually add the estimate
        DriveSubsystem.getSwerveDrive().setVisionMeasurementStdDevs(estimateStdDevVector(newPose, targets));
        DriveSubsystem.getSwerveDrive().addVisionMeasurement(newPose.estimatedPose.toPose2d(),
                Timer.getFPGATimestamp());
    }

    public double getDistance(PhotonTrackedTarget target) {
        // *2, as the offset is from the center of the robot, and this wants the
        // distance
        // from the floor
        return PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.CAMERA_OFFSET.getZ() * 2,
                fieldLayout.getTagPose(target.getFiducialId())
                        .orElse(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))).getZ(),
                VisionConstants.CAMERA_OFFSET.getRotation().getY(), // Pitch
                target.getPitch());
    }

    public Double[] getTargetAngles(PhotonTrackedTarget target) {
        return new Double[] { target.getYaw(), target.getPitch(), target.getSkew() };
    }

    @Override
    public void periodic() {
        results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return; // Immediately give up if there is no new work to do.
        }

        PhotonPipelineResult result = results.get(results.size() - 1); // We only care about the most recent reading.

        publish("Sees target?", result.hasTargets());

        if (!result.hasTargets() || result == null) {
            DriveSubsystem.requestDriveStop(Controller.VISION); // for testing, just stop if we don't see anything.
            return; // We don't see any tags, give up.
        }

        targets = result.getTargets();

        if (result != null) {
            estimateRobotPose(result);
        }

        // Testing code.
        visPidController.setSetpoint(0); // Assume we've rotated to face the target pose

        for (var target : targets) {
            switch (target.getFiducialId()) {
                case -1: // Error, not a valid tag!
                    reportWarning("Vision got an invalid tag!");
                    return;
                case -2: // disabled for now
                    // We found our favorite toy! (tag #9)
                    double requestedMovement = -Utils.clamp(-VisionConstants.VISION_ROTATION_MAX_SPEED,
                            VisionConstants.VISION_ROTATION_MAX_SPEED,
                            visPidController.calculate(target.getYaw()));

                    // Skew is horizontal offset from cam
                    publish("Yaw, Pitch, Skew", getTargetAngles(target).toString());
                    publish("Requested movement", requestedMovement);

                    DriveSubsystem.requestDriveMovement(Controller.VISION, new Translation2d(-0.4, 0),
                            requestedMovement,
                            false);
                    break;

                default:
                    break;
            }
        }
    }

    public VisionCommands getCommands() {
        return commands;
    }
}
