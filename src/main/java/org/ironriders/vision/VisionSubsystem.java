package org.ironriders.vision;

import java.io.UncheckedIOException;
import java.util.List;

import org.ironriders.drive.DriveSubsystem;
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
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

public class VisionSubsystem extends IronSubsystem {
    private final VisionCommands commands = new VisionCommands(this);

    private PhotonCamera camera = new PhotonCamera(VisionConstants.VISION_CAMERA);
    private final PhotonPoseEstimator poseEstimator;

    private List<PhotonPipelineResult> results;

    private Double skew;
    private Double lastSkew = null;

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

        skew = calculateSkew(result.getBestTarget());

        publish("Skew", skew);
        publish("Last Skew", lastSkew);

        if (Math.abs(lastSkew - skew) > VisionConstants.SKEW_THROWAWAY_THRESHOLD && lastSkew != null) {
            reportWarning("Skew Jump");
            return;
        }

        // Throwaway the pose if it is too normal to us or is too far away.
        if (Math.abs(skew) < VisionConstants.SKEW_THROWAWAY_THRESHOLD
                || Utils.getPoseDifference(Utils.flattenPose3d(newPose.estimatedPose),
                        DriveSubsystem.getSwerveDrive().getPose()).getNorm() > VisionConstants.DISTANCE_THROWAWAY_THRESHOLD) {
            reportWarning("Skew Throwaway");
            return;
        }

        lastSkew = skew;

        // Actually add the estimate
        DriveSubsystem.getSwerveDrive().setVisionMeasurementStdDevs(estimateStdDevVector(newPose, result.targets));
        DriveSubsystem.getSwerveDrive().addVisionMeasurement(newPose.estimatedPose.toPose2d(),
                Timer.getFPGATimestamp());
    }

    public double calculateSkew(PhotonTrackedTarget target) {
        return (target.getBestCameraToTarget().getRotation().getZ() * 180.0 / Math.PI) - 90;
    }

    /*
     * Get the distance to the provided target from the camera. Can throw an error
     * if the provided target's tag is not valid
     */
    public double getDistance(PhotonTrackedTarget target) {
        // *2, as the offset is from the center of the robot, and this wants the
        // distance from the floor
        return PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.CAMERA_OFFSET.getZ() * 2,
                fieldLayout.getTagPose(target.getFiducialId())
                        .orElseThrow().getZ(),
                VisionConstants.CAMERA_OFFSET.getRotation().getY(), // Pitch
                target.getPitch());
    }

    public Double[] getTargetAngles(PhotonTrackedTarget target) {
        return new Double[] { target.getYaw(), target.getPitch(), calculateSkew(target) };
    }

    @Override
    public void periodic() {
        results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return; // Immediately give up if there is no new work to do.
        }

        PhotonPipelineResult result = results.get(results.size() - 1); // We only care about the most recent reading.

        publish("Sees target?", result.hasTargets());

        if (result == null || !result.hasTargets()) {
            return; // We don't see any tags, give up.
        }

        estimateRobotPose(result);
    }

    public VisionCommands getCommands() {
        return commands;
    }
}
