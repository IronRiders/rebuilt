package org.ironriders.vision;

import java.io.UncheckedIOException;
import java.util.List;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.IronSubsystem;
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
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

/**
 * Subsystem for vision processing using PhotonVision and AprilTags.
 */
public class VisionSubsystem extends IronSubsystem {
    private final VisionCommands commands = new VisionCommands(this);

    private PhotonCamera camera = new PhotonCamera(VisionConstants.VISION_CAMERA);
    private PIDController visPidController = new PIDController(VisionConstants.VISION_P, VisionConstants.VISION_I,
            VisionConstants.VISION_D);
    private final PhotonPoseEstimator poseEstimator;

    private List<PhotonTrackedTarget> targets;
    private List<PhotonPipelineResult> results;

    double skew;
    double lastSkew = -9999;

    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

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

    /**
     * Estimates the standard deviation of a given position based on the number of
     * tags.
     * 
     * @param pose    The estimated {@link EstimatedRobotPose robot position}.
     * @param targets The list of tracked targets used for the estimation.
     * @return A vector representing the estimated standard deviation in the
     *         position.
     */
    public Vector<N3> estimateStdDevVector(EstimatedRobotPose pose, List<PhotonTrackedTarget> targets) {
        double xyStdDev;
        double thetaStdDev;

        double avgDistance = pose.targetsUsed.parallelStream()
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

        double ambiguity = targets.parallelStream()
                .mapToDouble(t -> t.getPoseAmbiguity())
                .average()
                .orElse(Double.POSITIVE_INFINITY) + 1d; // Make sure we really don't like this pose if the optional is
                                                        // null.

        // if we have high ambiguity, remove some trust.
        xyStdDev *= (ambiguity);
        thetaStdDev *= (ambiguity * 2.0);

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    /**
     * Estimates the robot's pose using the given {@link PhotonPipelineResult
     * PhotonVision result}.
     * 
     * @param result The PhotonVision pipeline result containing detected targets.
     */
    public void estimateRobotPose(PhotonPipelineResult result) {
        EstimatedRobotPose newPose;

        if (result.getTargets().size() > 1) {
            newPose = poseEstimator.estimateCoprocMultiTagPose(result).orElse(null);
        } else {
            newPose = poseEstimator.estimateLowestAmbiguityPose(result).orElse(null); // Because a single tag is worse.
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

    /**
     * Calculates the distance to a given target using the camera's pitch and the
     * target's pitch.
     * 
     * @param target The tracked target for which to calculate the distance.
     * @return The estimated distance to the target in meters.
     */
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

    /**
     * Gets the target angles (yaw, pitch, and skew) for a given tracked target.
     * 
     * @param target The tracked target for which to get the angles.
     * @return An array containing the yaw, pitch, and skew angles of the target (in
     *         that order).
     */
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
            return; // We don't see any tags, give up.
        }

        targets = result.getTargets();

        if (result != null) {
            estimateRobotPose(result);
        }
    }

    /**
     * Gets the {@link VisionCommands commands} for this subsystem.
     * 
     * @return The VisionCommands instance associated with this subsystem.
     */
    public VisionCommands getCommands() {
        return commands;
    }
}
