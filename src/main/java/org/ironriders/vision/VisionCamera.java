package org.ironriders.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.ironriders.core.Robot;
import org.jspecify.annotations.NonNull;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

/**
 * Class which provides an interface to get a camera's best guess as to the
 * robot's pose.
 * 
 * Many of these are innitialized in the subsystem class.
 * 
 * The main logic for calculating standard deviations is in this class.
 */
public class VisionCamera {
    public enum CameraMode {
        MULTI_TAG,
        SINGLE_TAG;
    }

    private final PhotonCamera camera;
    private Optional<@NonNull PhotonCameraSim> cameraSim;
    private final PhotonPoseEstimator poseEstimator;
    private CameraMode mode;

    public record PoseEstimate(Pose3d estimatedPose, double timestampSeconds, List<PhotonTrackedTarget> targets,
            Matrix<N3, N1> devs) {
    }

    public VisionCamera(String name, Transform3d robotToCamera, AprilTagFieldLayout fieldLayout, CameraMode mode,
            Optional<@NonNull PhotonCameraSim> cameraSim) {
        this.camera = new PhotonCamera(name);
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
        this.mode = mode;
        if (Robot.isSimulation()) {
            if (cameraSim.isEmpty()) {
                DogLog.log("Camera " + name + "Simulation", "Not passed a sim instance, but robot is simulating .");
                DogLog.log("Simulation", "Initiating a new sim instance.");
                cameraSim = Optional.of(new PhotonCameraSim(camera));
            }
            this.cameraSim = cameraSim;
        } else {
            this.cameraSim = Optional.empty();
        }
    }

    public PhotonCameraSim getCameraSim() {
        if (cameraSim.isEmpty()) {
            throw new IllegalStateException("This camera does not have a simulation instance");
        }
        if (!Robot.isSimulation()) {
            throw new IllegalStateException("Cannot get simulation instance of a real camera");
        }
        return cameraSim.get();
    }

    public Transform3d getCameraOffset() {
        return poseEstimator.getRobotToCameraTransform();
    }

    /**
     * Gets the most recent estimated pose of the robot based on the camera's mode.
     * Redirects to the correct method (estimate)
     *
     * @return {@link List<PoseEstimate> estimatedPose}
     */
    public List<PoseEstimate> getEstimatedPose() {
        return switch (mode) {
            case MULTI_TAG -> getPoseMultiTag();
            case SINGLE_TAG -> getPoseSingleTag();
        };
    }

    private List<PoseEstimate> getPoseSingleTag() {
        var unread = camera.getAllUnreadResults();
        var poses = new LinkedList<PoseEstimate>();
        for (var result : unread) {
            var estimate = poseEstimator.estimateLowestAmbiguityPose(result);
            if (estimate.isEmpty()) {
                continue;
            }
            poses.add(new PoseEstimate(
                    estimate.get().estimatedPose,
                    result.getTimestampSeconds(),
                    result.getTargets(),
                    getStdDevs(estimate, result.getTargets())));
        }
        return poses;
    }

    private List<PoseEstimate> getPoseMultiTag() {
        var poses = new LinkedList<PoseEstimate>();
        for (var result : camera.getAllUnreadResults()) {
            var estimate = poseEstimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = poseEstimator.estimateLowestAmbiguityPose(result);
                if (estimate.isEmpty()) {
                    continue; // if neither come up, there likely is no tag in vision. If this is true, we
                              // should throw it away.
                }
            }
            poses.add(
                    new PoseEstimate(
                            estimate.get().estimatedPose,
                            result.getTimestampSeconds(),
                            result.getTargets(),
                            getStdDevs(estimate, result.getTargets())));
        }
        return poses;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     * 
     * This I got from the example.
     *
     * @param estimate      The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     * @param poseEstimator The pose estimator to get tag poses from
     * @return The standard deviations to use for this pose estimate
     */
    private Matrix<N3, N1> getStdDevs(Optional<EstimatedRobotPose> estimate,
            List<PhotonTrackedTarget> targets) {
        if (estimate.isEmpty()) {
            return VisionConstants.SINGLE_TAG_STD_DEV;
        } else {
            int numTags = 0;
            double avgDistMeters = 0;
            for (var target : targets) {
                var tagPoseMeters = poseEstimator.getFieldTags().getTagPose(target.getFiducialId()); // According to
                                                                                                     // docs, the
                                                                                                     // AprilTagFieldLayout
                                                                                                     // is stored in
                                                                                                     // meters, so I'm
                                                                                                     // assuming
                                                                                                     // individual
                                                                                                     // apriltag
                                                                                                     // positions are
                                                                                                     // likewise.
                if (tagPoseMeters.isEmpty()) {
                    continue;
                }
                numTags++;
                avgDistMeters += tagPoseMeters.get().toPose2d().getTranslation()
                        .getDistance(estimate.get().estimatedPose.toPose2d().getTranslation());
            }
            if (numTags <= 0) {
                return VisionConstants.SINGLE_TAG_STD_DEV;
            } else {
                avgDistMeters /= numTags;
                if (numTags == 1 && avgDistMeters > VisionConstants.THROWAWAY_TAG_DISTANCE_METERS_MULTITAG) {
                    return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    return VisionConstants.MULTI_TAG_STD_DEV
                            .times(1 + (avgDistMeters * avgDistMeters / VisionConstants.AVERAGE_DISTANCE_SCALE_VALUE));
                }
            }
        }
    }
}