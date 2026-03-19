package org.ironriders.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

/**
 * Camera source implementation for PhotonVision cameras.
 *
 * <p>
 * Uses multi-tag PnP as primary strategy with single-tag fallback.
 *
 * <p>
 * Example:
 * 
 * <pre>{@code
 * PhotonSource cam = new PhotonSource("cam-back",
 *         new Transform3d(-0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-20), Math.PI)),
 *         fieldLayout);
 * }</pre>
 */
public class VisionCamera {
    public enum CameraMode {
        MULTI_SIM,
        MULTI_REAL,
        SINGLE_SIM,
        SINGLE_REAL;
    }

    private final String name;
    private final PhotonCamera camera;
    private Optional<PhotonCameraSim> cameraSim;
    private final PhotonPoseEstimator estimator;
    private CameraMode mode;
    private boolean simulation;
    private Matrix<N3, N1> stdDevs;

    public record PoseEstimate(Pose2d estimatedPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    }

    public VisionCamera(String name, Transform3d robotToCamera, AprilTagFieldLayout fieldLayout, CameraMode mode,
            Optional<PhotonCameraSim> cameraSim) {
        this.name = name;
        this.camera = new PhotonCamera(name);
        this.estimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
        this.mode = mode;
        this.simulation = (mode == CameraMode.MULTI_SIM || mode == CameraMode.SINGLE_SIM) ? true : false;
        if (simulation) {
            if (cameraSim.isEmpty()) {
                throw new IllegalArgumentException("Simulation cameras must be passed a PhotonCameraSim instance");
            }
            this.cameraSim = cameraSim;
        } else {
            this.cameraSim = Optional.empty();
        }
    }

    public String getName() {
        return name;
    }

    public Transform3d getCameraOffset() {
        return estimator.getRobotToCameraTransform();
    }

    public boolean isSimulation() {
        return simulation;
    }

    public PhotonCameraSim getCameraSim() {
        if (cameraSim.isEmpty()) {
            throw new IllegalStateException("This camera does not have a simulation instance");
        }
        return cameraSim.get();
    }

    /**
     * 
     * WARNING!: this method must have 'Do Multi-tag Estimation' enabled to work.
     * 
     * @return
     */
    public Optional<PoseEstimate> getEstimatedPose() {
        return switch (mode) {
            case MULTI_REAL -> getEstimatedPoseMulti();
            case SINGLE_REAL -> getEstimatedPoseSingle();
            case MULTI_SIM -> getEstimatedPoseMulti();
            case SINGLE_SIM -> getEstimatedPoseSingle();
        };
    }

    public Optional<PoseEstimate> getEstimatedPoseSingle() {
        var results = camera.getAllUnreadResults();
        for (var result : results) {
            var estimate = estimator.estimateLowestAmbiguityPose(result);
            if (estimate.isEmpty()) {
                continue;
            }
            return Optional.of(new PoseEstimate(
                    estimate.get().estimatedPose.toPose2d(),
                    result.getTimestampSeconds(),
                    VisionConstants.SINGLE_TAG_STD_DEV));
        }
        return Optional.empty();
    }

    public Optional<PoseEstimate> getEstimatedPoseMulti() {
        Optional<EstimatedRobotPose> estimate = Optional.empty();
        var results = camera.getAllUnreadResults();
        var poses = new LinkedList<PoseEstimate>();
        for (var result : results) {
            estimate = estimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = estimator.estimateLowestAmbiguityPose(result);
                if (estimate.isEmpty()) {
                    continue;
                }
            }
            updateStdDevs(estimate, result.getTargets());
            poses.add(poseEstimate);
        }
        return Optional.empty();
    }

    public Optional<PoseEstimate> getEstimatedPoseMultiSim() {
        Optional<EstimatedRobotPose> estimate = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            estimate = estimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = estimator.estimateLowestAmbiguityPose(result);
                if (estimate.isEmpty())
                    continue;
            }
        }
    }

    public Optional<PoseEstimate> getEstimatedPoseSingleSim() {
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimate      The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     * @param poseEstimator The pose estimator to get tag poses from
     * @return The standard deviations to use for this pose estimate
     */
    public void updateStdDevs(Optional<PoseEstimate> estimate,
            List<PhotonTrackedTarget> targets) {
        if (estimate.isEmpty()) {
            this.stdDevs = VisionConstants.SINGLE_TAG_STD_DEV;
        } else {
            int numTags = 0;
            double avgDist = 0;
            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                }
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation()
                        .getDistance(estimate.get().estimatedPose().getTranslation());
            }
            if (numTags > 0) {
                this.stdDevs = VisionConstants.SINGLE_TAG_STD_DEV;
            } else {
                avgDist /= numTags;

                if (numTags == 1 && avgDist > 4) {
                    this.stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    this.stdDevs = VisionConstants.MULTI_TAG_STD_DEV.times(1 + (avgDist * avgDist / 30));
                }
            }
        }
    }
}