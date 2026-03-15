package org.ironriders.vision;

import org.ironriders.lib.VisionCamera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem {
    private final Map<VisionCamera, PhotonPoseEstimator> estimators = new HashMap<>();
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;
    private boolean isSimulation = false;

    // Simulation
    public static PhotonCameraSim cameraSim;
    public static VisionSystemSim visionSim;

    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your
     *                    desired {@link
     *                    edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public VisionSubsystem(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        VisionConstants.CAMERAS.forEach((camera) -> {
            estimators.put(
                    camera,
                    new PhotonPoseEstimator(
                            VisionConstants.TAG_FIELD_LAYOUT,
                            camera.getOffset()));
        });
    }

    public VisionSubsystem(EstimateConsumer estConsumer, boolean isSimulation) {
        this.estConsumer = estConsumer;
        this.isSimulation = isSimulation;
        if (isSimulation) {
            // Create the vision system simulation which handles cameras and targets on the
            // field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this
            // simulated field.
            visionSim.addAprilTags(VisionConstants.TAG_FIELD_LAYOUT);
            // Create simulated camera properties. These can be set to mimic your actual
            // camera. // TODO: set correct values
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values
            // with visible targets.
            // Add the simulated cameras to view the targets on this simulated field.
            estimators.forEach((camera, estimator) -> {
                visionSim.addCamera(
                        cameraSim,
                        camera.getOffset());
            });
            cameraSim.enableDrawWireframe(true);
        }
    }

    public void visionEstCamera(VisionCamera camera, PhotonPoseEstimator estimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getPhotonCamera().getAllUnreadResults()) {
            visionEst = estimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = estimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets(), estimator);

            if (isSimulation) {
                visionEst.ifPresentOrElse(
                        est -> getSimDebugField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }
            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds,
                                estStdDevs);
                    });
        }
    }

    public void periodic() {
        estimators.entrySet().parallelStream().forEach(entry -> visionEstCamera(entry.getKey(), entry.getValue()));
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator estimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.SINGLE_TAG_STD_DEV;

        List<PhotonTrackedTarget> validTargets = new ArrayList<PhotonTrackedTarget>();
        Map<PhotonTrackedTarget, String> tagStrings = new HashMap<PhotonTrackedTarget, String>();

        // for every target (tag)...
        for (PhotonTrackedTarget target : camera.getTargets()) {
            makeDebugString(target);

            // get the skew (the angle off of straight on)
            Double skew = calculateSkew(target);

            // get the distance from the camera to the target.
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();

            String distanceString = String.format("%03.2f", distance);

            // -- Checks to make sure that tag is valid --

            // if we are too normal to the tag, we can't trust the result. this is for
            // complicated reasons involving how photon vision sees tags. ask Issy in
            // discord if you really need to know (you probably don't)
            if (Math.abs(skew) < VisionConstants.SKEW_THROWAWAY_THRESHOLD
                    || Math.abs(skew) > 360 - VisionConstants.SKEW_THROWAWAY_THRESHOLD) {
                addBadTagToString(TagInvalidReason.NO_SKEW, String.valueOf(skew));
                tagStrings.put(target, debugString);

                continue;
            }

            // the distance is negative, something has gone wrong.
            if (distance < 0) {
                addBadTagToString(TagInvalidReason.TOO_CLOSE, distanceString);
                tagStrings.put(target, debugString);

                continue;
            }

            // if the distance is too great, we can't trust that the tag will be read
            // reliably, so just ignore it.
            if (distance > VisionConstants.TARGET_DISTANCE_THROWAWAY_THRESHOLD) {
                addBadTagToString(TagInvalidReason.TOO_DISTANT, distanceString);
                tagStrings.put(target, debugString);

                continue;
            }

            addGoodTagToString(String.format("dist: %s skew: %s", distanceString, skew));
            tagStrings.put(target, debugString);

            // tag is valid! yay :3
            validTargets.add(target);
        }

        List<PhotonTrackedTarget> invalidTargets = camera.getTargets();
        invalidTargets.removeAll(validTargets);

        publish("Invalid targets",
                invalidTargets.stream().map(t -> String.valueOf(t.fiducialId)).collect(Collectors.joining(" | ")));

        publish("Valid targets:",
                validTargets.stream().map(PhotonTrackedTarget::getFiducialId).map(i -> String.valueOf(i))
                        .collect(Collectors.joining(" | ")));

        publish(String.format("Tag data for camera: %s", camera.getSimpleName()),
                tagStrings.values().stream().sorted().collect(Collectors.joining(" | ")));

        // construct a new result
        PhotonPipelineResult validResult = new PhotonPipelineResult(camera.getResult().metadata,
                validTargets, camera.getResult().getMultiTagResult());

        EstimatedRobotPose estimatedPose;

        // if (camera.getTargets().size() > 1) {
        // if we have more than one tag, do multi-tag estimation,
        // estimatedPose =
        // camera.getEstimator().estimateCoprocMultiTagPose(validResult).orElse(null);
        // } else if (camera.getTargets().size() == 1) {
        // if we only have one, do single tag.
        estimatedPose = camera.getEstimator().estimateLowestAmbiguityPose(validResult).orElse(null);
        // }// else {
        // otherwise, we must not have any, exit.
        // return;
        // }

        if (estimatedPose == null) {
            // Something has gone wrong, give up and try again next tick.
            return;
        }

        // Throw away the new pose if it is too far away.
        if (estimatedPose.estimatedPose.getTranslation()
                .getDistance(
                        DriveSubsystem.getPose3d().getTranslation()) > VisionConstants.POSE_DISTANCE_THROWAWAY_THRESHOLD
                && (DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled())) {
            return;
        }

        // Swerve odometry is jumpy, so it messes with the autos. Dead reckoning is
        // accurate enough and smooth, so we use that for auto.
        // TODO: vision is for shooting maybe (might be more accurate to use for
        // shooting, and smoothness doesn't matter if you're not using it to move the
        // robot)
        if (DriverStation.isAutonomous() && !RobotContainer.scoringZone.inside()) {
            //DriveSubsystem.getSwerveDrive().setVisionMeasurementStdDevs(VecBuilder.fill(20, 20, 50));
            return;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEV;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.SINGLE_TAG_STD_DEV;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = VisionConstants.MULTI_TAG_STD_DEV;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (isSimulation)
            visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!isSimulation)
            return null;
        return visionSim.getDebugField();
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}