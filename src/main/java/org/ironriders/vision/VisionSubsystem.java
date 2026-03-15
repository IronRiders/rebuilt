package org.ironriders.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;

import org.ironriders.core.RobotContainer;
import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.VisionCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Subsystem for doing machine vision and calculating location based on that.
 */
public class VisionSubsystem extends IronSubsystem {
    public enum TagInvalidReason {
        NO_SKEW,
        TOO_DISTANT,
        NEGATIVE_DISTANCE
    }

    private String debugString;

    public static AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static VisionSystemSim visionSim = new VisionSystemSim("main");

    public VisionSubsystem() {
        /**
         * If we are simulating, add the cameras to the photonsim.
         */
        if (VisionConstants.CAMERA_MODE == VisionConstants.CameraMode.SIM) {

            visionSim.addAprilTags(tagLayout);

            SimCameraProperties cameraProp = new SimCameraProperties();

            cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70));

            // cameraProp.setCalibError(4, 0.1);
            cameraProp.setCalibError(0.25, 0.08);

            cameraProp.setFPS(40);

            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);
            final AtomicInteger i = new AtomicInteger(1);

            VisionConstants.CAMERAS.stream().forEach((camera) -> {
                PhotonCameraSim cameraSim = new PhotonCameraSim(camera.getPhotonCamera(), cameraProp);

                cameraSim.enableRawStream(true);
                cameraSim.enableProcessedStream(true);

                cameraSim.enableDrawWireframe(true);

                visionSim.addCamera(cameraSim, camera.getOffset());

                System.out.printf(
                        "Camera %s's processed stream is at http://localhost:118%d, it's raw stream is at http://localhost:118%d\n",
                        camera.getSimpleName(), i.get() * 2, ((i.get() * 2) - 1));
                System.out.println(camera.toString());
                i.getAndIncrement();
            });
        }
    }

    @Override
    public void periodic(){

        // for every camera...
        VisionConstants.CAMERAS.stream().forEach((camera) -> {
            camera.updateResultBuffer();
            if (!camera.seesTargets()) {

                return; // We don't see any tags, give up.
            }
            // estimate the pose
            estimateRobotPose(camera);
        });
    }

    /**
     * Try to estimate how much we should trust the opinion of this camera. Higher
     * numbers mean less trust.
     */
    public Vector<N3> estimateStdDevVector(VisionCamera camera) {
        double xyStdDev;
        double thetaStdDev;

        double avgDistance = camera.getTargets().stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(-1);

        // TODO: These numbers are mostly arbitrary.
        if (camera.getTargets().size() > 1) { // Multi target
            xyStdDev = 0.02 + (avgDistance * 0.03);
            thetaStdDev = Math.toRadians(1 + avgDistance);
        } else { // Single Target
            xyStdDev = 0.5 + (avgDistance * 0.1);
            thetaStdDev = Math.toRadians(10 + avgDistance * 5); // Really don't trust single tag rotation
        }

        double ambiguity = camera.getTargets().stream()
                .mapToDouble(t -> t.getPoseAmbiguity())
                .average()
                .orElse(Double.POSITIVE_INFINITY) + 1d; // Make sure we really don't like this pose if the optional is
                                                        // null.

        // if we have high ambiguity, remove some trust.
        xyStdDev *= (ambiguity);
        thetaStdDev *= (ambiguity * 2.0);

        // add camera weighting. Inverted because higher numbers are less trusting.
        xyStdDev += -camera.getWeight() * VisionConstants.WEIGHT_SCALE;
        thetaStdDev += -camera.getWeight() * VisionConstants.WEIGHT_SCALE;

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    /**
     * Try to estimate the position of the robot using the visible tags.
     * This function should be called for every camera.
     */
    public void estimateRobotPose(VisionCamera camera) {
        publish("estimating pose", true);

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
                addBadTagToString(TagInvalidReason.NEGATIVE_DISTANCE, distanceString);
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

        List<PhotonTrackedTarget> invalidTargets = new ArrayList<PhotonTrackedTarget>(camera.getTargets());
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
            DriveSubsystem.getSwerveDrive().setVisionMeasurementStdDevs(estimateStdDevVector(camera));
        }

        // Actually add the estimate.
        DriveSubsystem.getSwerveDrive().addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
                estimatedPose.timestampSeconds); // Use capture time, not now, for latency compensation
    }

    // debugging helper functions
    public void makeDebugString(PhotonTrackedTarget target) {
        debugString = "Tag " + String.valueOf(target.getFiducialId()) + ": ";
    }

    public void addBadTagToString(TagInvalidReason reason, String extra) {
        if (debugString != null) {
            debugString += "BAD: " + reason.toString();
            if (extra != null) {
                debugString += " | " + extra;
            }
        }
    }

    public void addGoodTagToString(String extra) {
        if (debugString != null) {
            debugString += "GOOD: ";
            if (extra != null) {
                debugString += " | " + extra;
            }
        }
    }

    /*
     * Calculate the skew for a tag
     */
    public double calculateSkew(PhotonTrackedTarget target) {
        return (target.getBestCameraToTarget().getRotation().getZ() * 180.0 / Math.PI) - 180;
    }

    /*
     * Get a list of the angles of a tag
     */
    public Double[] getTargetAngles(PhotonTrackedTarget target) {
        return new Double[] { target.getYaw(), target.getPitch(), calculateSkew(target) };
    }
}
