package org.ironriders.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;

import org.ironriders.core.Robot;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.vision.VisionCamera.CameraMode;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystem extends IronSubsystem {

    /**
     * We use this class instead of {@link EstimatedRobotPose} because it allows me
     * to
     * send standard deviations
     * 
     * @param estimatedPose    the estimated pose of the robot
     * @param timestampSeconds the timestamp of the estimate in seconds
     * @param devs             the standard deviations of the estimate
     */
    public record VisionLogEntry(Pose3d estimatedPose, double timestampSeconds, Matrix<N3, N1> devs) {
    }

    private final List<VisionCamera> cameras;
    private final Consumer<VisionLogEntry> poseEstimateConsumer;
    private final boolean simulation;

    public static VisionSystemSim visionSim;

    /**
     * Constructor for the vision subsystem. If in simulation, will construct
     * {@link PhotonCameraSim sim cameras} as well.
     */
    public VisionSubsystem(List<VisionCamera> cameras, Consumer<VisionLogEntry> poseEstimateConsumer,
            CameraMode mode, AprilTagFieldLayout fieldLayout) {
        this.cameras = cameras;
        this.poseEstimateConsumer = poseEstimateConsumer;
        if (Robot.isSimulation()) {
            this.simulation = true;
            visionSim = constructSim(cameras, fieldLayout);
        } else {
            this.simulation = false;
        }
    }

    /**
     * Constructor for the vision subsystem
     * 
     * @param estConsumer      Consumer for estimated robot poses (should be
     *                         {@link })
     * @param fieldLayout
     * @param mode
     * @param cameras
     */
    public VisionSubsystem(Consumer<VisionLogEntry> poseEstimateConsumer,
            AprilTagFieldLayout fieldLayout, CameraMode mode, VisionConstants.CAMERA[] cameras) {
        this(constructCameras(cameras, mode, fieldLayout, Robot.isSimulation()),
                poseEstimateConsumer, mode,
                fieldLayout);
    }

    public VisionSubsystem(Consumer<VisionLogEntry> poseEstimateConsumer, CameraMode mode) {
        this(constructCameras(VisionConstants.CAMERA.values(), mode, VisionConstants.TAG_FIELD_LAYOUT, Robot.isSimulation()), poseEstimateConsumer, mode, VisionConstants.TAG_FIELD_LAYOUT);
    }

    public VisionSubsystem(Consumer<VisionLogEntry> poseEstimateConsumer) {
        this(poseEstimateConsumer, CameraMode.MULTI_TAG);
    }



    /**
     * Makes a list of {@link VisionCamera}s for the subsystem from a {@link Map} of
     * camera names and their {@link Transform3d transforms} from the robot center.
     * // TODO: figure out unit.
     */
    public static List<VisionCamera> constructCameras(VisionConstants.CAMERA[] cameras,
            VisionCamera.CameraMode camMode, AprilTagFieldLayout fieldLayout, boolean simulation) {
        var cams = new LinkedList<VisionCamera>();
        for (var camera : cameras) {
            if (!camera.isEnabled) break;
            if (simulation) {
                cams.add(new VisionCamera(camera.cameraName, camera.robotToCamera, fieldLayout, camMode,
                        Optional.of(new PhotonCameraSim(new PhotonCamera(camera.cameraName)))));
            } else {
                cams.add(new VisionCamera(camera.cameraName, camera.robotToCamera, fieldLayout, camMode, Optional.empty()));
            }
        }
        return cams;
    }

    /**
     * Helper method for chained constructors. For some reason, without this it
     * gives an error that I can't chain contructors with method contents.
     * 
     * @param mode the vision mode to decide the camera mode from
     * @returm the corrosponding camera mode
     */

    /**
     * Helper method for chained constructors (see previous comment for reason).
     * Constructs a {@link VisionSystemSim} for this instance if it is required (if
     * the current mode is simulation)
     * 
     * @param cameras     A list of {@link VisionCamera}s to add to simulation
     * @param fieldLayout The simulation's {@link AprilTagFieldLayout field layout}.
     * @return A {@link VisionSystemSim} with the given cameras and april tag
     *         layout.
     */
    public static VisionSystemSim constructSim(List<VisionCamera> cameras, AprilTagFieldLayout fieldLayout) {
        var sim = new VisionSystemSim("main");
        sim.addAprilTags(fieldLayout);
        for (var cam : cameras) {
            if (Robot.isSimulation()) {
                sim.addCamera(cam.getCameraSim(), cam.getCameraOffset());
            } else {
                throw new IllegalStateException("Vision cameras were instantiated incorrectly for simulation mode");
            }
        }
        return sim;
    }

    @Override
    public void periodic() {
        cameras.stream().forEach(camera -> {
            var estimates = camera.getEstimatedPose();
            for (var estimate : estimates) {
                poseEstimateConsumer.accept(
                        new VisionLogEntry(estimate.estimatedPose(), estimate.timestampSeconds(), estimate.devs()));
                if (simulation) {
                    if (estimates.isEmpty()) {
                        visionSim.getDebugField().getObject("VisionEstimation").setPoses();
                    } else {
                        visionSim.getDebugField().getObject("VisionEstimation")
                                .setPose(estimate.estimatedPose().toPose2d());
                    }
                }
            }
        });
    }
}