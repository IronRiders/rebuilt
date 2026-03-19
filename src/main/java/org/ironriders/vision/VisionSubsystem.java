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

import org.ironriders.lib.IronSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystem extends IronSubsystem {

    /**
     * doing this instead of {@link EstimatedRobotPose} because it allows me to
     * send standard deviations
     * 
     * @param estimatedPose    the estimated pose of the robot
     * @param timestampSeconds the timestamp of the estimate in seconds
     * @param devs             the standard deviations of the estimate
     */
    public record SettableVisionLog(Pose3d estimatedPose, double timestampSeconds, Matrix<N3, N1> devs) {
    }

    public enum VisionMode {
        OLD,
        NEW_MULTI,
        NEW_SINGLE,
        OFF,
        SIM_OLD,
        SIM_NEW_MULTI,
        SIM_NEW_SINGLE,
        SIM_OFF;
    }

    private final List<VisionCamera> cameras;
    private final Consumer<SettableVisionLog> estConsumer;
    private final boolean simulation;

    public static VisionSystemSim visionSim;

    /**
     * Constructor for the vision subsystem. If in simulation, will construct
     * {@link PhotonCameraSim sim cameras} as well.
     */
    public VisionSubsystem(List<VisionCamera> cameras, Consumer<SettableVisionLog> estConsumer,
            VisionMode mode, AprilTagFieldLayout fieldLayout) {
        this.cameras = cameras;
        this.estConsumer = estConsumer;
        if (getSimulationStatus(mode)) {
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
     * @param cameraTransforms
     */
    public VisionSubsystem(Consumer<SettableVisionLog> estConsumer,
            AprilTagFieldLayout fieldLayout, VisionMode mode, Map<String, Transform3d> cameraTransforms) {
        this(constructCameras(cameraTransforms, decideCameraMode(mode), fieldLayout, getSimulationStatus(mode)),
                estConsumer, mode,
                fieldLayout);
    }

    /**
     * Constructor for the default vision system. Uses the new multi-tag estimation
     * method.
     */
    public VisionSubsystem(Consumer<SettableVisionLog> estConsumer) {
        this(estConsumer, VisionConstants.TAG_FIELD_LAYOUT, VisionMode.NEW_MULTI, VisionConstants.CAMERA_TRANSFORMS);
    }

    private static boolean getSimulationStatus(VisionMode mode) {
        return switch (mode) {
            case SIM_OLD, SIM_NEW_MULTI, SIM_NEW_SINGLE, SIM_OFF -> true;
            default -> false;
        };
    }

    /**
     * Makes a list of {@link VisionCamera}s for the subsystem from a {@link Map} of
     * camera names and their {@link Transform3d transforms} from the robot center.
     * // TODO: figure out unit.
     */
    public static List<VisionCamera> constructCameras(Map<String, Transform3d> cameraTransforms,
            VisionCamera.CameraMode camMode, AprilTagFieldLayout fieldLayout, boolean simulation) {
        var cams = new LinkedList<VisionCamera>();
        for (var entry : cameraTransforms.entrySet()) {
            if (simulation) {
                cams.add(new VisionCamera(entry.getKey(), entry.getValue(), fieldLayout, camMode,
                        Optional.of(new PhotonCameraSim(new PhotonCamera(entry.getKey())))));
            } else {
                cams.add(new VisionCamera(entry.getKey(), entry.getValue(), fieldLayout, camMode, Optional.empty()));
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
    public static VisionCamera.CameraMode decideCameraMode(VisionMode mode) {
        return switch (mode) {
            case SIM_NEW_MULTI -> VisionCamera.CameraMode.MULTI_SIM;
            case NEW_MULTI -> VisionCamera.CameraMode.MULTI_REAL;
            case SIM_NEW_SINGLE -> VisionCamera.CameraMode.SINGLE_SIM;
            case NEW_SINGLE -> VisionCamera.CameraMode.SINGLE_REAL;
            default -> VisionCamera.CameraMode.MULTI_REAL;
        };
    }

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
            if (cam.isSimulation()) {
                sim.addCamera(cam.getCameraSim(), cam.getCameraOffset());
            } else {
                throw new IllegalStateException("Vision cameras were instantiated incorrectly for simulation mode");
            }
        }
        return sim;
    }

    @Override
    public void periodic() {
        cameras.parallelStream().forEach(camera -> {
            var estimates = camera.getEstimatedPose();
            for (var estimate : estimates) {
                estConsumer.accept(
                        new SettableVisionLog(estimate.estimatedPose(), estimate.timestampSeconds(), estimate.devs()));
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