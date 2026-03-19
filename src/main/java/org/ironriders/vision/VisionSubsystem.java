package org.ironriders.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;

import org.ironriders.lib.IronSubsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystem extends IronSubsystem {
    public enum VisionMode {
        OLD,
        NEW,
        OFF,
        SIM_OLD,
        SIM_NEW,
        SIM_OFF;
    }

    private final List<VisionCamera> cameras;
    private final Consumer<EstimatedRobotPose> estConsumer;
    private final boolean simulation;
    private final VisionMode mode;

    // Simulation
    public static VisionSystemSim visionSim;

    public VisionSubsystem(List<VisionCamera> cameras, Consumer<EstimatedRobotPose> estConsumer,
            VisionMode mode, AprilTagFieldLayout fieldLayout) {
        this.cameras = cameras;
        this.estConsumer = estConsumer;
        this.mode = mode;
        if (mode == VisionMode.SIM_OLD || mode == VisionMode.SIM_NEW || mode == VisionMode.SIM_OFF) {
            this.simulation = true;
            visionSim = constructSim(cameras, fieldLayout);
        } else {
            this.simulation = false;
        }
    }

    public VisionSubsystem(Consumer<EstimatedRobotPose> estConsumer,
            AprilTagFieldLayout fieldLayout, VisionMode mode, Map<String, Transform3d> cameraTransforms) {
        this(constructCameras(cameraTransforms, decideCameraMode(mode), fieldLayout), estConsumer, mode, fieldLayout);
    }

    public VisionSubsystem(Consumer<EstimatedRobotPose> estConsumer, VisionMode mode,
            Map<String, Transform3d> cameraTransforms) {
        this(estConsumer, VisionConstants.TAG_FIELD_LAYOUT, mode, cameraTransforms);
    }

    public VisionSubsystem(Consumer<EstimatedRobotPose> estConsumer, Map<String, Transform3d> cameraTransforms) {
        this(estConsumer, VisionConstants.TAG_FIELD_LAYOUT, VisionMode.NEW, cameraTransforms);
    }

    public VisionSubsystem(Consumer<EstimatedRobotPose> estConsumer) {
        this(estConsumer, VisionConstants.CAMERA_TRANSFORMS);
    }

    public static List<VisionCamera> constructCameras(Map<String, Transform3d> cameraTransforms,
            VisionCamera.CameraMode camMode, AprilTagFieldLayout fieldLayout) {
        var cams = new LinkedList<VisionCamera>();
        for (var entry : cameraTransforms.entrySet()) {
            cams.add(new VisionCamera(entry.getKey(), entry.getValue(), fieldLayout, camMode, Optional.empty()));
        }
        return cams;
    }

    public static VisionCamera.CameraMode decideCameraMode(VisionMode mode) {
        return switch (mode) {
            case SIM_NEW -> VisionCamera.CameraMode.MULTI_SIM;
            case NEW -> VisionCamera.CameraMode.MULTI_REAL;
            case SIM_OLD -> VisionCamera.CameraMode.SINGLE_SIM;
            case OLD -> VisionCamera.CameraMode.SINGLE_REAL;
            default -> VisionCamera.CameraMode.MULTI_REAL;
        };
    }

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
}