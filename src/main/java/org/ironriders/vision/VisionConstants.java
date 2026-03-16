package org.ironriders.vision;

import java.util.ArrayList;
import java.util.List;

import org.ironriders.lib.VisionCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Constants for the {@link VisionSubsystem} */
public class VisionConstants {
    public enum CameraMode {
        SIM,
        REAL;
    }

    public static List<VisionCamera> CAMERAS = new ArrayList<VisionCamera>();

    public static final CameraMode CAMERA_MODE = CameraMode.SIM;

    static {
        switch (CAMERA_MODE) {
            case REAL:
                CAMERAS.add(new VisionCamera("launcher", new Transform3d( //TODO: rename to the real camera. Disabled for now 
                        new Translation3d(
                                0,
                                0,
                                0.498
                        ),
                        new Rotation3d(
                                0.0, // roll
                                -Math.toRadians(35),
                                Math.PI // yaw
                        ))));
                break;

            case SIM:
                CAMERAS.add(new VisionCamera("launcher-back", new Transform3d(
                        new Translation3d(
                                0.45, // forward (meters)
                                -0.26, // left (meters)
                                0.19 // up (meters)
                        ),
                        new Rotation3d(
                                0.0, // roll
                                -Math.toRadians(15), // pitch
                                0.0 // yaw
                        ))));

                CAMERAS.add(new VisionCamera("launcher-back-high", new Transform3d(
                        new Translation3d(
                                0.45, // forward (meters)
                                -0.26, // left (meters)
                                0.19 // up (meters)
                        ),
                        new Rotation3d(
                                0.0, // roll
                                -Math.toRadians(30), // pitch
                                0.0 // yaw
                        ))));

                CAMERAS.add(new VisionCamera("launcher-hood", new Transform3d(
                        new Translation3d(
                                0, // forward (meters)
                                0, // left (meters)
                                0.498 // up (meters)
                        ),
                        new Rotation3d(
                                0.0, // roll
                                -Math.toRadians(20), // pitch
                                Math.PI // yaw
                        ))));

                CAMERAS.add(new VisionCamera("swerve-back-left", new Transform3d(
                        new Translation3d(
                                0.212, // forward (meters)
                                0.218, // left (meters)
                                0.16 // up (meters)
                        ),
                        new Rotation3d(
                                0.0, // roll
                                -Math.toRadians(25), // pitch
                                Math.toRadians(45)))));

                CAMERAS.add(new VisionCamera("swerve-back-right", new Transform3d(
                        new Translation3d(
                                0.212, // forward (meters)
                                -0.218, // left (meters)
                                0.16 // up (meters)
                        ),
                        new Rotation3d(
                                0.0, // roll
                                -Math.toRadians(25), // pitch
                                -Math.toRadians(45)))));

            default:
                break;
        }
    }

    public static final Double SKEW_THROWAWAY_THRESHOLD = 15d; // deg,
    public static final Double POSE_DISTANCE_THROWAWAY_THRESHOLD = 6d; // meters
    public static final Double AMBIGUITY_THROWAWAY_THRESHOLD = 0.2d; // ambiguity ratio, from https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html#ambiguity;

    public static final Double TARGET_DISTANCE_THROWAWAY_THRESHOLD = 4d; // meters

    public static final Double WEIGHT_SCALE = 5d;
}
