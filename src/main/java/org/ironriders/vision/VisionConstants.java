package org.ironriders.vision;

import java.util.List;

import org.ironriders.lib.VisionCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Constants for the {@link VisionSubsystem} */
public class VisionConstants {
    public enum CameraMode {
        SIM,
        REAL;
    }

    public static List<VisionCamera> CAMERAS = List.of(
            new VisionCamera(
                    "launcher-back",
                    new Transform3d(
                            new Translation3d(
                                    0.45,
                                    -0.26,
                                    0.19),
                            new Rotation3d(
                                    0.0,
                                    -1 * Math.toRadians(15),
                                    0.0))),
            new VisionCamera("launcher-back-high", new Transform3d(
                    new Translation3d(
                            0.45, // forward (meters)
                            -0.26, // left (meters)
                            0.19 // up (meters)
                    ),
                    new Rotation3d(
                            0.0, // roll
                            -Math.toRadians(30), // pitch
                            0.0 // yaw
                    ))),
            new VisionCamera("launcher-hood", new Transform3d(
                    new Translation3d(
                            0, // forward (meters)
                            0, // left (meters)
                            0.498 // up (meters)
                    ),
                    new Rotation3d(
                            0.0, // roll
                            -Math.toRadians(20), // pitch
                            Math.PI // yaw
                    ))),
            new VisionCamera("swerve-back-left", new Transform3d(
                    new Translation3d(
                            0.212, // forward (meters)
                            0.218, // left (meters)
                            0.16 // up (meters)
                    ),
                    new Rotation3d(
                            0.0, // roll
                            -Math.toRadians(25), // pitch
                            Math.toRadians(45)))),
            new VisionCamera("swerve-back-right", new Transform3d(
                    new Translation3d(
                            0.212, // forward (meters)
                            -0.218, // left (meters)
                            0.16 // up (meters)
                    ),
                    new Rotation3d(
                            0.0, // roll
                            -Math.toRadians(25), // pitch
                            -Math.toRadians(45)))));
    // These are marked as "example values" in the code, but according to XBot, they
    // work very well
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);

    public static final CameraMode CAMERA_MODE = CameraMode.REAL;

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
    public static final Double TARGET_DISTANCE_THROWAWAY_THRESHOLD = 4d; // meters

    public static final Double WEIGHT_SCALE = 5d;
}
