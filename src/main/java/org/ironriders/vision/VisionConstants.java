package org.ironriders.vision;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Constants for the {@link VisionSubsystem} */
public class VisionConstants {
    public enum CAMERA {
        LAUNCHER_BACK(true, "launcher-back", new Transform3d(
                    new Translation3d(
                            0.45,
                            -0.26,
                            0.19),
                    new Rotation3d(
                            0.0,
                            -1 * Math.toRadians(15),
                            0.0))),
        LAUNCHER_BACK_HIGH(false, "launcher-back-high", new Transform3d(
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
        LAUNCHER_HOOD(true, "launcher-hood",  new Transform3d(
                    new Translation3d(
                            0,
                            0,
                            0.498),
                    new Rotation3d(
                            0.0,
                            -Math.toRadians(20),
                            Math.PI))),
        SWERVE_BACK_LEFT(false, "swerve-back-left", new Transform3d(
                    new Translation3d(
                            0.212,
                            0.218,
                            0.16),
                    new Rotation3d(
                            0.0,
                            -Math.toRadians(25),
                            Math.toRadians(45)))),
        SWERVE_BACK_RIGHT(false, "swerve-back-right", new Transform3d(
                    new Translation3d(
                            0.212,
                            -0.218,
                            0.16),
                    new Rotation3d(
                            0.0,
                            -Math.toRadians(25),
                            -Math.toRadians(45))));
        public final boolean isEnabled;
        public final String cameraName;
        public final Transform3d robotToCamera;
        CAMERA(boolean isEnabled, String cameraName, Transform3d robotToCamera) {
            this.cameraName = cameraName;
            this.isEnabled = isEnabled;
            this.robotToCamera = robotToCamera;
        }
    }

    // These are marked as "example values" in the code, but according to XBot they
    // work very well during comp
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);

    public static final AprilTagFieldLayout TAG_FIELD_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.kDefaultField);
    public static final SimCameraProperties SIM_CAM_PROPS = new SimCameraProperties() // todo: get actual values
            .setCalibration(960, 720, Rotation2d.fromDegrees(90))
            .setCalibError(0.35, 0.1)
            .setFPS(15)
            .setAvgLatencyMs(50)
            .setLatencyStdDevMs(50);
}
