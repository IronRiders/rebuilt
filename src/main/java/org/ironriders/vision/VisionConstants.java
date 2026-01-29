package org.ironriders.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public static final double VISION_P = 0.1;
    public static final double VISION_I = 0.05;
    public static final double VISION_D = 0;
    public static final double VISION_ROTATION_MAX_SPEED = 2; // rad/s
    public static final String VISION_CAMERA = "main";

    public static final Double SKEW_THROWAWAY_THRESHOLD = 60d; // deg, TODO: Tune
    public static final Double DISTANCE_THROWAWAY_THRESHOLD = 6d; // meters, TODO: Tune


    public static final Transform3d CAMERA_OFFSET = new Transform3d( // idk just guessed
        new Translation3d(
            0.25, // forward (meters)
            0.0, // left (meters)
            0.5 // up (meters)
        ),
        new Rotation3d(
            0.0, // roll
            0.0, // pitch
            0.0 // yaw
        ));
}
