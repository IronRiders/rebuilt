package org.ironriders.vision;

import java.util.Optional;

import org.ironriders.vision.VisionCamera.PoseEstimate;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionCameraInterface {
    public record PoseEstimate(Pose2d estimatedPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    }

    public enum Mode {
        MULTI,
        SINGLE,
        OLD;
    }

    String getName();

    Transform3d getOffset();

    PhotonCamera getCamera();

    PhotonPoseEstimator getEstimator();

    Mode getMode();

    Matrix<N3, N1> getStdDevs();

    Optional<PoseEstimate> getPoseEstimate(PoseEstimate mode);

}
