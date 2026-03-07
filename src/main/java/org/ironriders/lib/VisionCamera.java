package org.ironriders.lib;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionCamera {
    String m_name;
    Transform3d m_offset;
    Double m_trustWeight;

    PhotonCamera m_photonCamera;
    PhotonPoseEstimator m_estimator;

    PhotonPipelineResult m_mostRecent;

    public List<PhotonTrackedTarget> m_targets;

    private AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    @Override
    public String toString() {
        return String.format("VisionCamera{name='%s', offset=%s, trustWeight=%.2f, hasTarget=%s}",
                m_name, m_offset, m_trustWeight, m_mostRecent != null && m_mostRecent.hasTargets());
    }

    /*
     * Return the PhotonCamera instance.
     */
    public PhotonCamera getPhotonCamera() {
        return m_photonCamera;
    }

    /*
     * Returns the PhotonPoseEstimator instance we made in the constructor.
     */
    public PhotonPoseEstimator getEstimator() {
        return m_estimator;
    }

    /*
     * Returns the camera name set in the Photon Vision dashboard.
     */
    public String getSimpleName() {
        return m_name;
    }

    /*
     * Returns the capitalized version of the camera name set in the Photon Vision
     * dashboard.
     */
    public String getName() {
        return m_name.substring(0, 1).toUpperCase() + m_name.substring(1);
    }

    /*
     * Return the weight (trust) of this camera. Will be in the range [-1 (least
     * trusting) to 1 (most trusting)]. Zero is no weighing.
     */
    public Double getWeight() {
        return m_trustWeight;
    }

    /*
     * Get the camera's offset from the center of the robot.
     */
    public Transform3d getOffset() {
        return m_offset;
    }

    /*
     * Updates internal buffer and reads PhotonCamera results.
     * You should call this every tick exactly once. (Will probably be fine if
     * called more often)
     */
    public void updateResultBuffer() {
        List<PhotonPipelineResult> results = m_photonCamera.getAllUnreadResults();

        if (results == null || results.size() <= 0) {
            SmartDashboard.putString("VisionCamera/" + m_name + "/null response",
                    "null response from " + m_photonCamera.getName() + " at " + Timer.getFPGATimestamp());
            return; // don't update the buffer if we get a null response. Could be incorrect.
        }

        m_mostRecent = results.get(results.size() - 1); // get the most recent result.

        m_targets = m_mostRecent.targets;
    }

    /*
     * Get the most recent result from this camera.
     */
    public PhotonPipelineResult getResult() {
        if (m_mostRecent == null) {
            updateResultBuffer();
        }

        if (m_mostRecent == null) { // could still be null after the update, check again
            m_mostRecent = new PhotonPipelineResult(); // return an empty result.
        }

        return m_mostRecent;
    }

    /*
     * Get the list of all targets seen by this camera
     */
    public List<PhotonTrackedTarget> getTargets() {
        return m_targets;
    }

    /*
     * Do we see any targets?
     */
    public boolean seesTargets() {
        return getResult().hasTargets();
    }

    /*
     * Define a new camera.
     * 
     * @param name is the camera name set in the Photon Vision dashboard.
     * 
     * @param offset is the offset from the center of the robot, positive x towards
     * the battery.
     * 
     * @param trustWeight is the weight on the trust we have in estimations made
     * by this camera. Useful if you have one poor camera and one good one or
     * the like. Should be in the range [-1 (least trusting) to 1 (most
     * trusting)]. Zero is no weighing.
     */
    public VisionCamera(String name, Transform3d offset, Double trustWeight) {
        m_name = name;
        m_offset = offset;
        m_trustWeight = Utils.clamp(-1, 1, trustWeight);

        m_photonCamera = new PhotonCamera(name);
        m_estimator = new PhotonPoseEstimator(m_fieldLayout, offset);
    }

    /*
     * Define a new camera.
     * 
     * @param name is the camera name set in the Photon Vision dashboard.
     * 
     * @param offset is the offset from the center of the robot, positive x towards
     * the battery.
     */
    public VisionCamera(String name, Transform3d offset) {
        m_name = name;
        m_offset = offset;
        m_trustWeight = 0d;

        m_photonCamera = new PhotonCamera(name);
        m_estimator = new PhotonPoseEstimator(m_fieldLayout, offset);
    }
}
