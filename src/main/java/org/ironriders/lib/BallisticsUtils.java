package org.ironriders.lib;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.manipulation.launcher.LauncherSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

/** Utilities for ballistic calculations. */
public class BallisticsUtils {

    /** @return current 2D position of the robot. */
    public static Pose2d getPosition() {
        return DriveSubsystem.getSwerveDrive().getPose();
    }

    /**
     * @return current 3D position of the robot using {@link #expandPose2d(Pose2d)}.
     */
    public static Pose3d get3dPosition() {
        return Utils.expandPose2d(getPosition());
    }

    // --- Distance ---

    /**
     * Calculate the distance to the current target from the current position.
     *
     * @return The distance to the current target.
     */
    public static double calculateDistanceToInternalTarget() {
        return calculateDistanceToTarget(LauncherSubsystem.currentTarget);
    }

    /**
     * Calculate the distance to a given target from the current position.
     *
     * @param pose The target pose.
     * @return The distance to the target.
     */
    public static double calculateDistanceToTarget(Pose3d pose) {
        return Utils.getPoseDifference(getPosition(), pose.toPose2d()).getNorm();
    }

    /**
     * Calculate the distance to the hub from the current position.
     *
     * @return The distance to the hub.
     */
    public static double calculateDistanceToHub() {
        return calculateDistanceToTarget(FieldPositions.get(ElementType.HUB));
    }

    /**
     * Check if a given pose is within range using {@link #inRange(double)}.
     *
     * @param inputPose The pose to check.
     * @return True if the pose is within range, false otherwise.
     */
    public static boolean inRange(Pose3d inputPose) {
        return inRange(Utils.getPose3dDifference(inputPose, Utils.expandPose2d(getPosition())).getNorm());
    }

    /** Uses the internal launcher range. */
    public static boolean inRange(double distance) {
        return Utils.inRange(LauncherSubsystem.range[0], LauncherSubsystem.range[1], distance);
    }

    public static Translation2d translationToPoint(
            Pose2d from,
            Pose2d to,
            double magnitude) {

        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();

        double currentMagnitude = Math.sqrt(dx * dx + dy * dy);

        if (currentMagnitude == 0) {
            return new Translation2d(0, 0);
        }

        double scale = magnitude / currentMagnitude;
        double vectorX = dx * scale;
        double vectorY = dy * scale;

        return new Translation2d(vectorX, vectorY);
    }

}
