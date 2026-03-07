package org.ironriders.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static org.ironriders.manipulation.launcher.LauncherConstants.G;
import static org.ironriders.manipulation.launcher.LauncherConstants.LAUNCHER_HIGHT;
import static org.ironriders.manipulation.launcher.LauncherConstants.TARGET_BALL_VELOCITY;

import java.util.Optional;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.manipulation.launcher.LauncherMaps;
import org.ironriders.manipulation.launcher.LauncherSubsystem;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

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

    // --- Angle ---

    /**
     * Calculate the extension to the hub from the current position.
     *
     * @return The extension to the hub, in percentage of full e.
     */
    public static Optional<Double> calculateExtensionToHubKinematics() {
        return calculateExtensionToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP));
    }

    /**
     * Calculate the extension to the current target from the current position.
     *
     * @return The extension to the current target, in percentage of full extension.
     */
    public static Optional<Double> calculateExtensionToInternalTarget() {
        return calculateExtensionToTarget(LauncherSubsystem.currentTarget);
    }

    /**
     * Calculate the extension to the specified target from the current position.
     *
     * @param target The target {@link Pose3d position}.
     * @return The extension to the target, in percentage of full extension.
     */
    public static Optional<Double> calculateExtensionToTarget(Pose3d target) {
        double distance = Utils.getPoseDifference(getPosition(), target.toPose2d()).getNorm();
        DogLog.log("Launcher/Distance to target", String.valueOf(distance));
        return calculateExtensionToTarget(target, distance);
    }

    public static Optional<Double> calculateExtensionToTarget(Pose3d target, double distance) {
        double v = TARGET_BALL_VELOCITY;
        double y = target.getZ() - LAUNCHER_HIGHT;

        double inner = Math.pow(v, 4) - G * (G * distance * distance + 2 * y * v * v);

        if (inner < 0) {
            return Optional.empty();
        }

        double sqrt = Math.sqrt(inner);
        double lowAngle = Math.atan((v * v - sqrt) / (G * distance));
        double highAngle = Math.atan((v * v + sqrt) / (G * distance));

        if (Utils.inRange(LauncherMaps.AngleToExtensionMap.getAngleForExtension(0),
                LauncherMaps.AngleToExtensionMap.getAngleForExtension(1), highAngle)) {
            return Optional.of(LauncherMaps.AngleToExtensionMap.getExtensionForAngle(Math.toDegrees(highAngle)));
        } else if (Utils.inRange(LauncherMaps.AngleToExtensionMap.getAngleForExtension(0),
                LauncherMaps.AngleToExtensionMap.getAngleForExtension(1), lowAngle)) {
            return Optional.of(LauncherMaps.AngleToExtensionMap.getExtensionForAngle(Math.toDegrees(lowAngle)));
        } else {
            return Optional.empty();
        }
    }
}
