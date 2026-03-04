package org.ironriders.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static org.ironriders.manipulation.launcher.LauncherConstants.G;
import static org.ironriders.manipulation.launcher.LauncherConstants.LAUNCHER_HIGHT;
import static org.ironriders.manipulation.launcher.LauncherConstants.MAX_ROTATION;
import static org.ironriders.manipulation.launcher.LauncherConstants.MIN_ROTATION;
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
    public static double calculateExtensionToHub() {
        return calculateExtensionToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP));
    }

    /**
     * Calculate the extension to the current target from the current position.
     *
     * @return The extension to the current target, in percentage of full extension.
     */
    public static double calculateExtensionToInternalTarget() {
        return calculateExtensionToTarget(LauncherSubsystem.currentTarget);
    }

    /**
     * Calculate the extension to the specified target from the current position.
     *
     * @param target The target {@link Pose3d position}.
     * @return The extension to the target, in percentage of full extension.
     */
    public static double calculateExtensionToTarget(Pose3d target) {
        double distance = Utils.getPoseDifference(getPosition(), target.toPose2d()).getNorm();
        DogLog.log("Launcher/Distance to target", String.valueOf(distance));
        return LauncherMaps.distanceToExtension.getExtensionForDistance(distance);
    }

    /**
     * Calculate the launcher angle required to hit a target from the current
     * position.
     *
     * @param target   The target {@link Pose3d position}.
     * @param distance The distance to the target.
     * @return The angle of the launcher to hit the target. Returns sentinel angles
     *         when
     *         target is unreachable.
     */
    public static Angle calculateAngleToTarget(Pose3d target, double distance) {
        double v = TARGET_BALL_VELOCITY;
        double y = target.getZ() - LAUNCHER_HIGHT;

        double inner = Math.pow(v, 4) - G * (G * distance * distance + 2 * y * v * v);

        if (inner < 0) {
            return Angle.ofBaseUnits(-1d, Degrees);
        }

        double sqrt = Math.sqrt(inner);
        double lowAngle = Math.atan((v * v - sqrt) / (G * distance));
        double highAngle = Math.atan((v * v + sqrt) / (G * distance));

        DogLog.log("Launcher/Angles", "high: " + String.valueOf(highAngle) + " low: " + String.valueOf(lowAngle));

        if (Utils.inRange(MIN_ROTATION, MAX_ROTATION, highAngle)) {
            return Angle.ofBaseUnits(highAngle, Radians);
        } else if (Utils.inRange(MIN_ROTATION, MAX_ROTATION, lowAngle)) {
            return Angle.ofBaseUnits(lowAngle, Radians);
        } else {
            return Angle.ofBaseUnits(-2d, Degrees);
        }
    }

    /**
     * Estimate the minimum and maximum ranges (distance) where the hub can be hit.
     *
     * @return Optional containing a double[] { min, max } when successful,
     *         otherwise
     *         Optional.empty().
     * 
     *         This code really sucks.
     */
    public static Optional<double[]> estimateMinMaxRange() {
        double low = 0d;
        double high = 0d;
        double step = 1d;
        int loops = 0;
        double lastResult = -2; // Start assuming we're too close
        double minLow = 0d;
        double minHigh = 0d;
        boolean foundMinRegion = false;

        while (loops <= 25) {
            double result = calculateAngleToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP), high)
                    .in(Radians);

            if (result == -1) {
                break;
            }

            if (lastResult == -2 && result != -2) {
                minLow = low;
                minHigh = high;
                foundMinRegion = true;
            }

            low = high;
            high += step;
            step *= 2;
            loops += 1;
            lastResult = result;
        }

        if (loops > 25) {
            DogLog.log("exit early on range estimate", String.valueOf(true));
            return Optional.empty();
        }

        double maxLow = low;
        double maxHigh = high;

        // Binary search for min region
        double min = 0d;
        if (foundMinRegion) {
            for (int i = 0; i < 20; i++) {
                double mid = (minLow + minHigh) / 2.0;
                Double result = calculateAngleToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP),
                        mid)
                        .in(Radians);

                if (result == -2) {
                    minLow = mid;
                } else {
                    minHigh = mid;
                }
            }
            min = minHigh;
        }

        // Binary search for max region
        for (int i = 0; i < 20; i++) {
            double mid = (maxLow + maxHigh) / 2.0;
            Double result = calculateAngleToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP), mid)
                    .in(Radians);

            if (result == -1) {
                maxHigh = mid;
            } else {
                maxLow = mid;
            }
        }
        double max = maxLow;

        return Optional.of(new double[] { min, max });
    }
}
