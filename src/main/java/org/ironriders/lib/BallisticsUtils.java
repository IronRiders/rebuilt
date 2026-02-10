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
import org.ironriders.manipulation.launcher.LauncherSubsystem;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
        return Utils.getPoseDifference(getPosition(), Utils.flattenPose3d(pose)).getNorm();
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

    // NOTE: currently broken (keeps for reference)
    /**
     * Returns a new {@link Pose3d} that is the closest point to {@code inputPose}
     * within the specified distance {@code range} from the {@code centerPoint},
     * preserving the direction from the center.
     *
     * @param inputPose   The pose to be adjusted.
     * @param range       The allowed distance range [max, min] from the centerPoint (in the XY
     *                    plane).
     * @param centerPoint The center point from which the range is measured.
     * @return A new {@link Pose3d} within the specified range from the centerPoint, or the
     *         original pose if already in range.
     */
    public static Pose3d snapPoseToRange(Pose3d inputPose, double[] range, Pose3d centerPoint) {
        double dx = inputPose.getX() - centerPoint.getX();
        double dy = inputPose.getY() - centerPoint.getY();

        double distanceXY = Math.sqrt(dx * dx + dy * dy);

        double targetDistance;
        if (distanceXY > range[0]) {
            targetDistance = range[0]; // Clamp to max
        } else if (distanceXY < range[1]) {
            targetDistance = range[1]; // Clamp to min
        } else {
            return inputPose;
        }

        distanceXY = (distanceXY == 0) ? 1e-6 : distanceXY;
        double scale = targetDistance / distanceXY;

        return new Pose3d(
                centerPoint.getX() + dx * scale,
                centerPoint.getY() + dy * scale,
                inputPose.getZ(),
                inputPose.getRotation());
    }

    /** Snap a pose to the given range around the current robot position. */
    public static Pose3d snapPoseToRange(Pose3d inputPose, double[] range) {
        return snapPoseToRange(inputPose, range, get3dPosition());
    }

    /** Snap a pose to the launcher subsystem range around the current robot position. */
    public static Pose3d snapPoseToRange(Pose3d inputPose) {
        return snapPoseToRange(inputPose, LauncherSubsystem.range, get3dPosition());
    }

    /** Snap a 2D pose to the launcher range (logs current range). */
    public static Pose2d snapPoseToRange(Pose2d inputPose) {
        DogLog.log(
                "Range-test-inner",
                "Big: " + String.valueOf(LauncherSubsystem.range[0]) + " | Small: "
                        + String.valueOf(LauncherSubsystem.range[1]));

        return Utils.flattenPose3d(
                snapPoseToRange(Utils.expandPose2d(inputPose), LauncherSubsystem.range, get3dPosition()));
    }

    // --- Angle ---

    /**
     * Calculate the angle to the hub from the current position.
     *
     * @return The angle to the hub, in radians.
     */
    public static Angle calculateAngleToHub() {
        return calculateAngleToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP));
    }

    /**
     * Calculate the angle to the current target from the current position.
     *
     * @return The angle to the current target, in radians. See {@link #calculateAngleToTarget(Pose3d)}
     *         for out-of-range values.
     */
    public static Angle calculateAngleToInternalTarget() {
        return calculateAngleToTarget(LauncherSubsystem.currentTarget);
    }

    /**
     * Calculate the angle to the specified target from the current position.
     *
     * @param target The target {@link Pose3d position}.
     * @return The angle to the target, in radians.
     */
    public static Angle calculateAngleToTarget(Pose3d target) {
        double distance = Utils.getPoseDifference(getPosition(), Utils.flattenPose3d(target)).getNorm();
        DogLog.log("distance", String.valueOf(distance));
        return calculateAngleToTarget(target, distance);
    }

    /**
     * Calculate the shooter angle required to hit a target from the current
     * position. (Formula adapted from contributor @Amber-leaf.)
     *
     * @param target   The target {@link Pose3d position}.
     * @param distance The distance to the target.
     * @return The angle of the shooter to hit the target. Returns sentinel angles when
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
        double lowAngle = Math.atan((v * v - sqrt) / (G * distance)) + Math.PI / 2;
        double highAngle = Math.atan((v * v + sqrt) / (G * distance)) + Math.PI / 2;

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
     * @return Optional containing a double[] { min, max } when successful, otherwise
     *         Optional.empty().
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
                Double result = calculateAngleToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP), mid)
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
