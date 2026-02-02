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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public class BallisticsUtils {
    public static Pose2d getPosition() {
        return DriveSubsystem.getSwerveDrive().getPose();
    }

    public static Pose3d get3dPosition() {
        return Utils.expandPose2d(getPosition());
    }

    // --- Distance ---
    public static double calculateDistanceToInternalTarget() {
        return calculateDistanceToTarget(LauncherSubsystem.currentTarget);
    }

    public static double calculateDistanceToTarget(Pose3d pose) {
        return Utils.getPoseDifference(getPosition(), Utils.flattenPose3d(pose)).getNorm();
    }

    public static double calculateDistanceToHub() {
        return calculateDistanceToTarget(FieldPositions.get(ElementType.HUB));
    }

    public static boolean inRange(Pose3d inputPose) {
        return inRange(Utils.getPose3dDifference(inputPose, Utils.expandPose2d(getPosition())).getNorm());
    }

    /*
     * Uses the internal range
     */
    public static boolean inRange(double distance) {
        return Utils.inRange(LauncherSubsystem.range[0], LauncherSubsystem.range[1], distance);
    }

    /*
     * Get the closest pose to @param inputPose that is in the @param range around the @param centerPoint.
     */
    public static Pose3d snapPoseToRange(Pose3d inputPose, double[] range, Pose3d centerPoint) {
        double dx = inputPose.getX() - centerPoint.getX();
        double dy = inputPose.getY() - centerPoint.getY();

        double distanceXY = Math.sqrt(dx * dx + dy * dy);

        double targetDistance;
        if (distanceXY > range[0]) {
            targetDistance = range[0];
        } else if (distanceXY < range[1]) {
            targetDistance = range[1];
        } else {
            return inputPose;
        }

        double scale = targetDistance / distanceXY;

        return new Pose3d(centerPoint.getX() + dx * scale, centerPoint.getY() + dy * scale, inputPose.getZ(),
                inputPose.getRotation());
    }


    /*
     * Get the closest pose to @param inputPose that is in @param range.
     */
    public static Pose3d snapPoseToRange(Pose3d inputPose, double[] range) {
        return snapPoseToRange(inputPose, range, get3dPosition());
    }

    /*
     * Get the closest pose to @param inputPose that is in the LauncherSubsystem's range.
     */
    public static Pose3d snapPoseToRange(Pose3d inputPose) {
        return snapPoseToRange(inputPose, LauncherSubsystem.range, get3dPosition());
    }

    // --- Angle ---
    public static Angle calculateAngleToHub() {
        return calculateAngleToTarget(FieldPositions.preparePose(FieldPositions.Hub.HUB_TOP));
    }

    public static Angle calculateAngleToInternalTarget() {
        return calculateAngleToTarget(LauncherSubsystem.currentTarget);
    }

    public static Angle calculateAngleToTarget(Pose3d target) {
        double distance = Utils.getPoseDifference(getPosition(), Utils.flattenPose3d(target)).getNorm();
        return calculateAngleToTarget(target, distance);
    }

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

        if (Utils.inRange(MIN_ROTATION, MAX_ROTATION, highAngle)) {
            return Angle.ofBaseUnits(highAngle, Radians);
        } else if (Utils.inRange(MIN_ROTATION, MAX_ROTATION, lowAngle)) {
            return Angle.ofBaseUnits(lowAngle, Radians);
        } else {
            return Angle.ofBaseUnits(-2d, Degrees);
        }
    }

    public static Optional<double[]> estimateMinMaxRange() {
        double low = 0d;
        double high = 0d;
        double step = 1d;
        double loops = 0d;
        double lastResult = 0;
        double min = 0d;

        while (loops <= 25) {
            Double result = calculateAngleToTarget(FieldPositions.preparePose(FieldPositions.Hub.HUB_TOP), high)
                    .in(Radians);

            if (result == -1) {
                break;
            }

            if (lastResult == -2 && result != -2) {
                min = result;
            }

            low = high;
            high += step;
            step *= 2;
            loops += 1;
            lastResult = result;
        }

        if (loops > 25)
            return Optional.empty();

        for (int i = 0; i < 50; i++) {
            double mid = (low + high) / 2.0;
            double result = calculateAngleToTarget(FieldPositions.preparePose(FieldPositions.Hub.HUB_TOP), mid)
                    .in(Radians);

            if (result == -1)
                high = mid;
            else
                low = mid;
        }

        DogLog.log("Range-test", "Big: " + String.valueOf(low) + " | Small: " + String.valueOf(min));

        return Optional.of(new double[] { low, min });
    }
}
