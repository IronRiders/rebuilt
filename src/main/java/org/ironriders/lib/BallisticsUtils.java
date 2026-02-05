package org.ironriders.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static org.ironriders.manipulation.launcher.LauncherConstants.G;
import static org.ironriders.manipulation.launcher.LauncherConstants.HEIGHT_DIFFERENCE_HUB_TO_LAUNCHER;
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

    // Currently broken
    /// *
    // * Get the closest pose to @param inputPose that is in the @param range around
    // the @param centerPoint.
    // */
    // public static Pose3d snapPoseToRange(Pose3d inputPose, double[] range, Pose3d
    // centerPoint) {
    // double dx = inputPose.getX() - centerPoint.getX();
    // double dy = inputPose.getY() - centerPoint.getY();
    //
    // double distanceXY = Math.sqrt(dx * dx + dy * dy);
    //
    // double targetDistance;
    // if (distanceXY > range[0]) {
    // targetDistance = range[0];
    // } else if (distanceXY < range[1]) {
    // targetDistance = range[1];
    // } else {
    // return inputPose;
    // }
    //
    //
    // distanceXY = distanceXY == 0 ? 0.000001 : distanceXY;
    // double scale = targetDistance / distanceXY;
    //
    // return new Pose3d(centerPoint.getX() + dx * scale, centerPoint.getY() + dy *
    // scale, inputPose.getZ(),
    // inputPose.getRotation());
    // }
    //
    /// *
    // * Get the closest pose to @param inputPose that is in @param range.
    // */
    // public static Pose3d snapPoseToRange(Pose3d inputPose, double[] range) {
    // return snapPoseToRange(inputPose, range, get3dPosition());
    // }
    //
    /// *
    // * Get the closest pose to @param inputPose that is in the LauncherSubsystem's
    // range.
    // */
    // public static Pose3d snapPoseToRange(Pose3d inputPose) {
    // return snapPoseToRange(inputPose, LauncherSubsystem.range, get3dPosition());
    // }
    //
    /// *
    // * Get the closest pose to @param inputPose that is in the LauncherSubsystem's
    // range.
    // */
    // public static Pose2d snapPoseToRange(Pose2d inputPose) {
    // DogLog.log("Range-test-inner", "Big: " +
    // String.valueOf(LauncherSubsystem.range[0]) + " | Small: " +
    // String.valueOf(LauncherSubsystem.range[1]));
    // return Utils.flattenPose3d(snapPoseToRange(Utils.expandPose2d(inputPose),
    // LauncherSubsystem.range, get3dPosition()));
    // }

    // --- Angle ---
    public static Angle calculateAngleToHub() {
        return calculateAngleToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP));
    }

    public static Angle calculateAngleToInternalTarget() {
        return calculateAngleToTarget(LauncherSubsystem.currentTarget);
    }

    public static Angle calculateAngleToTarget(Pose3d target) {
        double distance = Utils.getPoseDifference(getPosition(), Utils.flattenPose3d(target)).getNorm();
        DogLog.log("distance", String.valueOf(distance));
        return calculateAngleToTarget(target, distance);
    }

    public static Angle calculateAngleToTarget(Pose3d target, double distance) {
        double v = TARGET_BALL_VELOCITY;
        double y = target.getZ() - LAUNCHER_HIGHT;

        DogLog.log("Y | D", String.valueOf(y) + " | " + String.valueOf(distance));

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
        int loops = 0;
        double lastResult = -2; // Start assuming we're too close
        double minLow = 0d;
        double minHigh = 0d;
        boolean foundMinRegion = false;

        while (loops <= 25) {
            Double result = calculateAngleToTarget(
                    FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP),
                    high).in(Radians);

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

        // Binary search
        double min = 0d;
        if (foundMinRegion) {
            for (int i = 0; i < 20; i++) {
                double mid = (minLow + minHigh) / 2.0;
                Double result = calculateAngleToTarget(
                        FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP),
                        mid).in(Radians);

                if (result == -2) {
                    minLow = mid;
                } else {
                    minHigh = mid;
                }
            }
            min = minHigh;
        }

        // Binary search
        for (int i = 0; i < 20; i++) {
            double mid = (maxLow + maxHigh) / 2.0;
            Double result = calculateAngleToTarget(
                    FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP),
                    mid).in(Radians);

            if (result == -1) {
                maxHigh = mid;
            } else {
                maxLow = mid;
            }
        }
        double max = maxLow;

        return Optional.of(new double[] { min, max });
    }

    public static double[] calculateOptimalAngles() {
        double realMin = Math.toRadians(5);
        double realMax = Math.toRadians(85);

        double[] best = { 0, 0 };
        double lowRange = 0;
        double highRange = 0;
        double bestRangeWidth = 0;

        for (double i = realMin; i < realMax; i += Math.toRadians(3)) {
            for (double j = i + Math.toRadians(10); j < realMax; j += Math.toRadians(3)) {
                double[] newRange = tempestimateMinMaxRange(i, j)
                        .orElse(new double[] { 0, 0 });

                if (newRange[0] < 0.1) {
                    continue;
                }

                double newRangeWidth = newRange[1] - newRange[0];

                if (newRangeWidth > bestRangeWidth) {
                    best = newRange;
                    lowRange = i;
                    highRange = j;
                    bestRangeWidth = newRangeWidth;
                }
            }
        }

        if (bestRangeWidth > 0) {
            double searchMin = Math.max(realMin, lowRange - Math.toRadians(10));
            double searchMax = Math.min(realMax, highRange + Math.toRadians(10));

            for (double i = searchMin; i < lowRange + Math.toRadians(5); i += Math.toRadians(1)) {
                for (double j = highRange - Math.toRadians(5); j < searchMax; j += Math.toRadians(1)) {
                    if (j <= i + Math.toRadians(5))
                        continue;

                    double[] newRange = tempestimateMinMaxRange(i, j)
                            .orElse(new double[] { 0, 0 });

                    if (newRange[0] < 0.1)
                        continue;

                    double newRangeWidth = newRange[1] - newRange[0];

                    if (newRangeWidth > bestRangeWidth) {
                        best = newRange;
                        lowRange = i;
                        highRange = j;
                        bestRangeWidth = newRangeWidth;
                    }
                }
            }
        }

        return new double[] { best[0], best[1], lowRange, highRange };
    }

    public static Angle tempcalculateAngleToTarget(Pose3d target, double distance, double min, double max) {
        double v = TARGET_BALL_VELOCITY;

        double y = target.getZ() - LAUNCHER_HIGHT;

        double inner = Math.pow(v, 4) - G * (G * distance * distance + 2 * y * v * v);

        if (inner < 0) {
            return Angle.ofBaseUnits(-1d, Degrees);
        }

        double sqrt = Math.sqrt(inner);
        double lowAngle = Math.atan((v * v - sqrt) / (G * distance));
        double highAngle = Math.atan((v * v + sqrt) / (G * distance));

        if (Utils.inRange(min, max, highAngle)) {
            return Angle.ofBaseUnits(highAngle, Radians);
        } else if (Utils.inRange(min, max, lowAngle)) {
            return Angle.ofBaseUnits(lowAngle, Radians);
        } else {
            return Angle.ofBaseUnits(-2d, Degrees);
        }
    }

    public static Optional<double[]> tempestimateMinMaxRange(double minRot, double maxRot) {
        double low = 0d;
        double high = 0d;
        double step = 1d;
        int loops = 0;
        double lastResult = -2; // Start assuming we're too close
        double minLow = 0d;
        double minHigh = 0d;
        boolean foundMinRegion = false;

        while (loops <= 25) {
            Double result = tempcalculateAngleToTarget(
                    FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP),
                    high, minRot, maxRot).in(Radians);

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

        // Binary search
        double min = 0d;
        if (foundMinRegion) {
            for (int i = 0; i < 20; i++) {
                double mid = (minLow + minHigh) / 2.0;
                Double result = tempcalculateAngleToTarget(
                        FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP),
                        mid, minRot, maxRot).in(Radians);

                if (result == -2) {
                    minLow = mid;
                } else {
                    minHigh = mid;
                }
            }
            min = minHigh;
        }

        // Binary search
        for (int i = 0; i < 20; i++) {
            double mid = (maxLow + maxHigh) / 2.0;
            Double result = tempcalculateAngleToTarget(
                    FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP),
                    mid, minRot, maxRot).in(Radians);

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
