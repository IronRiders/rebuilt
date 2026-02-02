package org.ironriders.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static org.ironriders.manipulation.launcher.LauncherConstants.ESTIMATION_STARTING_DISTANCE;
import static org.ironriders.manipulation.launcher.LauncherConstants.G;
import static org.ironriders.manipulation.launcher.LauncherConstants.MAX_ROTATION;
import static org.ironriders.manipulation.launcher.LauncherConstants.MIN_ROTATION;
import static org.ironriders.manipulation.launcher.LauncherConstants.LAUNCHER_HIGHT;
import static org.ironriders.manipulation.launcher.LauncherConstants.TARGET_BALL_VELOCITY;

import java.util.Optional;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.manipulation.launcher.LauncherSubsystem;
import org.ironriders.lib.field.FieldPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;

public class BallisticsUtils {
    public static Pose2d getPosition() {
        return DriveSubsystem.getSwerveDrive().getPose();
    }

    // --- Distance ---
    public static double calculateDistanceToTarget() {
        return calculateDistanceToTarget(LauncherSubsystem.currentTarget);
    }

    public static double calculateDistanceToTarget(Pose3d pose) {
        return Utils.getPoseDifference(getPosition(), Utils.flattenPose3d(pose)).getNorm();
    }

    public static double calculateDistanceToHub() {
        return calculateDistanceToTarget(FieldPositions.get(ElementType.HUB));
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

    public static Optional<Double> estimateMaxRange() {
        double low = ESTIMATION_STARTING_DISTANCE;
        double high = ESTIMATION_STARTING_DISTANCE;
        double step = 1d;
        double loops = 0d;

        while (loops <= 25) {
            Double result = calculateAngleToTarget(FieldPositions.preparePose(FieldPositions.Hub.HUB_TOP), high)
                    .in(Radians);

            if (result == -1)
                break;

            low = high;
            high += step;
            step *= 2;
            loops += 1;
        }

        if (loops > 25)
            return Optional.empty();

        for (int i = 0; i < 50; i++) {
            double mid = (low + high) / 2.0;
            Double result = calculateAngleToTarget(FieldPositions.preparePose(FieldPositions.Hub.HUB_TOP), mid)
                    .in(Radians);

            if (result == -1)
                high = mid;
            else
                low = mid;
        }

        return Optional.of(low);
    }
}
