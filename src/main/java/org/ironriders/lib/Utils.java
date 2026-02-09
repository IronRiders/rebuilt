package org.ironriders.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Utility class to encourage the robot's dangerous math addiction. */
public class Utils {

    /**
     * Applies a control curve (currently just an exponential function) Only works
     * with input values
     * from 0 to 1 because 1^x = 1.
     *
     * @param input    The value to put into the curve (0.0 - 1.0 ONLY)
     * @param deadband The exponent value.
     * @return The end result of the curve.
     */
    public static double controlCurve(double input, double exponent, double deadband) {
        return Math.pow(input, exponent);
    }

    /**
     * Normalizes a rotational input value to the range [0, 360) degrees.
     *
     * @param input The input rotational value.
     * @return The normalized rotational value within the range [0, 360) degrees.
     */
    public static double absoluteRotation(double input) {
        return (input % 360 + 360) % 360;
    }

    /**
     * Normalizes Added Voltage from Feed Forward to a number between (0.0, 1.0).
     *
     * @param input The additional voltage.
     * @return The normalized voltage value within the range (0.0, 1.0).
     */
    public static double percentOfMaxVoltage(double voltage, double maxVoltage) {
        if (maxVoltage == 0) {
            maxVoltage = 0.000001;
        }

        return voltage / maxVoltage;
    }

    /**
     * Flattens a {@link Pose3d} to a {@link Pose2d} by dropping the z
     * value and converting the rotation to a {@link Rotation2d}.
     * 
     * @param pose The {@link Pose3d} to flatten.
     * @return The flattened {@link Pose2d}.
     */
    public static Pose2d flattenPose3d(Pose3d pose) {
        return new Pose2d(new Translation2d(pose.getX(), pose.getY()), new Rotation2d(pose.getRotation().getAngle()));
    }

    /**
     * Gets the difference between two poses as a {@link Translation2d}. Does
     * not use rotation.
     * 
     * @param pose1 The first pose.
     * @param pose2 The second pose.
     * @return The difference between the two poses as a {@link Translation2d}.
     */
    public static Translation2d getPoseDifference(Pose2d pose1, Pose2d pose2) {
        return new Translation2d(pose1.getX() - pose2.getX(), pose1.getY() - pose2.getY());
    }

    /**
     * Gets the difference between two poses as a {@link Translation3d}. Does
     * not use rotation.
     * 
     * @param pose1 The first pose.
     * @param pose2 The second pose.
     * @return The difference between the two poses as a {@link Translation3d}.
     */
    public static Translation3d getPose3dDifference(Pose3d pose1, Pose3d pose2) {
        return new Translation3d(pose1.getX() - pose2.getX(), pose1.getY() - pose2.getY(), pose1.getZ() - pose2.getZ());
    }

    /**
     * Clamps a value between a minimum and maximum range.
     * 
     * @param min The minimum value.
     * @param max The maximum value.
     * @param in  The input value to clamp.
     * @return The clamped value.
     */
    public static double clamp(double min, double max, double in) {
        if (in > max) {
            in = max;
        }
        if (in < min) {
            in = min;
        }
        return in;
    }

    /**
     * Checks if a value is within a specified range.
     * 
     * @param min The minimum value.
     * @param max The maximum value.
     * @param in  The input value to check.
     * @return True if the input value is within the range, false otherwise.
     */
    public static boolean inRange(double min, double max, double in) {
        if (in > max) {
            return false;
        } else if (in < min) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Generates an array of all integers within a specified range, inclusive.
     * 
     * Can this be replaced by
     * {@code IntStream.range(Math.min(a, b), Math.max(a, b) + 1).toArray()}?
     * 
     * @param a The start of the range.
     * @param b The end of the range.
     * @return An array containing all integers from the smaller of a and b to the
     *         larger of a and b.
     */
    public static int[] everyIntInRange(int a, int b) {
        int start = Math.min(a, b);
        int end = Math.max(a, b);

        int[] result = new int[end - start + 1];
        for (int i = 0; i < result.length; i++) {
            result[i] = start + i;
        }

        return result;
    }

    /**
     * Converts a {@link Pose3d} from inches to meters (multiplies by 0.0254).
     * 
     * @param pose The {@link Pose3d} to convert, in inches.
     * @return The converted {@link Pose3d} in meters.
     */
    public static Pose3d inchesToMeters(Pose3d pose) {
        return new Pose3d(pose.getTranslation().times(0.0254), pose.getRotation());
    }

    /**
     * Converts a {@link Pose2d} from inches to meters (multiplies by 0.0254).
     * 
     * @param pose The {@link Pose2d} to convert, in inches.
     * @return The converted {@link Pose2d} in meters.
     */
    public static Pose2d inchesToMeters(Pose2d pose) {
        return new Pose2d(pose.getTranslation().times(0.0254), pose.getRotation());
    }

    /**
     * Expands a {@link Pose2d} to a {@link Pose3d} by adding a z value of
     * 0 and converting the rotation to a {@link Rotation3d}.
     * 
     * @param pose The {@link Pose2d} to expand.
     * @return The expanded {@link Pose3d}.
     */
    public static Pose3d expandPose2d(Pose2d pose) {
        return new Pose3d(pose.getX(), pose.getY(), 0d, new Rotation3d(pose.getRotation()));
    }

    public static double getAngleToPointRadians(Pose2d p1, Pose2d p2) {
        double deltaX = p2.getX() - p1.getX();
        double deltaY = p2.getY() - p1.getY();
        return Math.atan2(deltaY, deltaX);
    }

    public static double getAngleToPoint(Pose2d p1, Pose2d p2) {
        return Math.toDegrees(getAngleToPointRadians(p1, p2));
    }
}