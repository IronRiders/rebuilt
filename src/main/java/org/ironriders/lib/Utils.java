package org.ironriders.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

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
  public static double percentOfMaxVoltage(double voltage, int maxVoltage) {
    return (voltage / maxVoltage);
  }

  public static Pose2d flattenPose3d(Pose3d pose) {
    return new Pose2d(new Translation2d(pose.getX(), pose.getY()), new Rotation2d(pose.getRotation().getAngle()));
  }

  public static Translation2d getPoseDifference(Pose2d pose1, Pose2d pose2) {
    return new Translation2d(pose1.getX() - pose2.getX(), pose1.getY() - pose2.getY());
  }

  public static Translation3d getPose3dDifference(Pose3d pose1, Pose3d pose2) {
    return new Translation3d(pose1.getX() - pose2.getX(), pose1.getY() - pose2.getY(), pose1.getZ() - pose2.getZ());
  }

  public static double clamp(double min, double max, double in) {
    if (in > max) {
      in = max;
    }
    if (in < min) {
      in = min;
    }
    return in;
  }

  public static boolean inRange(double min, double max, double in) {
    if (in > max) {
      return false;
    } else if (in < min) {
      return false;
    } else {
      return true;
    }
  }

  public static int[] everyIntInRange(int min, int max) {
    int[] out = { 0 };

    if (max < min) {
      return out;
    }

    for (int i = 0; min < max; i++) {
      out[i] = min + i;
    }

    return out;
  }
}