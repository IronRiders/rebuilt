package org.ironriders.wrist;

/** Constants for the wrist subsystem. */
public class WristConstants {
  public static final Integer PRIMARY_WRIST_MOTOR = 12;

  public static final Integer SECONDARY_WRIST_MOTOR = 13;

  // TODO: Need to tune
  public static final double P = 0.020; // proportion
  public static final double I = 0.002; // integral
  public static final double D = 0.0; // derivative
  public static final double T = 0.02; // time to next step

  public static final double TOLERANCE = 1;

  public static final double MAX_ACC = 250;
  public static final double MAX_VEL = 350;

  public static final double ENCODER_SCALE = 1;
  public static final double CAD_POSITION_OFFSET = 50; // Adjustment for odd alignment in the CAD
  public static final double ENCODER_OFFSET =
      0.935; // Rotations for the absolute encoder to get to zero when in
  // FULLY stowed
  // The Arm should be inside the elevator at the start
}
