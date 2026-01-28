package org.ironriders.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Constants for {@linkplain org.ironriders.wrist.WristSubsystem WristSubsystem}
 */
public class WristConstants {

    /* MOTOR CONSTANTS */
    public static final int MOTOR_ID = 0b1011011000011;
    public static final double CURRENT_LIMIT = 40.0; // Current limit for the supply current (not for the current in the
                                                     // motor including regen)
                                                     // in Amps
    public static final double P = 0.0; // TODO: Test // proportional gain
    public static final double I = 0.0; // Integral gain
    public static final double D = 0.0; // Derivative gain
    public static final TrapezoidProfile.Constraints CONSTRAINTS = null;

    /**
     * Wrist positions (Up & Down) in degrees from horizontal (interchangable with
     * {@linkplain edu.wpi.first.math.trajectory.TrapezoidProfile TrapezoidProfiles}
     * with a velocity of 0).
     */
    public enum WristPositions {
        UP(0.0), // TODO: Find these later
        DOWN(0.0);

        public final double POSITION;

        WristPositions(double POSITION) {
            this.POSITION = POSITION;
        }
    }

    /**
     * Inner class for feed forward voltages
     */
    public class FeedForward {
        public static final double FRICTION = 0; // TODO: calculate this // Static friction overcome voltage
                                                 // (https://www.reca.lc/arm)
        public static final double COASTING = 0; // Voltage required to maintain velocity
        public static final double GRAVITY = 0; // Voltage required to overcome gravity
    }
}
