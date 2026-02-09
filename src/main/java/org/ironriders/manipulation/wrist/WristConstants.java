package org.ironriders.manipulation.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Constants for {@link org.ironriders.wrist.WristSubsystem WristSubsystem}
 */
public class WristConstants {

    /* MOTOR CONSTANTS */
    public static final int MOTOR_ID = 13;
    public static final double CURRENT_LIMIT = 40.0; // Current limit for the supply current
    public static final double P = 0.5; // proportional gain
    public static final double I = 0.0; // Integral gain
    public static final double D = 0.1; // Derivative gain
    public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(100, 100); // TODO:
                                                                                                               // Test
                                                                                                               // later

    /**
     * Wrist positions (Up & Down) in degrees from horizontal (interchangeable with
     * {@link edu.wpi.first.math.trajectory.TrapezoidProfile TrapezoidProfiles}
     * with a velocity of 0).
     */
    public enum State {
        UP(90.0),
        DOWN(0.0),
        JOSTLE(30.0);

        public final double position;

        State(double position) {
            this.position = position;
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
