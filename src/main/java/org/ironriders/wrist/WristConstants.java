package org.ironriders.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Constants for {@linkplain org.ironriders.wrist.WristSubsystem WristSubsystem}
 */
public class WristConstants {
    public static final int WRIST_MOTOR_CANT_ID = 0b1011011000011;
    public static final double WRIST_MOTOR_SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double PID_PROPORTIONAL = 0.0; // TODO: Test
    public static final double PID_INTEGRAL = 0.0;
    public static final double PID_DERIVATIVE = 0.0;
    public static final TrapezoidProfile.Constraints PID_CONSTRAINTS = null;

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
        public static final double STATIC_FRICTION_OVERCOME_VOLTAGE = 0; // TODO: calculate this
                                                                         // (https://www.reca.lc/arm)
        public static final double MAINTAIN_VELOCITY_VOLTAGE = 0;
        public static final double GRAVITY_OVERCOME_VOLTAGE = 0;
    }
}
