package org.ironriders.manipulation.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Constants for {@link org.ironriders.wrist.WristSubsystem WristSubsystem}
 */
public class WristConstants {

    public static final int MOTOR_ID = 17;
    public static final int ENCODER_ID = 20;

    public static final Double ENCODER_OFFSET = 0d; // TODO

    public static final Double CURRENT_LIMIT = 40.0; // Current limit for the supply current
    public static final Double P = 0.5; // proportional gain
    public static final Double I = 0.0; // Integral gain
    public static final Double D = 0.1; // Derivative gain
    public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(100, 100); // TODO

    public static final Double JOSTLE_RANGE = 20d; // distance the wrist will move from the centerpoint while jostling; in degrees.
    public static final Double JOSTLE_TOLERANCE = 2d; // in degrees.


    /**
     * Named positions of the wrist. 0 is all the way up, positive forward.
     */
    public enum State {
        UP(0),
        DOWN(100),
        // Jostle is instead the midpoint of the motion.
        JOSTLE(45);

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
