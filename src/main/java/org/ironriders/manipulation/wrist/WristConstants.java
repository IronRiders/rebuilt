package org.ironriders.manipulation.wrist;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Constants for {@link org.ironriders.wrist.WristSubsystem WristSubsystem}
 */
public class WristConstants {

    public static final int MOTOR_ID = 17;
    public static final int ENCODER_ID = 20;

    public static final Double ENCODER_OFFSET = 0.3696;
    // 132.0d;

    public static final Double CURRENT_LIMIT = 40.0; // Current limit for the supply current
    public static InvertedValue MOTOR_INVERSION = InvertedValue.Clockwise_Positive;
    public static final Double P = 2.0; // proportional gain /TESTED but jerky needs feed forward
    public static final Double I = 0.0; // Integral gain
    public static final Double D = 0.1; // Derivative gain
    public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(100 / 360,
            100 / 360); // TODO

    public static final Double JOSTLE_RANGE = 25d / 360; // distance the wrist will move from the centerpoint while
                                                         // jostling; in degrees.
    public static final Double JOSTLE_TOLERANCE = 2d / 360; // in degrees.

    /**
     * Named positions of the wrist. 0 is all the way up, positive forward.
     */
    public enum State {
        UP(0),
        DOWN(0.282), //orginaly .282
        // Jostle is instead the midpoint of the motion.
        JOSTLE(0.15);

        public final double position;

        State(double position) {
            this.position = position;
        }
    }

    public static final double S = 0.00; // todo
    public static final double G = 0.0;
    public static final double V = 0;
}
