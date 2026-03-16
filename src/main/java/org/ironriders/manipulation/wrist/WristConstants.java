package org.ironriders.manipulation.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

import com.ctre.phoenix6.controls.PositionVoltage;

/**
 * Constants for {@link org.ironriders.wrist.WristSubsystem WristSubsystem}
 */
public class WristConstants {

    public static final int MOTOR_ID = 17;
    public static final int ENCODER_ID = 20;

    public static final Double ENCODER_OFFSET = 0.0339;
    // 132.0d;

    public static final Double CURRENT_LIMIT = 40.0; // Current limit for the supply current
    public static InvertedValue MOTOR_INVERSION = InvertedValue.Clockwise_Positive;
    public static final Double P = 1.5; // proportional gain /TESTED but jerky needs feed forward
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
        DOWN(0.282), // orginaly .282
        // Jostle is instead the midpoint of the motion.
        JOSTLE(0.15);

        public final double position;
        public final PositionVoltage posvol;

        State(double position) {
            this.position = position;
            this.posvol = new PositionVoltage(position);
        }
    }

    public static final double S = 0.00; // todo
    public static final double G = 0.0;
    public static final double V = 0;
    public static final Angle FORWARD_LIMIT = Degrees.of(0.0);
    public static final Angle REVERSE_LIMIT = Degrees.of(0.0);

    public static final double MECHANISM_RATIO = 1; // TODO: get;
    public static final double ERROR_TOLERANCE = 0.01; // TODO: get
    public static final AngularVelocity HOME_VELOCITY_THRESHOLD = DegreesPerSecond.of(5);
    public static final Time HOME_TIMEOUT = Seconds.of(20);
}
