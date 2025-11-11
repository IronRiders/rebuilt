package org.ironriders.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {

    public static final int INTAKE_MOTOR_RIGHT = 14;
    public static final int INTAKE_MOTOR_LEFT = 15;
    public static final int INTAKE_MOTOR_TOP = 16;

    public static final InvertedValue INTAKE_MOTOR_LEFT_INVERSION = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue INTAKE_MOTOR_RIGHT_INVERSION = InvertedValue.Clockwise_Positive;
    public static final InvertedValue INTAKE_MOTOR_ROLLER_INVERSION = InvertedValue.CounterClockwise_Positive;

    public static final NeutralModeValue INTAKE_NEUTRAL_MODE = NeutralModeValue.Brake;

    public static final double LEFT_SPEED_MUL = 2;
    public static final double RIGHT_SPEED_MUL = 1;
    public static final double ROLLER_SPEED_MUL = 1;

    public static final double BOOST_TIME = 0.1;
    public static final double UNBOOST_TIME = 0.05;

    // TODO Tune These
    public static final int INTAKE_STATOR_CURRENT = 50; // Stator Current Torque and Acceleration
    public static final int INTAKE_SUPPLY_CURRENT = 80; // Supply Current Speed + (a little Torque). If Supply
    // Current Lower Time is <= 0 then this will be the Supply
    // Current
    public static final int INTAKE_SUPPLY_CURRENT_LOWER_LIMIT = 30; // Supply Current if SupplyCurrentLowerTime is
    // >0 and applies after
    public static final int INTAKE_SUPPLY_CURRENT_LOWER_TIME = 1; // In Seconds. This determines how long Supply
    // Current is applied and then limit is lowered to
    // SupplyCurrentLowerLimit

    public static final int INTAKE_BEAMBREAK = 0;

    public static final double DISCHARGE_TIMEOUT = 2; // these are both in SECONDS

    public enum IntakeState {
        GRAB(.4),
        SCORE(.30),
        BOOST(-.10), // This is so that changes to the eject speed won't effect the boost
        EJECT(-.10),
        STOP(0.00);

        public final double speed;

        IntakeState(double speed) {
            this.speed = speed;
        }
    }
}
