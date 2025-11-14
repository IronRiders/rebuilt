package org.ironriders.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    public static final int INTAKE_MOTOR_RIGHT = 14;
    public static final int INTAKE_MOTOR_LEFT = 15;
    public static final int INTAKE_MOTOR_TOP = 16;

    public static final int INTAKE_STATOR_CURRENT = 20;
    public static final int INTAKE_SUPPLY_CURRENT = 30;

    public static final NeutralModeValue INTAKE_NEUTRAL_MODE = NeutralModeValue.Brake;

    public enum IntakeState {
        GRAB(.4),
        SCORE(-.30),
        STOP(0.00);
        
        public final double speed;
        IntakeState(double speed) {
            this.speed = speed;
        }
    
    }
}
