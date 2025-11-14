package org.ironriders.intake;

import static org.ironriders.intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


import org.ironriders.lib.IronSubsystem;

public class IntakeSubsystem extends IronSubsystem {
    private final TalonFX primaryMotor = new TalonFX(INTAKE_MOTOR_RIGHT);
    private final TalonFX leftIntake = new TalonFX(INTAKE_MOTOR_LEFT);
    private final TalonFX rollerMotor = new TalonFX(INTAKE_MOTOR_TOP);
    private final IntakeCommands commands = new IntakeCommands(this);
    
    public IntakeCommands getCommands() {
        return commands;
    }

    @Override
        public void periodic() {
            debugPublish("Left Velocity", leftIntake.getVelocity().getValue().in(Units.DegreesPerSecond));
            debugPublish("Right Velocity", primaryMotor.getVelocity().getValue().in(Units.DegreesPerSecond));
        }

    public void getMotors(IntakeConstants.IntakeState state) {
        leftIntake.set(state.speed);
        primaryMotor.set(state.speed);
        rollerMotor.set(state.speed);
    }

    public IntakeSubsystem() {
        
        TalonFXConfiguration mainConfig = new TalonFXConfiguration();
        mainConfig
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(INTAKE_STATOR_CURRENT)
                    .withSupplyCurrentLimit(INTAKE_SUPPLY_CURRENT)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(INTAKE_NEUTRAL_MODE)
            ); 

        primaryMotor.getConfigurator().apply(mainConfig);
        leftIntake.getConfigurator().apply(mainConfig);
        rollerMotor.getConfigurator().apply(mainConfig);

        intake.debugPublish("Intake Grab", set(IntakeState.GRAB));
        intake.debugPublish("Intake Score", set(IntakeState.SCORE));
        intake.debugPublish("Intake Stop", set(IntakeState.STOP));
    }
}