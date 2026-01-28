package org.ironriders.manipulation.intake;

import org.ironriders.lib.IronSubsystem;
import org.ironriders.manipulation.intake.IntakeConstants.State;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem extends IronSubsystem {
    private final IntakeCommands commands = new IntakeCommands(this);

    private final TalonFX motor = new TalonFX(IntakeConstants.ID);
    private TalonFXConfiguration configuration;
    public IntakeSubsystem() {
        configuration = new TalonFXConfiguration();

        configuration.CurrentLimits.withSupplyCurrentLimit(IntakeConstants.INTAKE_MOTOR_STALL_LIMIT);
        motor.getConfigurator().apply(configuration);
    }

    public void setMotor(double value) {
        motor.set(value);
    }

    public void setState(State state) {
        setMotor(state.speed);
    }

    public IntakeCommands getCommands() {
        return commands;
    }

}