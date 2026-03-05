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

    /**
     * Sets the intake's motor to a value.
     * 
     * @param value The speed to
     *              {@link com.ctre.phoenix6.hardware.TalonFX#set(double speed)
     *              set} the motor to (between -1 & 1)
     */
    public void setMotor(double value) {
        motor.set(value);
    }

    /**
     * Sets the intake's motor to a target state; see
     * {@link IntakeCommands#set(State) IntakeCommands.set} for more details.
     * 
     * @param state The intake's target
     *              {@link org.ironriders.manipulation.intake.IntakeConstants.State
     *              state}
     */
    public void setState(State state) {
        setMotor(state.speed);
    }

    @Override
    public void periodic() {
        publish("intakeOutput", motor.getMotorVoltage().getValueAsDouble());
    }

    /**
     * @return The IntakeCommands object for the intake subsystem
     */
    public IntakeCommands getCommands() {
        return commands;
    }

}