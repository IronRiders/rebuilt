package org.ironriders.manipulation.indexer;

import org.ironriders.lib.IronSubsystem;
import org.ironriders.manipulation.indexer.IndexerConstants.State;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/** Subsystem for indexer (controls ball movement inside robot) */
public class IndexerSubsystem extends IronSubsystem {
    private final IndexerCommands commands = new IndexerCommands(this);

    private TalonFX motor = new TalonFX(IndexerConstants.ID);
    
    private TalonFXConfiguration configuration;

    public IndexerSubsystem() {
        configuration = new TalonFXConfiguration();

        configuration.CurrentLimits.withSupplyCurrentLimit(IndexerConstants.STALL_LIMIT);
        motor.getConfigurator().apply(configuration);
    }

    /**
     * Sets the indexer motor to a value.
     * 
     * @param value The speed to
     *              {@link com.ctre.phoenix6.hardware.TalonFX#set(double speed)
     *              set} the motor to (between -1 & 1)
     */
    public void setMotor(double value) {
        motor.set(value);
    }

    /**
     * Sets the indexer's motor to a target state; see
     * {@link IndexerCommands#set(State) IndexerCommands.set} for more details.
     * 
     * @param state The indexer's target
     *              {@link org.ironriders.manipulation.indexer.IndexerConstants.State
     *              state}
     */
    public void setState(State state) {
        setMotor(state.speed);
    }

    /**
     * @return The IndexerCommands object for the indexer subsystem
     */
    public IndexerCommands getCommands() {
        return commands;
    }
}
