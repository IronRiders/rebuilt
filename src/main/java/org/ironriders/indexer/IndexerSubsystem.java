package org.ironriders.indexer;

import org.ironriders.indexer.IndexerConstants.State;
import org.ironriders.lib.IronSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class IndexerSubsystem extends IronSubsystem {
    private final IndexerCommands commands = new IndexerCommands(this);

    private TalonFX motor = new TalonFX(IndexerConstants.ID);
    private TalonFXConfiguration configuration;

    public IndexerSubsystem() {
        configuration = new TalonFXConfiguration();

        configuration.CurrentLimits.withSupplyCurrentLimit(IndexerConstants.STALL_LIMIT);
        motor.getConfigurator().apply(configuration);
    }

    public void setMotor(double value) {
        motor.set(value);
    }

    public void setState(State state) {
        setMotor(state.speed);
    }

    public IndexerCommands getCommands() {
        return commands;
    }
}
