package org.ironriders.climber;

import org.ironriders.climber.ClimberConstants.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimberCommands {
    private ClimberSubsystem climber;

    public ClimberCommands(ClimberSubsystem climber) {
        this.climber = climber;
    }

    public Command set(State state) {
        return Commands.runOnce(() -> climber.setGoal(state));
    }
}
