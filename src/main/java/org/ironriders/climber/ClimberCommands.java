package org.ironriders.climber;

import org.ironriders.climber.ClimberConstants.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** {@link Command Commands} for the Climber subsystem. */
public class ClimberCommands {
    private ClimberSubsystem climber;

    public ClimberCommands(ClimberSubsystem climber) {
        this.climber = climber;
    }

    /**
     * Sets the climber's target to a given {@link State state}. See
     * {@link ClimberSubsystem#setGoal(State) setGoal} for more details.
     * 
     * @param state The target {@link State state} for the climber.
     * @return A {@link Command} that sets the climber's target state.
     */
    public Command set(State state) {
        return Commands.runOnce(() -> climber.setGoal(state));
    }
}
