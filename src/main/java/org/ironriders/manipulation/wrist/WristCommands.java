package org.ironriders.manipulation.wrist;

import org.ironriders.manipulation.wrist.WristConstants.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Commands for the {@link WristSubsystem}.
 */
public class WristCommands {
    private final WristSubsystem wristSubsystem;

    public WristCommands(WristSubsystem subsystem) {
        this.wristSubsystem = subsystem;
    }

    /**
     * Sets the wrist's target to a given {@link State state}. See
     * {@link WristSubsystem#setGoal(State) setGoal} for more details.
     * 
     * @param goal The target {@link State} for the wrist.
     * @return A {@link Command} to do the above.
     */
    public Command set(WristConstants.State goal) {
        return wristSubsystem.runOnce(() -> {
            wristSubsystem.setGoal(goal);
        });
    }

    public Command setDown() {
        return set(State.DOWN);
    }

    public Command setUp() {
        return set(State.UP);
    }

    /**
     * Jostles balls using the {@link State.Jostle}.
     * 
     * @return A {@link Command} to do the above.
     */
    public Command jostleBalls() {
        return Commands.sequence(set(State.JOSTLE).finallyDo(() -> wristSubsystem.setGoal(State.DOWN)));
    }
}
