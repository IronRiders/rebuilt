package org.ironriders.manipulation.wrist;

import org.ironriders.manipulation.wrist.WristConstants.State;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Commands for the {@link WristSubsystem}.
 */
public class WristCommands {
    private final WristSubsystem wristSubsystem;

    public WristCommands(WristSubsystem subsystem) {
        this.wristSubsystem = subsystem;

        subsystem.publish("Wrist Down", setDown());
        subsystem.publish("Wrist Up", setUp());

        subsystem.publish("Jostle", jostleBalls());

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
        return set(State.JOSTLE);
    }

    public Command home() {
        return wristSubsystem.runOnce(() -> {
            try {
                wristSubsystem.home();
            } catch (InterruptedException e) {
                this.wristSubsystem.notifyError(
                        this.wristSubsystem.buildNotification(
                                e.getClass().descriptorString() + e.getCause().getMessage(),
                                e.getStackTrace().toString()));
            }
        });
    }
}
