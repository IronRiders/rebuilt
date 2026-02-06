package org.ironriders.manipulation.wrist;

import java.time.DayOfWeek;

import org.ironriders.manipulation.wrist.WristConstants.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Commands for the {@link WristSubsystem}.
 */
public class WristCommands {
    private final WristSubsystem wristSubsystem;

    public WristCommands(WristSubsystem subsystem) {
        this.wristSubsystem = subsystem;
        this.wristSubsystem.publish("Reset Wrist Encoder", resetRotations());
    }

    /**
     * Resets the wrist's relative encoder rotations to 0. See
     * {@link WristSubsystem#resetRelativeEncoderRotations()
     * resetRelativeEncoderRotations()} for more details.
     * 
     * @return A {@linkplain Command} to do the above.
     */
    public Command resetRotations() {
        return wristSubsystem.runOnce(() -> {
            wristSubsystem.resetRelativeEncoderRotations();
        });
    }

    /**
     * Sets the wrist's target to a given {@linkplain State state}. See
     * {@link WristSubsystem#setGoal(State) setGoal} for more details.
     * 
     * @param goal The target {@linkplain State} for the wrist.
     * @return A {@linkplain Command} to do the above.
     */
    public Command set(WristConstants.State goal) {
        return wristSubsystem.runOnce(() -> {
            wristSubsystem.setGoal(goal);
        });
    }

    /**
     * Jostles balls using the {@linkplain State.Jostle}.
     * 
     * @return A {@linkplain Command} to do the above.
     */
    public Command jostleBalls() {
        return Commands.sequence(
                set(State.DOWN).until(() -> wristSubsystem.atGoal()),
                set(State.JOSTLE).until(() -> wristSubsystem.atGoal())).repeatedly()
                .finallyDo(() -> wristSubsystem.setGoal(State.DOWN));
    }
}
