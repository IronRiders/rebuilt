package org.ironriders.manipulation.wrist;

import java.time.DayOfWeek;

import org.ironriders.manipulation.wrist.WristConstants.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class WristCommands {
    private final WristSubsystem wristSubsystem;

    public WristCommands(WristSubsystem subsystem) {
        this.wristSubsystem = subsystem;
        this.wristSubsystem.publish("Reset Wrist Encoder", resetRotations());
    }

    public Command resetRotations() {
        return wristSubsystem.runOnce(() -> {
            wristSubsystem.resetRelativeEncoderRotations();
        });
    }

    public Command set(WristConstants.State goal) {
        return wristSubsystem.runOnce(() -> {
            wristSubsystem.setGoal(goal);
        });
    }

    public Command feed() {
        return Commands.sequence(
            set(State.DOWN).until(()->wristSubsystem.atGoal()),
            set(State.JOSTLE).until(()->wristSubsystem.atGoal())
        ).repeatedly().finallyDo(()->wristSubsystem.setGoal(State.DOWN));
    }
}
