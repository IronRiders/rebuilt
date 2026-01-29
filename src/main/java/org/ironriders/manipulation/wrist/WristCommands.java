package org.ironriders.manipulation.wrist;

import edu.wpi.first.wpilibj2.command.Command;

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

    public Command setGoal(WristConstants.State goal) {
        return wristSubsystem.runOnce(() -> {
            wristSubsystem.setGoal(goal.position);
        });
    }
}
