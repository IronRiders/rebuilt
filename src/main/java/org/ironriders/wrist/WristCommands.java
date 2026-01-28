package org.ironriders.wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class WristCommands {
    private final WristSubsystem wristSubsystem;
    public WristCommands(WristSubsystem subsystem) {
        this.wristSubsystem = subsystem;
    }

    public Command resetRotations() {
        return wristSubsystem.runOnce(() -> {wristSubsystem.resetRelativeEncoderRotations();});
    }
    public Command setGoal(WristConstants.WristPositions goal) {
        return wristSubsystem.runOnce(() -> {wristSubsystem.setGoal(goal.position);});
    }
}
