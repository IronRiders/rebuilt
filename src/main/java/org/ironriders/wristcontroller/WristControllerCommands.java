package org.ironriders.wristcontroller;

import edu.wpi.first.wpilibj2.command.Command;

public class WristControllerCommands {
    private final WristControllerSubsystem wristControllerSubsystem;
    public WristControllerCommands(WristControllerSubsystem subsystem) {
        this.wristControllerSubsystem = subsystem;
    }

    public Command resetRotations() {
        return wristControllerSubsystem.runOnce(() -> {wristControllerSubsystem.resetRelativeEncoderRotations();});
    }
    public Command setGoal(WristControllerConstants.WristControllerPositions goal) {
        return wristControllerSubsystem.runOnce(() -> {wristControllerSubsystem.setGoal(goal.position);});
    }
}
