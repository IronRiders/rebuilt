package org.ironriders.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironriders.core.ElevatorWristCTL.WristRotation;

public class WristCommands {

    private WristSubsystem wristSubsystem;

    public WristCommands(WristSubsystem wrist) {
        this.wristSubsystem = wrist;
    }

    public Command set(WristRotation rotation) {
        return new Command() {
            public void initialize() {
                wristSubsystem.setGoal(rotation);
            }

            public boolean isFinished() {
                return wristSubsystem.atGoal();
            }
        };
    }

    public Command reset() {
        return wristSubsystem.runOnce(wristSubsystem::reset);
    }

    public Command stowReset() {
        return Commands.sequence(reset(), set(WristRotation.HOLD));
    }
}
