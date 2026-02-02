package org.ironriders.manipulation.launcher;

import org.ironriders.manipulation.launcher.LauncherConstants.State;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LauncherCommands {

    public final LauncherSubsystem launcher;

    LauncherCommands(LauncherSubsystem launcher) {
        this.launcher = launcher;
    }

    public Command setTarget(Pose3d target) {
        return Commands.runOnce(()->launcher.setTarget(target));
    }

    public Command set(State state) { // Will wait until we are ready
        if (state == State.READY && !LauncherSubsystem.inRange()) {
            return new Command() {};
        }

        return new Command() {
            @Override
            public void initialize() {
                launcher.setCurrentState(state);
            }

            @Override
            public boolean isFinished() {
                return launcher.isReady();
            }
        };

    }
}
