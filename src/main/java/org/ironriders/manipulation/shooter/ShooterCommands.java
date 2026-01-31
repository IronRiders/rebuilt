package org.ironriders.manipulation.shooter;

import org.ironriders.manipulation.shooter.ShooterConstants.State;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommands {

    public final ShooterSubsystem shooter;

    ShooterCommands(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    public Command set(State state) { // Will wait until we are ready
        if (state == State.READY && !ShooterSubsystem.inRange()) {
            return new Command() {};
        }

        return new Command() {
            @Override
            public void initialize() {
                shooter.setCurrentState(state);
            }

            @Override
            public boolean isFinished() {
                return shooter.isReady();
            }
        };

    }
}
