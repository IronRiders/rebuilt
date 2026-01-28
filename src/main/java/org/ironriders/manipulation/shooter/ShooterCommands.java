package org.ironriders.manipulation.shooter;

import org.ironriders.manipulation.shooter.ShooterConstants.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterCommands {
    
    public final ShooterSubsystem shooter;

    ShooterCommands(ShooterSubsystem shooter){
        this.shooter = shooter;
    }

    public Command set(State state) {
        return Commands.runOnce(()->shooter.setCurrentState(state));
    }
}
