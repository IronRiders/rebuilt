package org.ironriders.manipulation.launcher;

import org.ironriders.manipulation.launcher.LauncherConstants.State;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LauncherCommands {

    public final LauncherSubsystem launcher;
    

    LauncherCommands(LauncherSubsystem launcher) {
        this.launcher = launcher;

        launcher.publish("Set Launcher Ready State", set(State.READY));
        launcher.publish("Set Launcher Idle State", set(State.IDLE));
        launcher.publish("Set Launcher Stow State", set(State.STOW));

        launcher.publish("Set Launcher to Manual Angle", setAngleManually(launcher.getManualLauncherAngle()));
        launcher.publish("Set Launcher to Flywheel Velocity", setFlyWheelVelocityManually(launcher.getManualFlywheelVelocity()));
    }

    public Command setTarget(Pose3d target) {
        return Commands.runOnce(()->launcher.setTarget(target));
    }

    public Command set(State state) { // Will wait until we are ready
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

    public Command setAngleManually(double angle){
        return Commands.runOnce(()->launcher.setLauncherGoal(angle));
    }
    public Command setFlyWheelVelocityManually(double velocity){
        return Commands.runOnce(()->launcher.setFlywheelGoal(velocity));
    }
}
