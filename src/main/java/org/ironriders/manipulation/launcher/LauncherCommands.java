package org.ironriders.manipulation.launcher;

import org.ironriders.manipulation.launcher.LauncherConstants.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Commands for {@link LauncherSubsystem} */
public class LauncherCommands {

    public final LauncherSubsystem launcher;

    LauncherCommands(LauncherSubsystem launcher) {
        this.launcher = launcher;

        launcher.publish("Set Launcher Ready State", set(State.READY));
        launcher.publish("Set Launcher Idle State", set(State.IDLE));
        launcher.publish("Set Launcher Stow State", set(State.STOW));

        launcher.publish("Set Launcher to Manual Angle", setAngleManually(launcher.getManualLauncherAngle()));
        launcher.publish("Set Launcher to Flywheel Velocity",
                setFlyWheelVelocityManually(launcher.getManualFlywheelVelocity()));
    }

    public Command readyAndFire() {
        return Commands.sequence(set(State.READY), fire());
    }

    public Command fire() {
        return Commands.runOnce(() -> LauncherSubsystem.fire());
    }

    /**
     * Sets the launcher's target state. Will wait until the launcher is ready
     * before finishing.
     * 
     * @param state The target state to set for the launcher.
     * @return A command that sets the launcher's target state.
     */
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

    /**
     * Sets the launcher's angle to a given value. See
     * {@link LauncherSubsystem#setHoodAngleGoal(double) LauncherSubsystem's
     * setLauncherGoal} for more details.
     * 
     * @param angle The target angle to set for the launcher.
     * @return A command that sets the launcher's angle.
     */
    public Command setAngleManually(double angle) {
        return Commands.runOnce(() -> launcher.setHoodAngleGoal(angle));
    }

    /**
     * Sets the launcher's flywheel velocity to a given value. See
     * {@link LauncherSubsystem#setFlywheelGoal(double) setFlywheelGoal} for
     * more details.
     * 
     * @param velocity The target flywheel velocity to set for the launcher.
     * @return A command that sets the launcher's flywheel velocity.
     */
    public Command setFlyWheelVelocityManually(double velocity) {
        return Commands.runOnce(() -> launcher.setFlywheelGoal(velocity));
    }
}
