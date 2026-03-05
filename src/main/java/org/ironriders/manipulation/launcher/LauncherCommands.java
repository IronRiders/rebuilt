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

        launcher.publish("Set Launcher to Manual Extension 0.5", setExtensionManually(0.5));
        launcher.publish("Set Launcher to Manual Extension 0.0", setExtensionManually(0.1));
        launcher.publish("Set Launcher to Manual Extension 1.0", setExtensionManually(.9));
        // launcher.publish("Set Launcher Elastic", setExtensionFromElastic());


        launcher.publish("Set Launcher to Flywheel Velocity",
                setFlyWheelVelocityManually(launcher.getManualFlywheelVelocity()));
    }

    public Command readyAndFire() {
        return Commands.sequence(set(State.READY), runKicker());
    }

    public Command runKicker() {
        return Commands.runOnce(() -> launcher.runKicker());
    }

    /**
     * Sets the launcher's target state. Will wait until the launcher is ready
     * before finishing.
     * 
     * @param state The target state to set for the launcher.
     * @return A command that sets the launcher's target state.
     */
    public Command set(State state) { // Will wait until we are ready
        var cmd = new Command() {
            @Override
            public void initialize() {
                launcher.setCurrentState(state);
            }

            @Override
            public boolean isFinished() {
                return launcher.isReady();
            }
        };
        cmd.addRequirements(launcher);
        return cmd;
    }

    /**
     * Sets the launcher's extension to a given value. See
     * {@link LauncherSubsystem#setHoodExtension(double) LauncherSubsystem's
     * setLauncherGoal} for more details.
     * 
     * @param extension The target extension to set for the launcher.
     * @return A command that sets the launcher's extension.
     */
    public Command setExtensionManually(double amount) {
        return Commands.runOnce(() -> launcher.setServos(amount));
    }
    // public Command setExtensionFromElastic(){
    //     return Commands.runOnce(()-> launcher.setExtensionManually());
    // }

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
