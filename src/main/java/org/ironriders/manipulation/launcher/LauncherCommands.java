package org.ironriders.manipulation.launcher;

import static edu.wpi.first.units.Units.Rotations;

import org.ironriders.manipulation.launcher.LauncherConstants.FlyWheelState;
import org.ironriders.manipulation.launcher.LauncherConstants.State;

import edu.wpi.first.units.measure.Angle;
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
        launcher.publish("Set Launcher Manual State", set(State.MANUAL));

        launcher.publish("Set Min Angle 0", setAngleManually(Angle.ofRelativeUnits(0, Rotations)));
        launcher.publish("Set Min Angle .2", setAngleManually(Angle.ofRelativeUnits(0, Rotations)));
        // launcher.publish("Set Launcher Elastic", setExtensionFromElastic());

        launcher.publish("Set Launcher to Flywheel Velocity",
                setFlyWheelVelocityManually(launcher.getManualFlywheelVelocity()));

        launcher.publish("Close Hub Velocity 27.5", setCustomFlyWheelSpeed(FlyWheelState.HUB.speed));
        launcher.publish("Midrange Velocity 33.5", setCustomFlyWheelSpeed(FlyWheelState.CENTER.speed));
        launcher.publish("Tower Velocity 36", setCustomFlyWheelSpeed(FlyWheelState.TOWER.speed));
        launcher.publish("Trench Velocity 36.2", setCustomFlyWheelSpeed(FlyWheelState.TRENCH.speed));
        launcher.publish("Corner Velocity 49.5", setCustomFlyWheelSpeed(FlyWheelState.CORNER.speed));


        launcher.publish("kicker on", runKicker());
        launcher.publish("kicker off", stopKicker());
    }

    public Command readyAndFire() {
        return Commands.sequence(set(State.MANUAL), runKicker());
    }

    public Command runKicker() {
        return Commands.runOnce(() -> launcher.runKicker());
    }

    public Command stopKicker() {
        return Commands.runOnce(() -> launcher.stopKicker());
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
     * @param angle The target angle in rotations to set for the launcher's hood .
     * @return A command that sets the launcher's hood angle.
     */
    public Command setAngleManually(Angle angle) {
        return Commands.runOnce(() -> launcher.setHoodAngle(angle));
    }
    // public Command setExtensionFromElastic(){
    // return Commands.runOnce(()-> launcher.setExtensionManually());
    // }

    /**
     * Sets the launcher's internal variable for the angle setpoint when it is the manual targeting mode to a given value. See
     * {@link LauncherSubsystem#setManualLauncherAngle(angle) LauncherSubsystem's
     * setLauncherGoal} for more details and also check the LauncherSubsystem's peroidc for the exact impmentation of the manual Mode.
     * 
     * @param extension The target extension to set for the launcher.
     * @return A command that sets the launcher's extension.
     * THIS DOES NOT SET THIS IS THE GOAL AND ONLY SETS THE INTERNAL TARGET WHEN THE ROBOT IS IN MANUAL LAUNCHER MODE
     */

    public Command setCustomFlyWheelSpeed(double flywheelSpeed) {
        return Commands.runOnce(() -> launcher.setManualFlyWheelSpeed(flywheelSpeed));
    }

    /**
     * Sets the launcher's flywheel velocity to a given value. See
     * {@link LauncherSubsystem#setFlywheelGoal(double) setFlywheelGoal} for
     * more details.
     * 
     * @param velocity The target flywheel velocity to set for the launcher.
     * @return A command that sets the launcher's flywheel velocity.
     */
    public Command setFlyWheelVeocityInternalTarget(double velocity) {
        return Commands.runOnce(() -> launcher.setFlywheelGoal(velocity));
    }
}
