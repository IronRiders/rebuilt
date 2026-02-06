package org.ironriders.manipulation.launcher;

import org.ironriders.core.RobotContainer;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.manipulation.launcher.LauncherConstants.State;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Commands for {@linkplain LauncherSubsystem} */
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

    /**
     * Sets the launcher's target to a given {@link Pose3d target position}. See
     * {@link LauncherSubsystem#setTarget(Pose3d) LauncherSubsystem's setTarget} for
     * more details.
     * 
     * @param target The target position for the launcher.
     * @return A command that sets the launcher's target.
     */
    public Command setTarget(Pose3d target) {
        return Commands.runOnce(() -> launcher.setTarget(target));
    }

    /**
     * Sets the launcher's target to the closest point in the passing zone. See
     * {@link LauncherSubsystem#setTarget(Pose3d) setTarget} &
     * {@linkplain RobotContainer.passingZone#closestPointAsPose3d()
     * RobotContainer's passingZone.closestPointAsPose3d} for more details.
     * 
     * @return A command that sets the launcher's target to the closest point in the
     *         passing zone.
     */
    public Command targetPassing() {
        return setTarget(RobotContainer.passingZone.closestPointAsPose3d());
    }

    /**
     * Sets the launcher's target to the hub. See
     * {@link LauncherSubsystem#setTarget(Pose3d) setTarget} &
     * {@linkplain FieldPositions#get(ElementType) FieldPositions' get} for more
     * details.
     * 
     * @return A command that sets the launcher's target to the hub.
     */
    public Command targetHub() {
        return setTarget(FieldPositions.get(ElementType.HUB));
    }

    /**
     * Sets the launcher's target state. Will wait until the launcher is ready
     * before signaling {@code task complete} and self-destructing.
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
     * {@linkplain LauncherSubsystem#setLauncherGoal(double) LauncherSubsystem's
     * setLauncherGoal} for more details.
     * 
     * @param angle The target angle to set for the launcher.
     * @return A command that sets the launcher's angle.
     */
    public Command setAngleManually(double angle) {
        return Commands.runOnce(() -> launcher.setLauncherGoal(angle));
    }

    /**
     * Sets the launcher's flywheel velocity to a given value. See
     * {@linkplain LauncherSubsystem#setFlywheelGoal(double) setFlywheelGoal} for
     * more details.
     * 
     * @param velocity The target flywheel velocity to set for the launcher.
     * @return A command that sets the launcher's flywheel velocity.
     */
    public Command setFlyWheelVelocityManually(double velocity) {
        return Commands.runOnce(() -> launcher.setFlywheelGoal(velocity));
    }
}
