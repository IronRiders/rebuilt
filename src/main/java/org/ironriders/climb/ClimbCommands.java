package org.ironriders.climb;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommands {

    public final ClimbSubsystem climb;

    /**
     * Publishes commands to set climb to various positions to {@linkplain
     * edu.wpi.first.wpilibj.smartdashboard.SmartDashboard SmartDashboard};
     *
     * <ul>
     * <li>Publish {@code "Climb"}; {@linkplain #set(ClimbConstants.Targets) set}
     * {@linkplain
     * ClimbConstants.Targets#CLIMBED Climbed}
     * <li>Publish {@code "Climb MAX"}; {@linkplain #set(ClimbConstants.Targets)
     * set} {@linkplain
     * ClimbConstants.Targets#MAX Max}
     * <li>Publish {@code "Climb MIN"}; {@linkplain #set(ClimbConstants.Targets)
     * set} {@linkplain
     * ClimbConstants.Targets#MIN Min}
     * <li>Publish {@code "Rehome"}; {@linkplain #home() home()}
     * </ul>
     */
    public ClimbCommands(ClimbSubsystem climb) {
        this.climb = climb;

        climb.debugPublish("Climb", set(ClimbConstants.Targets.CLIMBED));
        climb.debugPublish("Climb MAX", set(ClimbConstants.Targets.MAX));
        climb.debugPublish("Climb MIN", set(ClimbConstants.Targets.MIN));
        climb.debugPublish("Rehome", home());
    }

    /**
     * Passes to {@link ClimbSubsystem#setGoal(ClimbConstants.Targets)
     * climb.setGoal()}.
     *
     * @param target the target to set the climb to (using
     *               {@link ClimbConstants.Targets
     *               ClimbConstants})
     * @return a command that sets the goal to {@code ClimbConstants.Targets target}
     */
    public Command set(ClimbConstants.Targets target) {
        return climb.runOnce(() -> climb.setGoal(target));
    }

    /**
     * Passes to {@link ClimbSubsystem#home() climb.home()}.
     *
     * @return a command that homes the climber
     */
    public Command home() {
        return climb.runOnce(() -> climb.home());
    }
}
