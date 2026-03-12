package org.ironriders.manipulation.intake;

import org.ironriders.manipulation.intake.IntakeConstants.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Public-facing methods which return commands for the Intake */
public class IntakeCommands {                       

    private IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;

        intake.publish("Intake", this.intake());
        intake.publish("Intake force", this.set(State.INTAKE));
        intake.publish("Stop Intake",this.set(State.STOP));
        intake.publish("Eject force", this.set(State.BACK));
        intake.publish("Eject", this.eject());
    }

    /**
     * Sets the intake's motor to a target {@link State state}.
     * 
     * @param state The intake's target {@link State state}
     * @return A command to set the intake's target State
     */
    public Command set(State state) {
        return Commands.runOnce(() -> intake.setState(state));
    }

    /**
     * Ejects balls by setting the Intake's target state to {@link State#BACK BACK}
     * 
     * @return A {@link Command} which does the above
     */
    public Command eject() {
        return set(State.BACK);
    }

    /**
     * Intakes balls by setting the Intake's target state to {@link State#INTAKE
     * INTAKE}
     * 
     * @return A {@link Command} which does the above
     */
    public Command intake() {
        return set(State.INTAKE);
    }
}