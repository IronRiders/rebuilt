package org.ironriders.manipulation.intake;

import org.ironriders.manipulation.intake.IntakeConstants.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {

    private IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;

        intake.publish("Intake", this.intake());
        intake.publish("Intake force", this.set(State.INTAKE));
        intake.publish("Stop", this.set(State.STOP));
        intake.publish("Eject force", this.set(State.BACK));
        intake.publish("Eject", this.eject());
    }

    public Command set(State state) {
        return Commands.runOnce(() -> intake.setState(state));
    }

    public Command eject() {
        return setAndWait(State.BACK);
    }

    public Command intake() {
        return setAndWait(State.INTAKE);
    }

    private Command setAndWait(State start) {
        return Commands.runOnce(() -> intake.setState(start));
    }
}