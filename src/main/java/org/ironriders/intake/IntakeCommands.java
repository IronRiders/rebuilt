package org.ironriders.intake;

import static org.ironriders.intake.IntakeConstants.DISCHARGE_TIMEOUT;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironriders.intake.IntakeConstants.IntakeState;

public class IntakeCommands {

    private final IntakeSubsystem intake;

    @SuppressWarnings("unused")
    private Runnable onSuccess;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;

        intake.debugPublish("Intake Grab", set(IntakeState.GRAB));
        intake.debugPublish("Intake Score", set(IntakeState.SCORE));
        intake.debugPublish("Intake Eject", set(IntakeState.EJECT));
        intake.debugPublish("Intake Stop", set(IntakeState.STOP));
    }

    public Command set(IntakeConstants.IntakeState state) {
        Command command = intake.runOnce(() -> intake.set(state));

        switch (state) {
            case GRAB:
                return command;
            case EJECT:
                return command.withTimeout(DISCHARGE_TIMEOUT).finallyDo(() -> intake.set(IntakeState.STOP));
            case SCORE:
                return command;
            case STOP:
                command.cancel(); // fall through
            default:
                return command.finallyDo(() -> intake.set(IntakeState.STOP));
        }
    }

    public Command boost() {
        return Commands.sequence(
                Commands.runOnce(() -> intake.setMotorsNoDiff(IntakeState.BOOST.speed)),
                Commands.waitSeconds(IntakeConstants.BOOST_TIME),
                Commands.runOnce(() -> intake.setMotorsNoDiff(IntakeState.STOP.speed)));
    }

    public Command unboost() {
        return Commands.sequence(
                Commands.runOnce(() -> intake.setMotorsNoDiff(-IntakeState.BOOST.speed)),
                Commands.waitSeconds(IntakeConstants.UNBOOST_TIME),
                Commands.runOnce(() -> intake.setMotorsNoDiff(IntakeState.STOP.speed)));
    }

    public Command reset() {
        return intake.runOnce(() -> intake.set(IntakeState.STOP));
    }

    public IntakeSubsystem getIntake() {
        return intake;
    }

    public void setOnSuccess(Runnable onSucess) {
        this.onSuccess = onSucess;
    }
}
