package org.ironriders.intake;
import org.ironriders.intake.IntakeSubsystem;

public class IntakeCommands {
    private final IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;
    }
    public Command set(IntakeConstants.IntakeState state) {
        return intake.runOnce(() -> intake.set(state));
        
    }
}
