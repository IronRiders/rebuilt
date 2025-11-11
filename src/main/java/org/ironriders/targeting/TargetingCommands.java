package org.ironriders.targeting;

import edu.wpi.first.wpilibj2.command.Command;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPose.Side;

public class TargetingCommands {

    private TargetingSubsystem targetingSubsystem;

    public TargetingCommands(TargetingSubsystem targetingSubsystem) {
        this.targetingSubsystem = targetingSubsystem;

        this.targetingSubsystem.debugPublish("Nearest", targetNearest());

        this.targetingSubsystem.debugPublish("Coral Station", targetNearest(ElementType.STATION));

        this.targetingSubsystem.debugPublish("Reef", targetNearest(ElementType.REEF));
        this.targetingSubsystem.debugPublish("Reef Left Pole", targetReefPole(Side.Left));
        this.targetingSubsystem.debugPublish("Reef Right Pole", targetReefPole(Side.Right));

        this.targetingSubsystem.debugPublish("Processor", targetNearest(ElementType.PROCESSOR));
    }

    public Command targetStationSlot(int number) {
        return targetingSubsystem
                .runOnce(
                        () -> {
                            targetingSubsystem.setTargetSlot(number);
                            targetingSubsystem.targetNearest(ElementType.STATION);
                        })
                .ignoringDisable(true);
    }

    public Command targetReefPole(Side side) {
        return targetingSubsystem
                .runOnce(
                        () -> {
                            targetingSubsystem.setTargetPole(side);
                            targetingSubsystem.targetNearest(ElementType.REEF);
                        })
                .ignoringDisable(true);
    }

    public Command targetNearest() {
        return targetingSubsystem.runOnce(targetingSubsystem::targetNearest).ignoringDisable(true);
    }

    public Command targetNearest(ElementType type) {
        return targetingSubsystem
                .runOnce(() -> targetingSubsystem.targetNearest(type))
                .ignoringDisable(true);
    }
}
