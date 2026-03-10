package org.ironriders.core;

import java.util.function.DoubleSupplier;

import org.ironriders.climber.ClimberCommands;
import org.ironriders.drive.DriveCommands;
import org.ironriders.manipulation.indexer.IndexerCommands;
import org.ironriders.manipulation.indexer.IndexerConstants;
import org.ironriders.manipulation.intake.IntakeCommands;
import org.ironriders.manipulation.intake.IntakeConstants;
import org.ironriders.manipulation.launcher.LauncherCommands;
import org.ironriders.manipulation.launcher.LauncherConstants;
import org.ironriders.manipulation.launcher.LauncherConstants.State;
import org.ironriders.manipulation.wrist.WristCommands;
import org.ironriders.manipulation.wrist.WristConstants;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * These commands require more complex logic and are not directly tied to a
 * subsystem. They generally interface with multiple subsystems via their
 * commands and are higher-level.
 */

public class RobotCommands {
    private final DriveCommands driveCommands;
    private final IndexerCommands indexerCommands;
    private final IntakeCommands intakeCommands;
    private final LauncherCommands launcherCommands;
    private final WristCommands wristCommands;
    private final ClimberCommands climberCommands;

    @SuppressWarnings("unused")
    private final GenericHID controller;

    public RobotCommands(DriveCommands driveCommands, IndexerCommands indexerCommands, IntakeCommands intakeCommands,
            LauncherCommands launcherCommands, WristCommands wristCommands, ClimberCommands climberCommands,
            GenericHID controller) {
        this.driveCommands = driveCommands;
        this.indexerCommands = indexerCommands;
        this.intakeCommands = intakeCommands;
        this.launcherCommands = launcherCommands;
        this.wristCommands = wristCommands;
        this.climberCommands = climberCommands;

        this.controller = controller;
        NamedCommands.registerCommand("Shooter fire", fire());
    }

    /**
     * Command to drive the robot given controller input.
     *
     * @param inputTranslationX DoubleSupplier, value from 0-1.
     * @param inputTranslationY DoubleSupplier, value from 0-1.
     * @param inputRotation     DoubleSupplier, value from 0-1.
     */
    public Command driveTeleop(DoubleSupplier inputTranslationX, DoubleSupplier inputTranslationY,
            DoubleSupplier inputRotation) {
        return driveCommands.driveTeleop(inputTranslationX, inputTranslationY, inputRotation, true);
    }

    public Command fire() {
        return Commands.sequence(
                launcherCommands.readyAndFire(),
                Commands.parallel(
                        indexerCommands.set(IndexerConstants.State.INDEX),
                        wristCommands.jostleBalls())
                        // intakeCommands.intake());//TODO re ad
        );
    }

    public Command stopFire() {
        return Commands.sequence(
                launcherCommands.stopKicker(),
                Commands.parallel(
                        launcherCommands.set(State.STOW),
                        indexerCommands.set(IndexerConstants.State.STOP),
                        wristCommands.set(WristConstants.State.DOWN)),
                        intakeCommands.set(IntakeConstants.State.STOP));
    }

    public Command intake() {
        return Commands.parallel(intakeCommands.set(IntakeConstants.State.INTAKE),
                wristCommands.set(WristConstants.State.DOWN));
    }

    public Command stow() { // Reset everything
        return Commands.parallel(launcherCommands.set(LauncherConstants.State.STOW),
                Commands.runOnce(() -> launcherCommands.launcher.stopKicker()),
                indexerCommands.set(IndexerConstants.State.STOP), intakeCommands.set(IntakeConstants.State.STOP),
                wristCommands.set(WristConstants.State.UP), climberCommands.home());
    }
}