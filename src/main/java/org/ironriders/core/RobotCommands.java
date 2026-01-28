package org.ironriders.core;

import java.util.function.DoubleSupplier;

import org.ironriders.climber.ClimberCommands;
import org.ironriders.drive.DriveCommands;
import org.ironriders.manipulation.indexer.IndexerCommands;
import org.ironriders.manipulation.intake.IntakeCommands;
import org.ironriders.manipulation.shooter.ShooterCommands;
import org.ironriders.manipulation.wrist.WristCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * These commands require more complex logic and are not directly tied to a
 * subsystem. They
 * generally interface w/ multiple subsystems via their commands and are
 * higher-level.
 */

public class RobotCommands {
    private final DriveCommands driveCommands;
    private final IndexerCommands indexerCommands;
    private final IntakeCommands intakeCommands;
    private final ShooterCommands shooterCommands;
    private final WristCommands wristCommands;
    private final ClimberCommands climberCommands;

    private final GenericHID controller;

    public RobotCommands(DriveCommands driveCommands, IndexerCommands indexerCommands, IntakeCommands intakeCommands,
            ShooterCommands shooterCommands, WristCommands wristCommands, ClimberCommands climberCommands,
            GenericHID controller) {
        this.driveCommands = driveCommands;
        this.indexerCommands = indexerCommands;
        this.intakeCommands = intakeCommands;
        this.shooterCommands = shooterCommands;
        this.wristCommands = wristCommands;
        this.climberCommands = climberCommands;

        this.controller = controller;
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
}
