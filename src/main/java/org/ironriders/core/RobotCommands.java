package org.ironriders.core;

import java.util.function.DoubleSupplier;

import org.ironriders.drive.DriveCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * These commands require more complex logic and are not directly tied to a subsystem. They
 * generally interface w/ multiple subsystems via their commands and are higher-level.
 */
public class RobotCommands {
    private final DriveCommands driveCommands;

    private final GenericHID controller;

    /**
     * Creates final variables for all command classes.
     *
     * @param driveCommands DriveCommands instance
     * @param targetingCommands TargetingCommands instance
     * @param climbCommands ClimbCommands instance
     * @param controller GenericHID controller (joystick/gamepad) instance
     */
    public RobotCommands(DriveCommands driveCommands, GenericHID controller) {
        this.driveCommands = driveCommands;
        this.controller = controller;
    }

    /**
     * Command to drive the robot given controller input.
     *
     * @param inputTranslationX DoubleSupplier, value from 0-1.
     * @param inputTranslationY DoubleSupplier, value from 0-1.
     * @param inputRotation DoubleSupplier, value from 0-1.
     */
    public Command driveTeleop(DoubleSupplier inputTranslationX, DoubleSupplier inputTranslationY,
            DoubleSupplier inputRotation) {
        return driveCommands.driveTeleop(inputTranslationX, inputTranslationY, inputRotation, true);
    }
}
