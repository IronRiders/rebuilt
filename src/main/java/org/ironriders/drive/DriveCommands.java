package org.ironriders.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveCommands {
        private final DriveSubsystem driveSubsystem;

        public DriveCommands(DriveSubsystem driveSubsystem) {
                this.driveSubsystem = driveSubsystem;
        }

        /**
         * Command to drive the robot given a supplier.
         * 
         * @param translation   A {@link Supplier} of {@link Translation2d}, the
         *                      translation to drive the robot in.
         * @param rotation      A {@link DoubleSupplier} for the rotation value.
         * @param fieldRelative A {@link BooleanSupplier} indicating if the drive is
         *                      field-relative.
         * @return A {@link Command} to drive the robot.
         */
        public Command drive(
                        Supplier<Translation2d> translation, DoubleSupplier rotation, BooleanSupplier fieldRelative) {
                return Commands.run(
                                () -> {
                                        DriveSubsystem.drive(
                                                        translation.get(), rotation.getAsDouble(),
                                                        fieldRelative.getAsBoolean());
                                },
                                driveSubsystem);
        }

        /**
         * Drive the robot given controller input. Corrects for alliance. Calls
         * {@link #drive}, converts inputTranslationX & inputTranslationX ->
         * {@link Translation2d}.
         * 
         * @param inputTranslationX DoubleSupplier, value from 0-1.
         * @param inputTranslationY DoubleSupplier, value from 0-1.
         * @param inputRotation     DoubleSupplier, value from 0-1.
         * @param fieldRelative     Whether the driving is field-relative.
         * @return A command to drive the robot.
         */
        public Command driveTeleop(
                        DoubleSupplier inputTranslationX,
                        DoubleSupplier inputTranslationY,
                        DoubleSupplier inputRotation,
                        boolean fieldRelative) {

                double invert = DriverStation.getAlliance().isEmpty()
                                || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                                                ? 1
                                                : -1;

                return drive(
                                () -> new Translation2d(inputTranslationX.getAsDouble(),
                                                inputTranslationY.getAsDouble())
                                                .times(DriveConstants.SWERVE_MAX_TRANSLATION_TELEOP * invert),
                                () -> inputRotation.getAsDouble() * DriveConstants.SWERVE_MAX_ANGULAR_TELEOP * invert,
                                () -> fieldRelative);
        }
}