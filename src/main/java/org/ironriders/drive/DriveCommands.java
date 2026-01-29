package org.ironriders.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironriders.drive.DriveConstants.Controller;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveCommands {
    private final DriveSubsystem driveSubsystem;

    public DriveCommands(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    public Command setController(Controller controller) {
        return Commands.runOnce(() -> DriveSubsystem.setController(controller));
    }

    public Command drive(
            Supplier<Translation2d> translation, DoubleSupplier rotation, BooleanSupplier fieldRelative) {
        return Commands.run(
                () -> {
                    DriveSubsystem.requestDriveMovement(
                            Controller.DRIVER, translation.get(), rotation.getAsDouble(),
                            fieldRelative.getAsBoolean());
                },
                driveSubsystem);
    }

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