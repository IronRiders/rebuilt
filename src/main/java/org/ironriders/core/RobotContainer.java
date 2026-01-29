// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ironriders.core;

import org.ironriders.climber.ClimberCommands;
import org.ironriders.climber.ClimberSubsystem;
import org.ironriders.drive.DriveCommands;
import org.ironriders.drive.DriveConstants;
import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.RobotUtils;
import org.ironriders.manipulation.indexer.IndexerCommands;
import org.ironriders.manipulation.indexer.IndexerSubsystem;
import org.ironriders.manipulation.intake.IntakeCommands;
import org.ironriders.manipulation.intake.IntakeSubsystem;
import org.ironriders.manipulation.shooter.ShooterCommands;
import org.ironriders.manipulation.shooter.ShooterSubsystem;
import org.ironriders.manipulation.wrist.WristCommands;
import org.ironriders.manipulation.wrist.WristSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

enum Config {
    PRIMARY_DRIVER, PRIMARY_DRIVER_WITH_BOOST, SECONDARY_DRIVER_WITH_BOOST;
}

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public final DriveCommands driveCommands = driveSubsystem.getCommands();

    public final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    public final IndexerCommands indexerCommands = indexerSubsystem.getCommands();

    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final IntakeCommands intakeCommands = intakeSubsystem.getCommands();

    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final ShooterCommands shooterCommands = shooterSubsystem.getCommands();

    public final WristSubsystem wristSubsystem = new WristSubsystem();
    public final WristCommands wristCommands = wristSubsystem.getCommands();

    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final ClimberCommands climberCommands = climberSubsystem.getCommands();

    public final Double triggerThreshold = 0.75;

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController primaryController = new CommandXboxController(
            DriveConstants.PRIMARY_CONTROLLER_PORT);
    private final CommandGenericHID secondaryController = new CommandJoystick(
            DriveConstants.KEYPAD_CONTROLLER_PORT);

    public final RobotCommands robotCommands = new RobotCommands(driveCommands, indexerCommands, intakeCommands,
            shooterCommands, wristCommands, climberCommands, primaryController.getHID());

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     *
     * <p>
     * builds the autos using
     * {@link com.pathplanner.lib.auto.AutoBuilder#buildAutoChooser()
     * buildAutoChooser()} posts the auto selection to
     * {@link SmartDashboard#putData(String, SendableChooser) SmartDashboard}
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
        DogLog.setPdh(new PowerDistribution());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Select", autoChooser);
    }

    /**
     * Th {@link CommandGenericHID#button(int)} method (such as
     * {@link CommandXboxController#button(int)},
     * {@link CommandJoystick#button(int)}, or one of the
     * {@link edu.wpi.first.wpilibj2.command.button.Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructors.
     */
    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // --- DRIVE CONTROLS ---
        driveSubsystem.setDefaultCommand(robotCommands.driveTeleop(
                () -> RobotUtils.controlCurve(primaryController.getLeftY()
                        * driveSubsystem.controlSpeedMultipler,
                        DriveConstants.TRANSLATION_CONTROL_EXPONENT,
                        DriveConstants.TRANSLATION_CONTROL_DEADBAND),
                () -> RobotUtils.controlCurve(primaryController.getLeftX()
                        * driveSubsystem.controlSpeedMultipler,
                        DriveConstants.TRANSLATION_CONTROL_EXPONENT,
                        DriveConstants.TRANSLATION_CONTROL_DEADBAND),
                () -> RobotUtils.controlCurve(primaryController.getRightX()
                        * driveSubsystem.controlSpeedMultipler,
                        DriveConstants.ROTATION_CONTROL_EXPONENT,
                        DriveConstants.ROTATION_CONTROL_DEADBAND)));

        // --- OTHER CONTROLS ---
        // TODO: Schedule a meeting to talk to drive about this once we have the design.
    }

    /**
     * Get command configured in auto chooser.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
