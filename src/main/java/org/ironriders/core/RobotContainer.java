// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ironriders.core;

import org.ironriders.climber.ClimberCommands;
import org.ironriders.climber.ClimberSubsystem;
import org.ironriders.drive.DriveCommands;
import org.ironriders.drive.DriveConstants;
import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.lib.field.Zone;
import org.ironriders.lib.field.Zone.ZoneType;
import org.ironriders.manipulation.indexer.IndexerCommands;
import org.ironriders.manipulation.indexer.IndexerSubsystem;
import org.ironriders.manipulation.intake.IntakeCommands;
import org.ironriders.manipulation.intake.IntakeSubsystem;
import org.ironriders.manipulation.launcher.LauncherCommands;
import org.ironriders.manipulation.launcher.LauncherSubsystem;
import org.ironriders.manipulation.wrist.WristCommands;
import org.ironriders.manipulation.wrist.WristSubsystem;
import org.ironriders.vision.VisionCommands;
import org.ironriders.vision.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
    public final LauncherCommands launcherCommands = launcherSubsystem.getCommands();

    public final WristSubsystem wristSubsystem = new WristSubsystem();
    public final WristCommands wristCommands = wristSubsystem.getCommands();

    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final ClimberCommands climberCommands = climberSubsystem.getCommands();

    public final VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final VisionCommands visionCommands = visionSubsystem.getCommands();

    public final Double triggerThreshold = 0.75;

    public final Zone passingZone = new Zone(FieldPositions.Zones.get(ZoneType.PASSING), ZoneType.PASSING); // TODO: Messy to have to
    // specify the zone
    public final Zone scoringZone = new Zone(FieldPositions.Zones.get(ZoneType.SCORING), ZoneType.SCORING);

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController primaryController = new CommandXboxController(
            DriveConstants.CONTROLLER_PRIMARY_PORT);
    private final CommandGenericHID secondaryController = new CommandJoystick(
            DriveConstants.CONTROLLER_SECONDARY_PORT);

    public final RobotCommands robotCommands = new RobotCommands(driveCommands, indexerCommands, intakeCommands,
            launcherCommands, wristCommands, climberCommands, visionCommands, primaryController.getHID());

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

        DogLog.log("Distance test", String.valueOf(passingZone.distanceTo().getNorm()));
    }

    public void periodic() {
        if (scoringZone.inside()) { // Auto-target
            launcherCommands.setTarget(FieldPositions.preparePose(FieldPositions.Hub.HUB_TOP));
        } else if (passingZone.inside()) {
            launcherCommands.setTarget(Utils.expandPose2d(passingZone.closestPoint()));
        }
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
        driveSubsystem.setDefaultCommand(Commands.parallel(robotCommands.driveTeleop(
                () -> Utils.controlCurve(primaryController.getLeftY(),
                        DriveConstants.TRANSLATION_CONTROL_EXPONENT,
                        DriveConstants.TRANSLATION_CONTROL_DEADBAND),
                () -> Utils.controlCurve(primaryController.getLeftX(),
                        DriveConstants.TRANSLATION_CONTROL_EXPONENT,
                        DriveConstants.TRANSLATION_CONTROL_DEADBAND),
                () -> Utils.controlCurve(primaryController.getRightX(),
                        DriveConstants.ROTATION_CONTROL_EXPONENT,
                        DriveConstants.ROTATION_CONTROL_DEADBAND)),
                Commands.run(() -> periodic())));

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
