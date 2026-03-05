// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ironriders.core;

import org.ironriders.climber.ClimberCommands;
import org.ironriders.climber.ClimberSubsystem;
import org.ironriders.drive.DriveCommands;
import org.ironriders.drive.DriveConstants;
import org.ironriders.drive.DriveSubsystem;
import org.ironriders.drive.PathPlannerHelpers;
import org.ironriders.lib.DriverRequest;
import org.ironriders.lib.DriverRequest.AlignTargetingMode;
import org.ironriders.lib.DriverRequest.PriorityMode;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.lib.field.Zone;
import org.ironriders.lib.field.Zone.ZoneType;
import org.ironriders.manipulation.indexer.IndexerCommands;
import org.ironriders.manipulation.indexer.IndexerSubsystem;
import org.ironriders.manipulation.intake.IntakeCommands;
import org.ironriders.manipulation.intake.IntakeSubsystem;
import org.ironriders.manipulation.launcher.LauncherCommands;
import org.ironriders.manipulation.launcher.LauncherMaps;
import org.ironriders.manipulation.launcher.LauncherSubsystem;
import org.ironriders.manipulation.launcher.LauncherConstants.State;
import org.ironriders.manipulation.wrist.WristCommands;
import org.ironriders.manipulation.wrist.WristSubsystem;
import org.ironriders.vision.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
    public static LauncherMaps launcherMaps = new LauncherMaps();

    public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static final DriveCommands driveCommands = DriveSubsystem.getCommands();

    public static final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    public static final IndexerCommands indexerCommands = indexerSubsystem.getCommands();

    public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public static final IntakeCommands intakeCommands = intakeSubsystem.getCommands();

    public static final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
    public static final LauncherCommands launcherCommands = launcherSubsystem.getCommands();

    public static final WristSubsystem wristSubsystem = new WristSubsystem();
    public static final WristCommands wristCommands = wristSubsystem.getCommands();

    public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public static final ClimberCommands climberCommands = climberSubsystem.getCommands();

    public static final VisionSubsystem visionSubsystem = new VisionSubsystem();

    public static Zone passingZone = new Zone(ZoneType.PASSING);
    public static Zone scoringZone = new Zone(ZoneType.SCORING);

    public final Double triggerThreshold = 0.75;

    private final SendableChooser<Command> autoChooser;

    public static final CommandXboxController primaryController = new CommandXboxController(
            DriveConstants.CONTROLLER_PRIMARY_PORT);

    public final static RobotCommands robotCommands = new RobotCommands(driveCommands, indexerCommands, intakeCommands,
                launcherCommands, wristCommands, climberCommands, primaryController.getHID());
    
        private static boolean targetingHub = false;
        private static boolean targetingPassing = false;
    
        public RobotContainer() {
            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Select", autoChooser);
    
            DriverStation.silenceJoystickConnectionWarning(true);
    
            passingZone = new Zone(ZoneType.PASSING);
            scoringZone = new Zone(ZoneType.SCORING);
    
            configureBindings();
        }
    
        public static void revertToSafeDefaults() {
            targetingHub = false;
            targetingPassing = false;
            TargetingControl.revertToSafeDefaults();
        }
    
        public static void init() {
            CommandScheduler.getInstance().schedule(robotCommands.stow());
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(Commands.parallel(
                robotCommands
                        .driveTeleop(
                                () -> Utils.controlCurve(primaryController.getLeftY(),
                                        DriveConstants.TRANSLATION_CONTROL_EXPONENT,
                                        DriveConstants.TRANSLATION_CONTROL_DEADBAND),
                                () -> Utils.controlCurve(primaryController.getLeftX(),
                                        DriveConstants.TRANSLATION_CONTROL_EXPONENT,
                                        DriveConstants.TRANSLATION_CONTROL_DEADBAND),
                                () -> Utils.controlCurve(primaryController.getRightX(),
                                        DriveConstants.ROTATION_CONTROL_EXPONENT,
                                        DriveConstants.ROTATION_CONTROL_DEADBAND))
                        .withName("Drive Teleop")));

        primaryController.a().onTrue(
                new InstantCommand(() -> {
                    targetingHub = !targetingHub;
                    if (targetingHub) {
                        targetingPassing = false;
                        TargetingControl.targetHub();
                    } else {
                        revertToSafeDefaults();
                    }
                }));

        primaryController.x().onTrue(
                new InstantCommand(() -> {
                    targetingPassing = !targetingPassing;
                    if (targetingPassing) {
                        targetingHub = false;
                        TargetingControl.targetPassing();
                    } else {
                        revertToSafeDefaults();
                    }
                }));

        // --- Align ---
        primaryController.y()
                .onTrue(buildAlignCommand(new DriverRequest(PriorityMode.ALIGN_PRIORITY, AlignTargetingMode.OUTPOST)))
                .onFalse(Commands.runOnce(() -> revertToSafeDefaults()));

        primaryController.b()
                .onTrue(buildAlignCommand(new DriverRequest(PriorityMode.ALIGN_PRIORITY, AlignTargetingMode.BUMP)))
                .onFalse(Commands.runOnce(() -> revertToSafeDefaults()));

        primaryController.leftBumper().onTrue(driveCommands
                .pathfindThenFlipPathIfBetterThenFollow(PathPlannerHelpers.loadPath("Center Sweep").orElseThrow()));

        // Line up to score
        primaryController.rightBumper().onTrue(
                driveCommands.pathfindToPoseThenAimAt(scoringZone.centerPoint(),
                        FieldPositions.get(ElementType.HUB).toPose2d()));

        // TODO: This binding currently only runs the kicker directly. It should
        // eventually be updated to use robotCommands.fire() (or handle Launcher state)
        // to ensure the flywheels and hood spin up properly.
        primaryController.rightTrigger(triggerThreshold).whileTrue(launcherCommands.runKicker());
        // primaryController.rightTrigger(triggerThreshold).whileTrue(robotCommands.fire());

        primaryController.leftTrigger(triggerThreshold).whileTrue(launcherCommands.set(State.STOW));

        primaryController.povUp().whileTrue(Commands.runOnce(() -> LauncherSubsystem.trim(1)));

        primaryController.povDown().whileTrue(Commands.runOnce(() -> LauncherSubsystem.trim(-1)));
    }

    public Command buildAlignCommand(DriverRequest request) {
        return Commands
                .runOnce(() -> {
                    request.send("");
                    targetingHub = false;
                    targetingPassing = false;
                });
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
