// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ironriders.core;

import org.ironriders.climber.ClimberCommands;
import org.ironriders.climber.ClimberConstants;
import org.ironriders.climber.ClimberSubsystem;
import org.ironriders.core.DriverRequest.AlignTargetingMode;
import org.ironriders.core.DriverRequest.PriorityMode;
import org.ironriders.drive.DriveCommands;
import org.ironriders.drive.DriveConstants;
import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.Zone;
import org.ironriders.lib.field.Zone.ZoneType;
import org.ironriders.manipulation.indexer.IndexerCommands;
import org.ironriders.manipulation.indexer.IndexerSubsystem;
import org.ironriders.manipulation.intake.IntakeCommands;
import org.ironriders.manipulation.intake.IntakeConstants;
import org.ironriders.manipulation.intake.IntakeSubsystem;
import org.ironriders.manipulation.launcher.LauncherCommands;
import org.ironriders.manipulation.launcher.LauncherMaps;
import org.ironriders.manipulation.launcher.LauncherSubsystem;
import org.ironriders.manipulation.wrist.WristCommands;
import org.ironriders.manipulation.wrist.WristSubsystem;
import org.ironriders.vision.VisionCommands;
import org.ironriders.vision.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
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
    public static LauncherMaps launcherMaps = new LauncherMaps();

    public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static final DriveCommands driveCommands = driveSubsystem.getCommands();

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
    public static final VisionCommands visionCommands = visionSubsystem.getCommands();

    public static Zone passingZone = new Zone(ZoneType.PASSING);
    public static Zone scoringZone = new Zone(ZoneType.SCORING);

    public final Double triggerThreshold = 0.75;

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController primaryController = new CommandXboxController(
            DriveConstants.CONTROLLER_PRIMARY_PORT);
    private final CommandGenericHID secondaryController = new CommandJoystick(
            DriveConstants.CONTROLLER_SECONDARY_PORT);

    public final RobotCommands robotCommands = new RobotCommands(driveCommands, indexerCommands, intakeCommands,
            launcherCommands, wristCommands, climberCommands, visionCommands, primaryController.getHID());

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     * <hr />
     * builds the autos using
     * {@link com.pathplanner.lib.auto.AutoBuilder#buildAutoChooser()
     * buildAutoChooser()}, posts the auto selection to
     * {@link SmartDashboard#putData(String, SendableChooser) SmartDashboard}.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
        DogLog.setPdh(new PowerDistribution());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Select", autoChooser);

        passingZone = new Zone(ZoneType.PASSING);
        scoringZone = new Zone(ZoneType.SCORING);

        LauncherMaps.AngleToExtensionMap.getAngleForExtension(226d);
    }

    public void periodic() {
        TargetingControl.update();
    }

    /**
     * Configures primary & secondary {@linkplain CommandGenericHID controllers}
     * bindings to commands. This also configures the default driving (controller
     * sticks)
     */
    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // See: https://tinyurl.com/yua72bn2

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

        primaryController.rightBumper().onTrue(Commands.runOnce(
                () -> DriveSubsystem.setSpeedMax(DriveConstants.SWERVE_MAX_TRANSLATION_PATHFIND +
                        3)))
                .onFalse(Commands.runOnce(() -> DriveSubsystem
                        .setSpeedMax(DriveConstants.SWERVE_MAX_TRANSLATION_PATHFIND)));

        primaryController.leftBumper().onTrue(Commands.runOnce(
                () -> DriveSubsystem.setSpeedMax(DriveConstants.SWERVE_MAX_TRANSLATION_PATHFIND -
                        2)))
                .onFalse(Commands.runOnce(() -> DriveSubsystem
                        .setSpeedMax(DriveConstants.SWERVE_MAX_TRANSLATION_PATHFIND)));

        primaryController.rightTrigger(triggerThreshold)
                .onTrue(intakeCommands.set(IntakeConstants.State.INTAKE))
                .onFalse(intakeCommands.set(IntakeConstants.State.STOP));

        primaryController.a().onTrue(Commands.sequence(TargetingControl.targetHub()/* , robotCommands.score() */))
                .onFalse(Commands.runOnce(() -> TargetingControl.revertToSafeDefaults()));

        primaryController.b()
                .onTrue(Commands
                        .runOnce(() -> new DriverRequest(PriorityMode.ALIGN_PRIORITY, AlignTargetingMode.BUMP)
                                .send("target bump")))
                .onFalse(Commands.runOnce(() -> TargetingControl.revertToSafeDefaults()));

        primaryController.x().onTrue(Commands.sequence(TargetingControl.targetPassing()/* , robotCommands.score() */))
                .onFalse(Commands.runOnce(() -> TargetingControl.revertToSafeDefaults()));

        primaryController.y()
                .onTrue(Commands
                        .runOnce(() -> new DriverRequest(PriorityMode.ALIGN_PRIORITY, AlignTargetingMode.OUTPOST)
                                .send("target outpost")))
                .onFalse(Commands.runOnce(() -> TargetingControl.revertToSafeDefaults()));

        primaryController.povUp().onTrue(climberCommands.set(ClimberConstants.State.MAX));

        primaryController.povDown().onTrue(climberCommands.set(ClimberConstants.State.CLIMBED));
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
