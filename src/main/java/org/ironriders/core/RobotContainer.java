// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ironriders.core;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ironriders.climb.ClimbCommands;
import org.ironriders.climb.ClimbConstants;
import org.ironriders.climb.ClimbSubsystem;
import org.ironriders.core.ElevatorWristControl.ElevatorWristState;
import org.ironriders.drive.DriveCommands;
import org.ironriders.drive.DriveConstants;
import org.ironriders.drive.DriveSubsystem;
import org.ironriders.intake.IntakeCommands;
import org.ironriders.intake.IntakeConstants.IntakeState;
import org.ironriders.intake.IntakeSubsystem;
import org.ironriders.lib.RobotUtils;
import org.ironriders.targeting.TargetingCommands;
import org.ironriders.targeting.TargetingSubsystem;

/**
 * Different button configurations for the driver controls PRIMARY_DRIVER: same as Driver Centered
 * Control Layout in the doc PRIMARY_DRIVER_WITH_BOOST: same as `William Boost buttons + primary
 * focus` in the doc SECONDARY_DRIVER_WITH_BOOST: same as `Secondary driver elevator controls` in
 * the doc.
 */
enum Config {
  PRIMARY_DRIVER,
  PRIMARY_DRIVER_WITH_BOOST,
  SECONDARY_DRIVER_WITH_BOOST;
}

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final DriveCommands driveCommands = driveSubsystem.getCommands();

  public final TargetingSubsystem targetingSubsystem = new TargetingSubsystem();
  public final TargetingCommands targetingCommands = targetingSubsystem.getCommands();

  public final ElevatorWristControl elevatorWristCommands = new ElevatorWristControl();

  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final IntakeCommands intakeCommands = intakeSubsystem.getCommands();

  public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  public final ClimbCommands climbCommands = climbSubsystem.getCommands();

  public final Double triggerThreshold = 0.75;

  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController primaryController =
      new CommandXboxController(DriveConstants.PRIMARY_CONTROLLER_PORT);
  private final CommandGenericHID secondaryController =
      new CommandJoystick(DriveConstants.KEYPAD_CONTROLLER_PORT);

  public final RobotCommands robotCommands =
      new RobotCommands(
          driveCommands,
          targetingCommands,
          intakeCommands,
          elevatorWristCommands,
          climbCommands,
          primaryController.getHID());

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   *
   * <p>builds the autos using {@link com.pathplanner.lib.auto.AutoBuilder#buildAutoChooser()
   * buildAutoChooser()} posts the auto selection to {@link SmartDashboard#putData(String,
   * SendableChooser) SmartDashboard}
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Select", autoChooser);
  }

  /**
   * Th {@link CommandGenericHID#button(int)} method (such as {@link
   * CommandXboxController#button(int)}, {@link CommandJoystick#button(int)}, or one of the {@link
   * edu.wpi.first.wpilibj2.command.button.Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructors.
   */
  private void configureBindings() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // This configures what control scheme the controller will use.
    // Changing this before the match will change the control layout for the driver
    // this may be useful if different drivers prefer different configurations.
    // See following document for configurations:
    // https://docs.google.com/document/d/1xFyZLRxw_a8ykvMorcS_41jLqNv0-CR9rZUkbbDNRMI/edit?copiedFromTrash&tab=t.0
    Config buttonConfiguration = Config.PRIMARY_DRIVER_WITH_BOOST;

    // DRIVE CONTROLS
    driveSubsystem.setDefaultCommand(
        robotCommands.driveTeleop(
            () ->
                RobotUtils.controlCurve(
                    -primaryController.getLeftY() // This sets the robot's x
                        // translation (as
                        // seen in driveTeleop) to
                        // the left
                        // joystick's y value
                        * driveSubsystem.controlSpeedMultipler,
                    DriveConstants.TRANSLATION_CONTROL_EXPONENT,
                    DriveConstants.TRANSLATION_CONTROL_DEADBAND), // the deadband for the
            // controller, not being
            // used
            // right now
            () ->
                RobotUtils.controlCurve(
                    -primaryController.getLeftX() // this sets the robot's y
                        // translation (as
                        // seen in driveTeleop) to
                        // the left
                        // joystick's x value
                        * driveSubsystem.controlSpeedMultipler, // for all these, see getLeftY
                    DriveConstants.TRANSLATION_CONTROL_EXPONENT,
                    DriveConstants.TRANSLATION_CONTROL_DEADBAND),
            () ->
                RobotUtils.controlCurve(
                    -primaryController.getRightX() // this rotates the robot
                        // based on the
                        // right joysticks x value
                        // (y value is
                        // unused)
                        * driveSubsystem.controlSpeedMultipler,
                    DriveConstants.ROTATION_CONTROL_EXPONENT,
                    DriveConstants.ROTATION_CONTROL_DEADBAND)));

    switch (buttonConfiguration) { // configures buttons based on selected config. see the
      // buttonConfiguration to
      // know the currently active configuration
      // currently, 'PRIMARY_DRIVER_WITH_BOOST' is the 'normal' configuration, when in
      // doubt use this one. // (see
      // https://europe1.discourse-cdn.com/unity/original/3X/5/8/58e7b2a50ec35ea142ae9c4d27c9df2d372cd1f3.jpeg
      // for button numbers)
      case PRIMARY_DRIVER:
        for (var angle = 0; angle < 360; angle += 45) {
          primaryController.pov(angle).onTrue(driveCommands.jog(-angle));
        }

        // Go to intaking, then grab until told to stop
        primaryController
            .rightTrigger(triggerThreshold)
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.INTAKING))
            .onTrue(intakeCommands.set(IntakeState.GRAB))
            .onFalse(intakeCommands.set(IntakeState.STOP));

        // Score coral
        primaryController
            .leftTrigger(triggerThreshold)
            .onTrue(intakeCommands.set(IntakeState.SCORE))
            .onFalse(intakeCommands.set(IntakeState.STOP));

        // Elevator Down
        primaryController
            .rightBumper()
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.INTAKING));

        // Eject coral
        primaryController
            .leftBumper()
            .onTrue(intakeCommands.set(IntakeState.EJECT))
            .onFalse(intakeCommands.set(IntakeState.STOP));

        primaryController
            .button(1)
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.L2));
        primaryController
            .button(2) // works for L1 as well (see the above link for button
            // configuration
            // information)
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.L2));

        primaryController
            .button(3)
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.L3));
        primaryController
            .button(4)
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.L4));
        break;

      case PRIMARY_DRIVER_WITH_BOOST:
        primaryController.povRight().onTrue(driveCommands.jog(-90));

        primaryController.povLeft().onTrue(driveCommands.jog(90));

        primaryController.povUp().onTrue(intakeCommands.boost());
        primaryController.povDown().onTrue(intakeCommands.unboost());

        // Intake and then go down
        primaryController
            .rightTrigger(triggerThreshold)
            .onTrue(robotCommands.intake())
            .onFalse(robotCommands.stopIntake());

        primaryController
            .leftTrigger(triggerThreshold)
            .whileTrue(intakeCommands.set(IntakeState.SCORE))
            .onFalse(intakeCommands.set(IntakeState.STOP));

        primaryController
            .leftBumper()
            .onTrue(driveCommands.setDriveTrainSpeed(0.5))
            .onFalse(driveCommands.setDriveTrainSpeed(1));

        primaryController
            .rightBumper()
            .onTrue(driveCommands.setDriveTrainSpeed(1.5))
            .onFalse(driveCommands.setDriveTrainSpeed(1));

        primaryController.a().onTrue(robotCommands.elevatorWristSet(ElevatorWristState.HOLD));
        primaryController.b().onTrue(robotCommands.elevatorWristSet(ElevatorWristState.L2));

        primaryController.x().onTrue(robotCommands.elevatorWristSet(ElevatorWristState.L3));

        primaryController.y().onTrue(robotCommands.elevatorWristSet(ElevatorWristState.L4));

        break;

      case SECONDARY_DRIVER_WITH_BOOST:
        for (var angle = 0; angle < 360; angle += 45) {
          primaryController.pov(angle).onTrue(driveCommands.jog(-angle));
        }

        primaryController
            .rightTrigger(triggerThreshold)
            .onTrue(robotCommands.intake())
            .onFalse(robotCommands.stopIntake());

        primaryController
            .leftTrigger(triggerThreshold)
            .onTrue(intakeCommands.set(IntakeState.SCORE))
            .onFalse(intakeCommands.set(IntakeState.STOP));

        primaryController
            .leftBumper()
            .onTrue(driveCommands.setDriveTrainSpeed(0.5))
            .onFalse(driveCommands.setDriveTrainSpeed(1));
        primaryController
            .rightBumper()
            .onTrue(driveCommands.setDriveTrainSpeed(1.5))
            .onFalse(driveCommands.setDriveTrainSpeed(1));

        secondaryController
            .button(5) // TODO: put actual button #
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.L2));
        secondaryController
            .button(6) // TODO: put actual button #
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.L2));
        secondaryController
            .button(7) // TODO: put actual button #
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.L3));
        secondaryController
            .button(8) // TODO: put actual button #
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.L4));
        secondaryController
            .button(9) // TODO: put actual button #
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.INTAKING));
        secondaryController
            .button(10) // TODO: put actual button #
            .onTrue(elevatorWristCommands.setElevatorWrist(ElevatorWristState.HOLD));
        break;
      default:
        throw new Error("Invalid buttonmap type!");
    } // TODO: figure out what driveteam wants to do with this.

    secondaryController
        .button(16) // TODO: decide on buttons for these commands
        .onTrue(climbCommands.set(ClimbConstants.Targets.CLIMBED));
    secondaryController
        .button(15) // TODO: decide on buttons for these commands
        .onTrue(climbCommands.set(ClimbConstants.Targets.MIN));
    secondaryController
        .button(14) // TODO: decide on buttons for these commands
        .onTrue(climbCommands.set(ClimbConstants.Targets.MAX));
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
