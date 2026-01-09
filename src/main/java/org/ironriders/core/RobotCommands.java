package org.ironriders.core;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.ironriders.climb.ClimbCommands;
import org.ironriders.core.ElevatorWristControl.ElevatorWristState;
import org.ironriders.drive.DriveCommands;
import org.ironriders.intake.IntakeCommands;
import org.ironriders.intake.IntakeConstants.IntakeState;
import org.ironriders.targeting.TargetingCommands;

/**
 * These commands require more complex logic and are not directly tied to a subsystem. They
 * generally interface w/ multiple subsystems via their commands and are higher-level.
 *
 * <p>These commands are those which the driver controls call.
 */
@SuppressWarnings("unused") // Targeting and climb are unused by high-level commands
public class RobotCommands {
  private final DriveCommands driveCommands;
  private final TargetingCommands targetingCommands;
  private final IntakeCommands intakeCommands;
  private final ClimbCommands climbCommands;
  private final ElevatorWristControl elevatorWristCommands;

  private final GenericHID controller;

  /**
   * Creates final variables for all command classes.
   *
   * @param driveCommands DriveCommands instance
   * @param targetingCommands TargetingCommands instance
   * @param climbCommands ClimbCommands instance
   * @param controller GenericHID controller (joystick/gamepad) instance
   */
  public RobotCommands(
      DriveCommands driveCommands,
      TargetingCommands targetingCommands,
      IntakeCommands intakeCommands,
      ElevatorWristControl elevatorWristCommands,
      ClimbCommands climbCommands,
      GenericHID controller) {
    this.driveCommands = driveCommands;
    this.targetingCommands = targetingCommands;
    this.intakeCommands = intakeCommands;
    this.elevatorWristCommands = elevatorWristCommands;
    this.climbCommands = climbCommands;
    this.controller = controller;
    // TODO: More named commands, implement good autos

    NamedCommands.registerCommand("ElevatorWrist L2", elevatorWristSet(ElevatorWristState.L2));
    NamedCommands.registerCommand("ElevatorWrist L3", elevatorWristSet(ElevatorWristState.L3));
    NamedCommands.registerCommand("ElevatorWrist L4", elevatorWristSet(ElevatorWristState.L4));

    NamedCommands.registerCommand("Intake Eject", eject());
    NamedCommands.registerCommand("Intake", intake());
    NamedCommands.registerCommand("Score", intakeCommands.set(IntakeState.SCORE));

    SmartDashboard.putData("RobotCommands/Reset Gyro", resetGyroAngle());
  }

  /** Initialize all subsystems when first enabled. */
  public Command startup() {
    intakeCommands.setOnSuccess(() -> rumbleController());

    return elevatorWristCommands.reset(); // moves everything to zero
  }

  /**
   * Command to drive the robot given controller input.
   *
   * @param inputTranslationX DoubleSupplier, value from 0-1.
   * @param inputTranslationY DoubleSupplier, value from 0-1.
   * @param inputRotation DoubleSupplier, value from 0-1.
   */
  public Command driveTeleop(
      DoubleSupplier inputTranslationX,
      DoubleSupplier inputTranslationY,
      DoubleSupplier inputRotation) {
    return driveCommands.driveTeleop(inputTranslationX, inputTranslationY, inputRotation, true);
  }

  /**
   * Small translation that is robot-centered rather than field-centered. For example, moving a
   * little 30 degrees will move 30 degrees relative to the front of the robot, rather than relative
   * to the field.
   *
   * @param robotRelativeAngleDegrees The angle to move, in degrees relative to where the robot is
   *     facing
   * @return Returns command object that calls the {@link DriveCommands#jog(double)} method
   */
  public Command jog(double robotRelativeAngleDegrees) {
    return driveCommands.jog(robotRelativeAngleDegrees);
  }

  /**
   * Command to make the robot intake. Runs two commands in parallel:
   *
   * <ul>
   *   <li>Sets the {@link ElevatorWristControl#setElevatorWrist(ElevatorWristState) elevator wrist
   *       state} to {@link ElevatorWristState#INTAKING "INTAKING"}.
   *   <li>Sets the {@link IntakeCommands#set(IntakeState) intake state} to {@link IntakeState#GRAB
   *       "GRAB"}.
   * </ul>
   *
   * <br>
   *
   * @return returns the command described above
   */
  public Command intake() {
    return Commands.parallel(
        elevatorWristCommands.setElevatorWrist(ElevatorWristState.INTAKING),
        intakeCommands.set(IntakeState.GRAB));
    // .unless(() -> intakeCommands.getIntake().beamBreakTriggered());
  }

  /**
   * Command to make the robot eject. Simply sets the {@link IntakeCommands#set(IntakeState) intake
   * state} to {@link IntakeState#EJECT "eject"}.
   *
   * @return returns the command described above
   */
  public Command eject() {
    return intakeCommands.set(IntakeState.EJECT);
  }

  public Command resetGyroAngle() {
    return Commands.runOnce(() -> resetPigeon());
  }
  /**Initalize the Pigeon. */

  public void resetPigeon() {
    Pigeon2 pigeon2 = new Pigeon2(9);
    pigeon2.reset();
    pigeon2.close();
    driveCommands.resetRotation();
  }

  /**
   * Command to stop the intake and stow the elevator wrist. Does the following in parallel:
   *
   * <ul>
   *   <li>Sets the {@link ElevatorWristControl#setElevatorWrist(ElevatorWristState) elevator wrist
   *       state} to {@link ElevatorWristState#HOLD "stow"}.
   *   <li>Sets the {@link IntakeCommands#set(IntakeState) intake state} to {@link IntakeState#STOP
   *       "stop"}.
   * </ul>
   *
   * @return returns the command described above
   */
  public Command stopIntake() {
    return Commands.parallel(
        elevatorWristCommands.setElevatorWrist(ElevatorWristState.HOLD),
        intakeCommands.set(IntakeState.STOP));
    // .unless(() -> intakeCommands.getIntake().beamBreakTriggered()));
  }
  /**Go to a elevator wrist state. */
  
  public Command elevatorWristSet(ElevatorWristState state) {
    switch (state) {
      case L4:
        return Commands.parallel(
            elevatorWristCommands.setElevatorWrist(state), intakeCommands.boost());

      default:
        return elevatorWristCommands.setElevatorWrist(state);
    }
  }

  /**
   * Sets the rumble on the controller for 0.3 seconds.
   *
   * <p>Does this by setting the {@link
   * edu.wpi.first.wpilibj.GenericHID#setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType, double)
   * GenericHID setRumble()} method to {@link
   * edu.wpi.first.wpilibj.GenericHID.RumbleType#kBothRumble kBothRumble}. This makes all motors on
   * a controller rumble.
   *
   * @return A command that does what is described above for 0.3 seconds, then returns rumble to 0.
   */
  public Command rumbleController() {
    return Commands.sequence(
            Commands.runOnce(() -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 1)),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0)))
        .handleInterrupt(() -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0));
  }
}
