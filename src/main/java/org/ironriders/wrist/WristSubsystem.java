package org.ironriders.wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.ironriders.core.ElevatorWristControl.WristRotation;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.RobotUtils;

/** Subsystem for thw wrist. */
public class WristSubsystem extends IronSubsystem {
  final SparkMax primaryMotor =
      new SparkMax(WristConstants.PRIMARY_WRIST_MOTOR, MotorType.kBrushless);
  final SparkMax secondaryMotor =
      new SparkMax(WristConstants.SECONDARY_WRIST_MOTOR, MotorType.kBrushless);
  final TrapezoidProfile movementProfile =
      new TrapezoidProfile(new Constraints(WristConstants.MAX_VEL, WristConstants.MAX_ACC));

  public PIDController pidControler;

  private TrapezoidProfile.State goalSetpoint =
      new TrapezoidProfile.State(); // Acts as a final setpoint
  private TrapezoidProfile.State periodicSetpoint =
      new TrapezoidProfile.State(); // Acts as a temporary setpoint for
  // calculating the next speed value

  public WristRotation targetRotation = WristRotation.HOLD;

  private TrapezoidProfile.State stopped;

  private final WristCommands commands = new WristCommands(this);

  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  // private final ArmFeedforward feedforward = new ArmFeedforward(, , ); TODO add
  // ff
  /** Initalizer. */
  public WristSubsystem() {
    motorConfig
        .smartCurrentLimit(30) // Can go to 40
        .idleMode(IdleMode.kBrake);

    primaryMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    secondaryMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidControler = new PIDController(WristConstants.P, WristConstants.I, WristConstants.D);
    pidControler.setTolerance(WristConstants.TOLERANCE);
    reset();
  }

  @Override
  public void periodic() {
    // Apply profile and PID to determine output level
    periodicSetpoint = movementProfile.calculate(WristConstants.T, periodicSetpoint, goalSetpoint);

    double speed = pidControler.calculate(getCurrentAngle(), periodicSetpoint.position);
    setMotors(speed);

    debugPublish("Current PID ouput", speed);
    updateDashboard();
  }

  /** Push all values to elastic. */
  public void updateDashboard() {
    debugPublish("Current target", targetRotation.toString());
    debugPublish("Current goal pos", goalSetpoint.position);
    debugPublish("Current angle", getCurrentAngle());
    debugPublish("Current angle raw", primaryMotor.getAbsoluteEncoder().getPosition());
    debugPublish("At goal?", isAtPosition());
    debugPublish("Wrist PID", pidControler);
  }

  /** Get what angle the wrist is at. */
  public double getCurrentAngle() {
    return (primaryMotor.getAbsoluteEncoder().getPosition() - WristConstants.ENCODER_OFFSET) * 360
        + WristConstants.CAD_POSITION_OFFSET;
    /*
     * ENCODER_OFFSET is added to encoder to get it to = 0 when it is fully stowed
     * (against hardstop)
     * CAD_POSITION_OFFSET is adjustment for odd alignment in the CAD
     */
  }

  /** Is the wrist in position?. */
  public boolean isAtPosition() {
    return pidControler.atSetpoint();
  }

  /** Reset wrist as if rebooted. */
  public void reset() {
    logMessage("resetting");

    pidControler.reset();

    stopped = new TrapezoidProfile.State(getCurrentAngle(), 0);

    goalSetpoint = stopped;
    periodicSetpoint = stopped;

    setMotors(0);
  }

  /** Is the wrist at it's goal?. */
  public boolean atGoal() {
    return RobotUtils.tolerance(getCurrentAngle(), goalSetpoint.position, WristConstants.TOLERANCE);
  }

  /** Set both wrist motors. */
  private void setMotors(double speed) {
    primaryMotor.set(speed);
    secondaryMotor.set(-speed);
  }

  /** Set the wrist goal state. */
  protected void setGoal(WristRotation rotation) {
    goalSetpoint = new TrapezoidProfile.State(rotation.pos, 0);
    targetRotation = rotation;
  }

  public WristCommands getCommands() {
    return commands;
  }
}
