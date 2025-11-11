package org.ironriders.elevator;

import static org.ironriders.elevator.ElevatorConstants.ELEVATOR_MOTOR_STALL_LIMIT;
import static org.ironriders.elevator.ElevatorConstants.ELEVATOR_POSITION_TOLERANCE;
import static org.ironriders.elevator.ElevatorConstants.FOLLOW_MOTOR_ID;
import static org.ironriders.elevator.ElevatorConstants.INCHES_PER_ROTATION;
import static org.ironriders.elevator.ElevatorConstants.PRIMARY_MOTOR_ID;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.ironriders.core.ElevatorWristCTL.ElevatorLevel;
import org.ironriders.lib.IronSubsystem;

/**
 * This subsystem controls the big ol' elevator that moves the manipulator
 * vertically.
 */
public class ElevatorSubsystem extends IronSubsystem {
    private final ElevatorCommands commands;

    private final SparkMax primaryMotor = new SparkMax(PRIMARY_MOTOR_ID, MotorType.kBrushless); // lead motor
    private final SparkMax followerMotor = new SparkMax(FOLLOW_MOTOR_ID, MotorType.kBrushless);

    private final SparkLimitSwitch bottomLimitSwitch = primaryMotor.getReverseLimitSwitch();

    private final RelativeEncoder encoder = primaryMotor.getEncoder();

    // profile is used to calculate our periodic outputs
    private final TrapezoidProfile profile;
    private final ElevatorFeedforward feedforward;
    private final PIDController pidController;

    // goalSetpoint is the final goal. periodicSetpoint is a sort-of inbetween
    // setpoint generated every periodic.
    private TrapezoidProfile.State goalSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State periodicSetpoint = new TrapezoidProfile.State();

    private ElevatorLevel currentTarget = ElevatorLevel.DOWN;
    private boolean isHomed = false;

    public ElevatorSubsystem() {
        // lots of config!!
        SparkMaxConfig primaryConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        LimitSwitchConfig forwardLimitSwitchConfig = new LimitSwitchConfig()
                .forwardLimitSwitchEnabled(true)
                .forwardLimitSwitchType(Type.kNormallyClosed);
        LimitSwitchConfig reverseLimitSwitchConfig = new LimitSwitchConfig()
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(Type.kNormallyClosed);

        primaryConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ELEVATOR_MOTOR_STALL_LIMIT)
                .inverted(true)
                .apply(forwardLimitSwitchConfig)
                .apply(reverseLimitSwitchConfig);

        followerConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ELEVATOR_MOTOR_STALL_LIMIT)
                .follow(ElevatorConstants.PRIMARY_MOTOR_ID, true);

        primaryMotor.configure(
                primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VEL, ElevatorConstants.MAX_ACC));

        pidController = new PIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D);

        feedforward = new ElevatorFeedforward(ElevatorConstants.S, ElevatorConstants.G, ElevatorConstants.V);

        reset();
        commands = new ElevatorCommands(this);
    }

    @Override
    public void periodic() {
        // if (getHeight() > ElevatorLevel.L3.pos & elevatorWristCTL.getWristRotation()
        // < WristRotation.L2L3.pos) {
        // logMessage("ELEVATOR STOPPED DUE TO BAD WRIST POSITION, WAITING");
        // primaryMotor.set(0);
        // return;
        // }

        // Calculate the next state and update the current state
        periodicSetpoint = profile.calculate(ElevatorConstants.T, periodicSetpoint, goalSetpoint);

        // Only do PID if homed
        if (isHomed) {

            double pidOutput = pidController.calculate(getHeight(), periodicSetpoint.position);
            // It's now recommended you remove the velocity one. Not sure why but it was
            // causing a warning
            double ff = feedforward.calculate(periodicSetpoint.position);

            primaryMotor.set(pidOutput + ff);
        } else {
            if (bottomLimitSwitch.isPressed()) {
                setHomed();
                UpdateDashboard();
                return;
            }

            // logMessage("trying to home!"); // this will spam alot, debuging only
            primaryMotor.set(-ElevatorConstants.HOME_SPEED);
        }

        UpdateDashboard();
    }

    private void UpdateDashboard() {
        // -- Debugging --
        debugPublish("Homed", isHomed);
        debugPublish("Goal State", currentTarget.toString());
        debugPublish("PID", pidController);
        debugPublish("Goal Position", goalSetpoint.position);
        debugPublish("Real Pos", getHeight());
        if (isHomed) {
            debugPublish("At Goal?", isAtPosition());
        } else {
            debugPublish("At Goal?", "N/A; Not Homed!");
        }

        debugPublish("Forward Limit Switch", primaryMotor.getForwardLimitSwitch().isPressed());
        debugPublish("Reverse Limit Switch", primaryMotor.getReverseLimitSwitch().isPressed());

        debugPublish("Is at goal?", isAtPosition());
        debugPublish("Primary Encoder", primaryMotor.getEncoder().getPosition());
        debugPublish("Follower Encoder", followerMotor.getEncoder().getPosition());

        // -- Competition Info --
        publish("Homed?", isHomed);
    }

    public void setGoal(ElevatorLevel level) {
        this.goalSetpoint = new TrapezoidProfile.State(level.pos, 0d);
    }

    public void reset() {
        logMessage("resetting");
        primaryMotor.set(0);
        pidController.reset();
    }

    public void zeroGoal() {
        logMessage("set zero goal");
        goalSetpoint = new TrapezoidProfile.State(0, 0d);
        periodicSetpoint = new TrapezoidProfile.State(0, 0d);
    }

    public SparkLimitSwitch getBottomLimitSwitch() {
        return bottomLimitSwitch;
    }

    public void setHomed() {
        logMessage("setting homed");
        isHomed = true;
        encoder.setPosition(0); // reset
        zeroGoal();
        reset();
    }

    public void setNotHomed() {
        logMessage("setting non-homed");
        isHomed = false;
        zeroGoal();
        reset();
    }

    public boolean isHomed() {
        return isHomed;
    }

    public double getHeight() {
        return encoder.getPosition() * INCHES_PER_ROTATION;
    }

    public boolean isAtPosition() {
        return Math.abs(getHeight() - goalSetpoint.position) < ELEVATOR_POSITION_TOLERANCE;
    }

    public ElevatorCommands getCommands() {
        return commands;
    }
}
