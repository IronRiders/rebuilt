package org.ironriders.climb;

import static org.ironriders.climb.ClimbConstants.D;
import static org.ironriders.climb.ClimbConstants.ENCODER_SCALE;
import static org.ironriders.climb.ClimbConstants.I;
import static org.ironriders.climb.ClimbConstants.MAX_ACC;
import static org.ironriders.climb.ClimbConstants.MAX_VEL;
import static org.ironriders.climb.ClimbConstants.P;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.ironriders.climb.ClimbConstants.Targets;
import org.ironriders.lib.IronSubsystem;

public class ClimbSubsystem extends IronSubsystem {

    private final SparkMax motor = new SparkMax(ClimbConstants.CLIMBER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACC));

    private TrapezoidProfile.State stopped;

    private final PIDController pid = new PIDController(P, I, D);

    public boolean atGoal;

    // goalSetpoint is the final goal. periodicSetpoint is a sort-of inbetween
    // setpoint generated every periodic.
    private TrapezoidProfile.State goalSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State periodicSetpoint = new TrapezoidProfile.State();

    private Targets currentTarget = Targets.MIN;

    private final ClimbCommands commands;

    /**
     * {@linkplain com.revrobotics.spark.config.SparkMaxConfig configures} the
     * {@linkplain
     * ClimbSubsystem#motor climb motor}; {@linkplain
     * edu.wpi.first.math.controller.PIDController#setTolerance(double) sets} the
     * {@linkplain
     * ClimbConstants#TOLERANCE tolerance} for the {@linkplain ClimbSubsystem#pid
     * PID controller};
     * {@linkplain ClimbSubsystem#home() homes} the {@linkplain ClimbSubsystem
     * climber}; constructs
     * {@link ClimbCommands commands}.
     */
    public ClimbSubsystem() {
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(ClimbConstants.CURRENT_LIMIT);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pid.setTolerance(ClimbConstants.TOLERANCE);

        home();

        commands = new ClimbCommands(this);
    }

    @Override
    public void periodic() {
        updateDashboard();

        var currentDegrees = getCurrentAngle();

        // Apply profile and PID to determine output level
        periodicSetpoint = profile.calculate(ClimbConstants.T, periodicSetpoint, goalSetpoint);

        var speed = pid.calculate(currentDegrees, periodicSetpoint.position);
        motor.set(speed);

        atGoal = pid.atSetpoint();

        debugPublish("PID out", speed);
    }

    private void updateDashboard() {
        debugPublish("Goal State", currentTarget.toString());
        debugPublish("Goal Position", goalSetpoint.position);
        debugPublish("Motor Current", motor.getOutputCurrent());

        debugPublish("Current Position", getCurrentAngle());
        debugPublish("PID", pid);
        debugPublish("at Goal?", atGoal);
        debugPublish("Motor raw angle", motor.getEncoder().getPosition());
    }

    /**
     * {@linkplain edu.wpi.first.math.controller.PIDController#reset() Resets} the
     * {@linkplain
     * ClimbSubsystem#pid PID controller}; sets the
     * {@linkplain ClimbSubsystem#stopped stopped state},
     * {@linkplain ClimbSubsystem#goalSetpoint goal setpoint}, and {@linkplain
     * ClimbSubsystem#periodicSetpoint periodic setpoint} to the current position
     * with zero velocity;
     * {@linkplain com.revrobotics.spark.SparkBase#set(double) sets} the {@linkplain
     * org.ironriders.climb.ClimbSubsystem#motor motor} to {@linkplain Integer#ZERO
     * 0}.
     */
    public void reset() {
        pid.reset();
        stopped = new TrapezoidProfile.State(getCurrentAngle(), 0);
        goalSetpoint = stopped;
        periodicSetpoint = stopped;

        motor.set(0);
        logMessage("resetting");
    }

    /**
     * Homes the climber unless the climber is not at
     * {@linkplain ClimbConstants.Targets#MIN MIN} (to
     * prevent damage to the climber).
     */
    public void home() {
        if (currentTarget != Targets.MIN) {
            // The climber is not all the way down, resetting it's encoder would cause it to
            // go boom.
            logMessage("aborting home, climber state is not MIN!");
            return;
        }
        logMessage("rehoming!");
        motor.getEncoder().setPosition(0);
        reset();
    }

    protected void setGoal(ClimbConstants.Targets target) {
        goalSetpoint = new TrapezoidProfile.State(target.pos, 0);
        currentTarget = target;

        logMessage("goes to " + currentTarget.toString());
    }

    public double getCurrentAngle() {
        return motor.getEncoder().getPosition() * 360 * ENCODER_SCALE;
    }

    public ClimbCommands getCommands() {
        return commands;
    }
}
