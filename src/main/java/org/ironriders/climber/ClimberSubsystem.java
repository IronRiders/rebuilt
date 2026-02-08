package org.ironriders.climber;

import java.util.List;
import java.util.stream.Collectors;

import org.ironriders.climber.ClimberConstants.State;
import org.ironriders.lib.IronSubsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * Subsystem for controlling the climber.
 */
public class ClimberSubsystem extends IronSubsystem {
    private ClimberCommands commands = new ClimberCommands(this);

    public boolean isHomed = false;

    private TalonFXConfiguration configuration;

    public List<TalonFX> motors = List.of(new TalonFX(ClimberConstants.PRIMARY_ID),
            new TalonFX(ClimberConstants.SECONDARY_ID));

    public List<Double> rollingAverageDoubles;

    public Double rollingAverage = Double.POSITIVE_INFINITY;

    private ProfiledPIDController pidController = new ProfiledPIDController(ClimberConstants.P, ClimberConstants.I,
            ClimberConstants.D, new Constraints(ClimberConstants.MAX_VEL, ClimberConstants.MAX_ACC));

    public ClimberSubsystem() {
        configuration = new TalonFXConfiguration();

        configuration.CurrentLimits.withSupplyCurrentLimit(ClimberConstants.STALL_LIMIT);

        motors.stream().forEach((TalonFX motor) -> motor.getConfigurator().apply(configuration));

        pidController.reset(getPosition());
    }

    /**
     * Sets the speed of both climber motors.
     * 
     * @param speed The speed to set the motors to (between -1 & 1)
     */
    public void setMotors(double speed) {
        motors.stream().forEach((TalonFX motor) -> motor.set(speed));
    }

    @Override
    public void periodic() {
        if (!isHomed) {
            setMotors(-ClimberConstants.HOME_SPEED);

            isHomed = currentCheckSpike();
            updateRollingAverage();
        } else {
            setMotors(pidController.calculate(getPosition()));
        }
    }

    public void updateRollingAverage() {
        if (rollingAverageDoubles.size() < 20) {
            rollingAverageDoubles.add(getTorqueCurrent());
        } else {
            rollingAverageDoubles.remove(0);
            rollingAverageDoubles.add(getTorqueCurrent());
        }
        rollingAverage = rollingAverageDoubles.stream().collect(Collectors.averagingDouble(num -> Double.valueOf(num)));
    }

    public boolean currentCheckSpike() {
        return Math.abs(rollingAverage - getTorqueCurrent()) > ClimberConstants.TORQUE_CURRENT_SPIKE_THRESHOLD;
    }

    /**
     * @return The ClimberCommands object for the climber subsystem
     */
    public ClimberCommands getCommands() {
        return commands;
    }

    /**
     * Sets the climber's {@link ProfiledPIDController PID}
     * {@link ProfiledPIDController#getGoal() goal} to a given {@link State state}.
     * 
     * @param goal The target {@link State state} for the climber.
     */
    public void setGoal(State goal) {
        pidController.setGoal(goal.position);
    }

    /**
     * Checks if the climber is at its {@link ProfiledPIDController PID}
     * {@link ProfiledPIDController#getGoal() goal}.
     * 
     * @return {@link ProfiledPIDController#atGoal()}
     */
    public boolean atGoal() {
        return pidController.atGoal();
    }

    /**
     * Gets the climber's current number of rotations.
     * 
     * @return The current position of the climber.
     */
    public double getPosition() {
        return motors.stream().map(TalonFX::getPosition)
                .collect(Collectors.averagingDouble(StatusSignal::getValueAsDouble));
    }

    /*
     * Gets the average torque current going to the motors. Used for homing
     */
    public double getTorqueCurrent() {
        return motors.stream().map(TalonFX::getTorqueCurrent)
                .collect(Collectors.averagingDouble(StatusSignal::getValueAsDouble));
    }
}