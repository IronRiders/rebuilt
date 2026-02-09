package org.ironriders.climber;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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

    private TalonFXConfiguration configuration;

    public List<TalonFX> motors = List.of(new TalonFX(ClimberConstants.PRIMARY_ID),
            new TalonFX(ClimberConstants.SECONDARY_ID));

    public Map<TalonFX, List<Double>> rollingAverageMap = new HashMap<TalonFX, List<Double>>();

    public Map<TalonFX, Double> averageMap = new HashMap<TalonFX, Double>();

    public Map<TalonFX, Boolean> homedMap = new HashMap<TalonFX, Boolean>();

    private ProfiledPIDController pidController = new ProfiledPIDController(ClimberConstants.P, ClimberConstants.I,
            ClimberConstants.D, new Constraints(ClimberConstants.MAX_VEL, ClimberConstants.MAX_ACC));

    public ClimberSubsystem() {
        configuration = new TalonFXConfiguration();

        configuration.CurrentLimits.withSupplyCurrentLimit(ClimberConstants.STALL_LIMIT);

        motors.parallelStream().forEach((TalonFX motor) -> motor.getConfigurator().apply(configuration));

        motors.parallelStream().map((TalonFX motor) -> rollingAverageMap.put(motor, new ArrayList<Double>()));

        pidController.reset(getPosition());
    }

    /**
     * Sets the speed of both climber motors.
     * 
     * @param speed The speed to set the motors to (between -1 & 1)
     */
    public void setMotors(double speed) {
        motors.parallelStream().forEach(motor -> motor.set(speed));
    }

    @Override
    public void periodic() {
        if (motors.parallelStream()
                .map(motor -> homedMap.get(motor))
                .allMatch(homed -> Boolean.FALSE.equals(homed))) {

            setMotors(-ClimberConstants.HOME_SPEED);

            motors.parallelStream().forEach(motor -> homedMap.put(motor, currentCheckSpike(motor)));

            updateRollingAverage();
        } else {
            setMotors(pidController.calculate(getPosition()));
        }
    }

    public void updateRollingAverage() {
        motors.parallelStream().forEach(motor -> {
            List<Double> list = rollingAverageMap.get(motor);

            if (list.size() < 20) {
                list.add(getTorqueCurrent(motor));
            } else {
                list.remove(0);
                list.add(getTorqueCurrent(motor));
            }

            averageMap.put(motor, list.parallelStream().collect(Collectors.averagingDouble(num -> Double.valueOf(num))));
        });
    }

    public boolean currentCheckSpike(TalonFX motor) {
        return Math
                .abs(averageMap.get(motor) - getTorqueCurrent(motor)) > ClimberConstants.TORQUE_CURRENT_SPIKE_THRESHOLD;
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
        return motors.parallelStream().map(TalonFX::getPosition)
                .collect(Collectors.averagingDouble(StatusSignal::getValueAsDouble));
    }

    /*
     * Gets the average torque current going to the motors. Used for homing
     */
    public double getTorqueCurrent() {
        return motors.parallelStream().map(TalonFX::getTorqueCurrent)
                .collect(Collectors.averagingDouble(StatusSignal::getValueAsDouble));
    }

    /*
     * Gets the average torque current going to provided motor. Used for homing
     */
    public double getTorqueCurrent(TalonFX motor) {
        return motor.getTorqueCurrent().getValueAsDouble();
    }
}