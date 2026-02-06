package org.ironriders.climber;

import org.ironriders.climber.ClimberConstants.State;
import org.ironriders.lib.IronSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * Subsystem for controlling the climber.
 */
public class ClimberSubsystem extends IronSubsystem {
    private ClimberCommands commands = new ClimberCommands(this);

    private final TalonFX primaryMotor = new TalonFX(ClimberConstants.PRIMARY_ID);
    private final TalonFX secondaryMotor = new TalonFX(ClimberConstants.SECONDARY_ID);

    private TalonFXConfiguration configuration;

    private ProfiledPIDController pidController = new ProfiledPIDController(ClimberConstants.P, ClimberConstants.I,
            ClimberConstants.D, new Constraints(ClimberConstants.MAX_VEL, ClimberConstants.MAX_ACC));

    public ClimberSubsystem() {
        configuration = new TalonFXConfiguration();

        configuration.CurrentLimits.withSupplyCurrentLimit(ClimberConstants.STALL_LIMIT);
        configuration.Feedback.withSensorToMechanismRatio(ClimberConstants.GEAR_RATIO);

        primaryMotor.getConfigurator().apply(configuration);
        secondaryMotor.getConfigurator().apply(configuration);

        pidController.reset(getPosition());
    }

    /**
     * Sets the speed of both climber motors.
     * 
     * @param speed The speed to set the motors to (between -1 & 1)
     */
    public void setMotors(double speed) {
        primaryMotor.set(speed);
        secondaryMotor.set(speed);
    }

    @Override
    public void periodic() {
        setMotors(pidController.calculate(getPosition()));
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
     * Gets the climber's current number of rotations from the primary motor's
     * sensor.
     * 
     * @return The current position of the climber.
     */
    public double getPosition() {
        return primaryMotor.getPosition().getValueAsDouble();
    }
}