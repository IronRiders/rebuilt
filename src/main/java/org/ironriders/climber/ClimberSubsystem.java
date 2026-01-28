package org.ironriders.climber;

import org.ironriders.climber.ClimberConstants.State;
import org.ironriders.lib.IronSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

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
        primaryMotor.getConfigurator().apply(configuration);
        secondaryMotor.getConfigurator().apply(configuration);

        pidController.reset(getPosition());
    }

    public void setMotors(double speed) {
        primaryMotor.set(speed);
        secondaryMotor.set(speed);
    }

    @Override
    public void periodic() {
        setMotors(pidController.calculate(getPosition()));
    }

    public ClimberCommands getCommands() {
        return commands;
    }

    public void setGoal(State goal) {
        pidController.setGoal(goal.position);
    }

    public boolean atGoal() {
        return pidController.atGoal();
    }

    public double getPosition() {
        return primaryMotor.getPosition().getValueAsDouble() * ClimberConstants.GEAR_RATIO;
    }
}