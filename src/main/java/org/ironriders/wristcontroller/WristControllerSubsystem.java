package org.ironriders.wristcontroller;

import org.ironriders.lib.IronSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.controller.ProfiledPIDController;

public class WristControllerSubsystem extends IronSubsystem {
    private final com.ctre.phoenix6.hardware.TalonFX wristMotor = new TalonFX(
            WristControllerConstants.WristMotorCantID);
    private final ProfiledPIDController pid;
    private final WristControllerCommands commands;

    // public final Command <---- Set this when I have commands
    public WristControllerSubsystem() {
        wristMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(WristControllerConstants.WristMotorSupplyCurrentLimit));
        pid = new ProfiledPIDController(WristControllerConstants.PIDProportional, WristControllerConstants.PIDIntegral, WristControllerConstants.PIDDerivative, WristControllerConstants.Constraints);
        commands = new WristControllerCommands(this);
    }

    @Override
    public void periodic() {
        pid.calculate(this.getPosition()); 
    }

    public double getPosition() {
        return wristMotor.getPosition().getValueAsDouble(); //TODO: fix when we know what abs encoder we're using
    }
    
    public void resetRelativeEncoderRotations() {
        wristMotor.setPosition(0.0); // I think this is it, may cause a problem
    }
    public void setGoal(double goal) {
        pid.setGoal(goal);
    }
}