package org.ironriders.wrist;

import org.ironriders.lib.IronSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.controller.ProfiledPIDController;

public class WristSubsystem extends IronSubsystem {
    private final com.ctre.phoenix6.hardware.TalonFX wristMotor = new TalonFX(
            WristConstants.WristMotorCantID);
    private final ProfiledPIDController pid;
    private final WristCommands commands;

    // public final Command <---- Set this when I have commands
    public WristSubsystem() {
        wristMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(WristConstants.WristMotorSupplyCurrentLimit));
        pid = new ProfiledPIDController(WristConstants.PIDProportional, WristConstants.PIDIntegral, WristConstants.PIDDerivative, WristConstants.Constraints);
        commands = new WristCommands(this);
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