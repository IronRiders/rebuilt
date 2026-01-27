package org.ironriders.wrist;

import org.ironriders.lib.IronSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
/**
 * Subsystem for controlling ?wrist?
 */
public class WristSubsystem extends IronSubsystem {
    private final com.ctre.phoenix6.hardware.TalonFX wristMotor = new TalonFX(
            WristConstants.WristMotorCantID);
    private final ProfiledPIDController pid;
    private final WristCommands commands;
    private final ArmFeedforward armFeedforward;

    // public final Command <---- Set this when I have commands
    public WristSubsystem() {
        wristMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(WristConstants.WristMotorSupplyCurrentLimit));
        pid = new ProfiledPIDController(WristConstants.PIDProportional, WristConstants.PIDIntegral,
                WristConstants.PIDDerivative, WristConstants.Constraints);
        commands = new WristCommands(this);
        armFeedforward = new ArmFeedforward(WristConstants.FeedForward.kS, WristConstants.FeedForward.kG,
                WristConstants.FeedForward.kV);
    }

    @Override
    public void periodic() {
        double pos = this.getPosition();
        wristMotor.setVoltage(
            pid.calculate(pos)
            + armFeedforward.calculate(
                pid.getSetpoint().position,
                pid.getSetpoint().velocity)); // TODO: check if correct
    }


    /**
     * Gets the position of the arm.
     * Currently this won't work with the proposed design with the dead shaft
     * <i>When this is swapped for the new design, MAKE SURE the encoder reads 0 degrees (or add offset make it) as horizontal,
     * as {@linkplain edu.wpi.first.math.controller.ArmFeedforward#calculate ArmFeedForward} requires.</i>
     * @return angle in degrees from horizontal
     */
    public double getPosition() {
        return wristMotor.getPosition().getValueAsDouble(); // TODO: fix when we know what abs encoder we're using
    }

    /**
     * Used to reset relative encoder, shouldn't be needed if we switch to absolute encoder
     */
    public void resetRelativeEncoderRotations() {
        wristMotor.setPosition(0.0); // I think this is it, may cause a problem
    }
    /**
     * Sets the {@linkplain edu.wpi.first.math.trajectory.TrapezoidProfile Trapezoid Profile} goal for the {@linkplain com.ctre.phoenix6.hardware.TalonFX wrist motor's} {@linkplain edu.wpi.first.math.controller.ProfiledPIDController profiled pid}.
     * @param goal Up and down double positions (interchangable with trapazoid profile's with a velocity goal of zero), currently both set to 0.0. Should be set in degrees from horizontal.
     */
    public void setGoal(double goal) {
        pid.setGoal(goal);
    }
}