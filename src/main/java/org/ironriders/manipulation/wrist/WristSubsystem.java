package org.ironriders.manipulation.wrist;

import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.manipulation.wrist.WristConstants.State;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

/**
 * Subsystem for controlling wrist
 */
public class WristSubsystem extends IronSubsystem {

    /* MOTOR COMPONENTS */
    private final TalonFX wristMotor = new TalonFX(
            WristConstants.MOTOR_ID);

    private final ArmFeedforward armFeedforward = new ArmFeedforward(
            WristConstants.FeedForward.FRICTION,
            WristConstants.FeedForward.GRAVITY,
            WristConstants.FeedForward.COASTING);

    private final ProfiledPIDController pid = new ProfiledPIDController(
            WristConstants.P,
            WristConstants.I,
            WristConstants.D,
            WristConstants.CONSTRAINTS);

    private final CANcoder encoder = new CANcoder(WristConstants.ENCODER_ID);

    private State currentState = State.UP;

    private final WristCommands commands;

    private Double jostleMin = State.JOSTLE.position - WristConstants.JOSTLE_RANGE;
    private Double jostleMax = State.JOSTLE.position + WristConstants.JOSTLE_RANGE;

    public WristSubsystem() {
        commands = new WristCommands(this);

        TalonFXConfiguration configuration = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(WristConstants.CURRENT_LIMIT));

        wristMotor.getConfigurator()
                .apply(configuration);

        setGoal(currentState);

        pid.reset(getPosition());
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case JOSTLE:
                if (atGoal()) {
                    pid.setGoal(Utils.inRange(-WristConstants.JOSTLE_TOLERANCE, WristConstants.JOSTLE_TOLERANCE,
                            Math.abs(getPosition() - jostleMax)) ? jostleMin : jostleMax);
                }
                break;
            default:
        }
        wristMotor.set(pid.calculate(getPosition()) + armFeedforward.calculate(getPosition(), getVelocity()));
    }

    /**
     * Gets the position of the arm.
     * 
     * @return The angle in degrees from vertical, with positive forward.
     */
    public double getPosition() {
        return encoder.getAbsolutePosition().getValueAsDouble() - WristConstants.ENCODER_OFFSET;
    }

    /**
     * Gets the velocity of the arm in RPMs.
     * 
     * @return The velocity in RPMs
     */
    public double getVelocity() {
        return encoder.getVelocity().getValueAsDouble();
    }

    /**
     * Set the angle target for the wrist.
     */
    public void setGoal(State goal) {
        currentState = goal;
        pid.setGoal(goal.position);
    }

    /**
     * Checks if the wrist is at its goal.
     * Will always return false while jostling.
     * 
     * @return true if the wrist is at its goal, false otherwise or if jostling.
     */
    public boolean atGoal() {
        if (currentState == State.JOSTLE) {
            return false;
        }
        return pid.atGoal();
    }

    /**
     * @return The {@link WristCommands commands} for this subsystem.
     */
    public WristCommands getCommands() {
        return commands;
    }
}