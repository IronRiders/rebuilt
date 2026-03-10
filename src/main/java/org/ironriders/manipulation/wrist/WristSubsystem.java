package org.ironriders.manipulation.wrist;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.manipulation.wrist.WristConstants.State;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/**
 * Subsystem for controlling wrist
 */
public class WristSubsystem extends IronSubsystem {

    /* MOTOR COMPONENTS */
    private final TalonFX wristMotor = new TalonFX(
            WristConstants.MOTOR_ID);

    private final ArmFeedforward armFeedforward = new ArmFeedforward(
            WristConstants.S,
            WristConstants.G,
            WristConstants.V);

    private final ProfiledPIDController pid = new ProfiledPIDController(
            WristConstants.P,
            WristConstants.I,
            WristConstants.D,
            WristConstants.CONSTRAINTS);

    private final CANcoder encoder = new CANcoder(WristConstants.ENCODER_ID);

    private State currentState = State.DOWN;
    private double lastStateChangeTime = 0;

    private final WristCommands commands;

    private double jostlingIterator = 0;
    private CANcoderConfiguration cANcoderConfiguration = new CANcoderConfiguration();

    private Double jostleMin = State.JOSTLE.position - WristConstants.JOSTLE_RANGE;
    private Double jostleMax = State.JOSTLE.position + WristConstants.JOSTLE_RANGE;

    public WristSubsystem() {
        commands = new WristCommands(this);
        cANcoderConfiguration.MagnetSensor.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        cANcoderConfiguration.MagnetSensor.withMagnetOffset(WristConstants.ENCODER_OFFSET);
        encoder.getConfigurator().apply(cANcoderConfiguration);

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration
                .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(WristConstants.CURRENT_LIMIT));
        configuration.withMotorOutput(new MotorOutputConfigs().withInverted(WristConstants.MOTOR_INVERSION));
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        wristMotor.getConfigurator()
                .apply(configuration);

        setGoal(currentState);
        pid.reset(getPositionRaw());
    }

    @Override
    public void periodic() {
        publish("pos", getPositionDegrees());
        publish("Pos Raw", getPositionRaw());
        publish("pid", pid);

        publish("state", currentState.name());

        switch (currentState) {
            case JOSTLE:
                double timeDifference = Timer.getFPGATimestamp() - lastStateChangeTime;
                if (timeDifference > 2.5) {
                    lastStateChangeTime = Timer.getFPGATimestamp();
                    break;
                } else if (timeDifference > 1.5) {
                    pid.setGoal(State.DOWN.position);
                    break;
                } else if (timeDifference > 1) {
                    pid.setGoal(State.UP.position);
                    break;
                } else if (timeDifference > .5) {
                    pid.setGoal(State.DOWN.position);
                    break;
                } else {
                    pid.setGoal(State.UP.position);
                }
            default:
                break;
        }
        double output = pid.calculate(getPositionRaw())
                + armFeedforward.calculate(Units.rotationsToRadians(getPositionRaw()), getVelocityRadiansPerSecond());
        publish("Motor output", output);
        wristMotor.set(output);
    }

    /**
     * Gets the position of the arm.
     * 
     * @return The angle in degrees from vertical, with positive forward.
     */
    public double getPositionDegrees() {
        return (encoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public double getPositionRaw() {
        return (encoder.getAbsolutePosition().getValueAsDouble());
    }

    public State getState() {
        return currentState;
    }

    /**
     * Gets the velocity of the arm in RadiansPerSecond.
     * 
     * @return The velocity in RadiansPerSecond
     */
    public double getVelocityRadiansPerSecond() {
        return encoder.getVelocity().getValue().in(RadiansPerSecond);
    }

    /**
     * Set the angle target for the wrist.
     */
    public void setGoal(State goal) {
        currentState = goal;
        lastStateChangeTime = Timer.getFPGATimestamp();
        switch (goal) {
            case JOSTLE:
            case UP:
                pid.setGoal(State.UP.position);
                break;
            case DOWN:
                pid.setGoal(goal.position);
                break;
            default:
                break;
        }
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