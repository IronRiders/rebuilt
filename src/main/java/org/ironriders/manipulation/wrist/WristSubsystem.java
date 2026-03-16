package org.ironriders.manipulation.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import org.ironriders.lib.IronSubsystem;
import org.ironriders.manipulation.wrist.WristConstants.State;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.Timer;

/**
 * Subsystem for controlling wrist
 */
public class WristSubsystem extends IronSubsystem {

    /* MOTOR COMPONENTS */
    private final TalonFX wristMotor = new TalonFX(
            WristConstants.MOTOR_ID);

    private State currentState = State.DOWN;
    private double lastStateChangeTime = 0;

    private final WristCommands commands;

    // private double jostlingIterator = 0;
    // private Double jostleMin = State.JOSTLE.position -
    // WristConstants.JOSTLE_RANGE;
    // private Double jostleMax = State.JOSTLE.position +
    // WristConstants.JOSTLE_RANGE;

    public WristSubsystem() {
        commands = new WristCommands(this);

        var slot = new Slot0Configs();
        slot.kP = WristConstants.P;
        slot.kI = WristConstants.I;
        slot.kD = WristConstants.D;
        slot.kS = WristConstants.S;
        slot.kV = WristConstants.V;
        slot.kG = WristConstants.G;
        slot.GravityType = GravityTypeValue.Arm_Cosine;
        wristMotor.getConfigurator()
                .apply(slot);

        setGoal(currentState);
    }

    @Override
    public void periodic() {
        publish("rotations", wristMotor.getPosition().getValue().in(Rotations) * WristConstants.MECHANISM_RATIO);
        publish("pos raw", getPositionRaw());
        publish("pid err", wristMotor.getClosedLoopError());
        publish("pid out", wristMotor.getClosedLoopOutput());
        publish("supply current", wristMotor.getSupplyCurrent().getValueAsDouble());
        publish("state", currentState.name());
        publish("goal (angle)", wristMotor.getClosedLoopReference());

        switch (currentState) {
            case JOSTLE:
                double timeDifference = Timer.getFPGATimestamp() - lastStateChangeTime;
                if (timeDifference > 2.5) {
                    lastStateChangeTime = Timer.getFPGATimestamp();
                    break;
                } else if (timeDifference > 1.5) {
                    wristMotor.setControl(State.DOWN.posvol);
                    break;
                } else if (timeDifference > 1) {
                    wristMotor.setControl(State.UP.posvol);
                    break;
                } else if (timeDifference > .5) {
                    wristMotor.setControl(State.DOWN.posvol);
                    break;
                } else {
                    wristMotor.setControl(State.UP.posvol);
                }
            default:
                break;
        }
        publish("Motor output", wristMotor.getClosedLoopOutput());
    }

    /**
     * Gets the position of the arm.
     * 
     * @return The angle in degrees from vertical, with positive forward.
     */
    public double getPositionDegrees() {
        return (wristMotor.getPosition().getValue().in(Degrees) * WristConstants.MECHANISM_RATIO);
    }

    public double getPositionRaw() {
        return (wristMotor.getPosition().getValueAsDouble());
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
        return wristMotor.getVelocity().getValue().in(RadiansPerSecond);
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
                wristMotor.setControl(State.UP.posvol);
                break;
            case DOWN:
                wristMotor.setControl(State.DOWN.posvol);
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
        return wristMotor.getClosedLoopError().getValue() < WristConstants.JOSTLE_TOLERANCE;
    }

    /**
     * @return The {@link WristCommands commands} for this subsystem.
     */
    public WristCommands getCommands() {
        return commands;
    }
}