package org.ironriders.manipulation.launcher;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_A;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_MAX_VEL;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_P;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_S;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_TOLERANCE;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_V;

import java.util.List;
import java.util.stream.Collectors;

import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.manipulation.launcher.LauncherConstants.KickerState;
import org.ironriders.manipulation.launcher.LauncherConstants.State;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Subsystem for targeting & shooting */
public class LauncherSubsystem extends IronSubsystem {
    private LauncherCommands commands;

    public static State currentState = State.STOW;
    public static Pose3d currentTarget = FieldPositions.get(ElementType.HUB);

    public static double[] range;

    // Motors
    private final List<TalonFX> flyWheelMotors = List.of(new TalonFX(13), new TalonFX(14), new TalonFX(15)); // IDs
    private final List<Servo> launcherHoodActuators = List.of(new Servo(0), new Servo(1));
    public final TalonFX kickerMotor = new TalonFX(16);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    double targetFlywheelVelocity = 0;

    private final TalonFXConfiguration configuration = new TalonFXConfiguration();

    public double manualFlywheelVelocity = 1;
    public double manualExtensionPosition = 1;

    public static double extensionTrim = 0;

    public LauncherSubsystem() {
        commands = new LauncherCommands(this);

        kickerMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        configuration.CurrentLimits.StatorCurrentLimit = 40;

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        configuration.Slot0.kS = FLYWHEEL_S;
        configuration.Slot0.kV = FLYWHEEL_V;
        configuration.Slot0.kA = FLYWHEEL_A;

        configuration.Slot0.kP = FLYWHEEL_P;

        configuration.MotionMagic.MotionMagicCruiseVelocity = 100;
        configuration.MotionMagic.MotionMagicAcceleration = 200;

        flyWheelMotors.parallelStream().forEach(motor -> {
            motor.getConfigurator().apply(configuration);
        });

        launcherHoodActuators.parallelStream().forEach(s -> s.setBoundsMicroseconds(2000, 0, 1500, 0, 1000));

        setCurrentState(State.STOW);

        range = new double[] { LauncherConstants.MIN_RANGE, LauncherConstants.MAX_RANGE };

        homeLauncherHood();

        publish("manualFlywheelVelocity", manualFlywheelVelocity);
        publish("manualExtensionPosition", manualExtensionPosition);
    }

    @Override
    public void periodic() {
        publish("State", currentState.name());
        manualFlywheelVelocity = SmartDashboard.getNumber("manualFlywheelVelocity", manualFlywheelVelocity);
        manualExtensionPosition = SmartDashboard.getNumber("manualExtension", manualExtensionPosition);

        putPose2d(currentTarget.toPose2d(), "LauncherTarget");

        switch (currentState) {
            case READY:
            case IDLE:
                // setHoodExtension(calculateExtensionToInternalTarget());
                break;
            default:
                break;
        }

        updatePID();
    }

    public void runKicker() {
        setKicker(KickerState.FIRE);
    }

    public void stopKicker() {
        setKicker(KickerState.STOP);
    }

    public void setKicker(KickerState state) {
        kickerMotor.set(-state.speed);
    }

    public boolean isKicking() {
        return kickerMotor.get() != 0;
    }

    /**
     * Get the commands for the launcher subsystem.
     * 
     * @return The commands for the launcher subsystem.
     */
    public LauncherCommands getCommands() {
        return commands;
    }

    /**
     * Sets the {@link State state} of the launcher, which determines flywheel
     * target.
     * 
     * @param state The state to set the launcher to.
     */
    public void setCurrentState(State state) {
        currentState = state;

        switch (currentState) {
            default:
            case STOW:
                setFlywheelGoal(0);
                // setHoodExtension(LAUNCHER_STOW_POSITION);
                stopKicker();
                return;

            case IDLE:
                setFlywheelGoal(FLYWHEEL_MAX_VEL / 2);
                return;

            case READY:
                setFlywheelGoal(FLYWHEEL_MAX_VEL);
                return;
        }
    }

    /**
     * Sets the launcher's {@link Pose3d target} for shooter angle
     * calculations.
     * 
     * @param target The target for targeting the shooter.
     */
    public static void setTarget(Pose3d target) {
        currentTarget = target;
    }

    /**
     * Sets the launcher's {@link Pose2d target} for shooter angle
     * calculations. Will be converted to a {@link Pose3d} with a z value of 0.
     * 
     * @param target The target for targeting the shooter.
     */
    public void setTarget(Pose2d target) {
        currentTarget = Utils.expandPose2d(target);
    }

    /**
     * Checks if the launcher is ready to launch (if hood & flywheel are at their
     * target and setpoint, respectively).
     * 
     * @return True if the launcher is ready, false otherwise.
     */
    public boolean isReady() {
        return flyWheelMotors.parallelStream()
                .allMatch(m -> Math.abs(m.getClosedLoopError().getValueAsDouble()) < FLYWHEEL_TOLERANCE);
    }

    /**
     * Updates the flywheel and launcher hood motor outputs using the PID
     * controllers. Utility class for {@link #periodic()}.
     */
    public void updatePID() {

        publish("Launcher RPS", getFlywheelAverageVelocity().in(RevolutionsPerSecond));

        publish("Fly Wheel Velocity M1 ", flyWheelMotors.get(0).getVelocity().getValue().in(RotationsPerSecond));
        publish("Fly Wheel Velocity M2 ", flyWheelMotors.get(1).getVelocity().getValue().in(RotationsPerSecond));
        publish("Fly Wheel Velocity M3 ", flyWheelMotors.get(2).getVelocity().getValue().in(RotationsPerSecond));

        publish("PID out",
                flyWheelMotors.parallelStream()
                        .map(m -> m.getClosedLoopOutput())
                        .map(String::valueOf).collect(Collectors.joining(" | ")));

        publish("PID goal",
                flyWheelMotors.parallelStream()
                        .map(m -> m.getClosedLoopReferenceSlope())
                        .map(String::valueOf).collect(Collectors.joining(" | ")));

        if (currentState == State.STOW) {
            flyWheelMotors.parallelStream().forEach((m) -> m.set(0));
            homeLauncherHood();
            return;
        }
    }

    public void setFlywheelMotors(TalonFX motor, double velocity) {
        motor.setControl(velocityRequest.withVelocity(velocity));
    }

    public static void trim(double val) {
        extensionTrim += val;
    }

    /**
     * Sets the flywheel's target velocity for the PID controller. See
     * {@link PIDController#setSetpoint setSetpoint} for more information.
     */
    public void setFlywheelGoal(double goalVelocity) {
        targetFlywheelVelocity = goalVelocity;
        if (goalVelocity == 0) {
            flyWheelMotors.parallelStream().forEach(m -> m.set(0));
        } else {
            flyWheelMotors.parallelStream().forEach(m -> setFlywheelMotors(m, goalVelocity));
        }
    }

    /**
     * @return Returns the *AVERAGE* velocity!!
     */
    public AngularVelocity getFlywheelAverageVelocity() {
        return AngularVelocity.ofBaseUnits(flyWheelMotors.parallelStream().map(m -> m.getVelocity().getValueAsDouble())
                .collect(Collectors.averagingDouble(Double::doubleValue)), RotationsPerSecond);
    }

    /**
     * @return Returns the velocity for the given motor.
     */
    public AngularVelocity getFlywheelVelocity(TalonFX motor) {
        return AngularVelocity.ofBaseUnits(motor.getVelocity().getValueAsDouble(), RotationsPerSecond);
    }

    public void setHoodExtension(double extension) {
        publish("Set extension", extension);
        setServos(extension);
    }

    public void setServos(double amount) {
        launcherHoodActuators.forEach((Servo servo) -> {
            servo.set(amount);
        });
        publish("Last requested extension", amount);
    }

    /**
     * @return The current angle of the shooter manually set in
     *         {@link SmartDashboard#getNumber() SmartDashboard}.
     */
    public double getManualLauncherAngle() {
        return manualFlywheelVelocity;
    }

    public double getManualExtension() {
        return manualExtensionPosition;
    }

    /**
     * @return The current flywheel velocity manually set in
     *         {@link SmartDashboard#getNumber() SmartDashboard}.
     */
    public double getManualFlywheelVelocity() {
        return SmartDashboard.getNumber("manualFlywheelVelocity", manualFlywheelVelocity);
    }

    /**
     * Homes the launcher hood to it's default position.
     */
    public void homeLauncherHood() {
        setServos(0);
    }
}
