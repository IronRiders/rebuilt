package org.ironriders.manipulation.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.ironriders.lib.BallisticsUtils.calculateAngleToInternalTarget;
import static org.ironriders.lib.BallisticsUtils.estimateMinMaxRange;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_D;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_I;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_MAX_VEL;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_P;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_TOLERANCE;
import static org.ironriders.manipulation.launcher.LauncherConstants.LAUNCHER_D;
import static org.ironriders.manipulation.launcher.LauncherConstants.LAUNCHER_HOOD_MAX_ACC;
import static org.ironriders.manipulation.launcher.LauncherConstants.LAUNCHER_HOOD_MAX_VEL;
import static org.ironriders.manipulation.launcher.LauncherConstants.LAUNCHER_I;
import static org.ironriders.manipulation.launcher.LauncherConstants.LAUNCHER_P;
import static org.ironriders.manipulation.launcher.LauncherConstants.LAUNCHER_STOW_POSITION;
import static org.ironriders.manipulation.launcher.LauncherConstants.LAUNCHER_TOLERANCE;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.manipulation.launcher.LauncherConstants.KickerState;
import org.ironriders.manipulation.launcher.LauncherConstants.State;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
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
    public final List<TalonFX> flyWheelMotors = List.of(new TalonFX(13), new TalonFX(14), new TalonFX(15)); // IDs
    public final List<Servo> launcherHoodActuators = List.of(new Servo(0), new Servo(1));
    public static final TalonFX kickerMotor = new TalonFX(16);

    // PID Controllers
    public Map<TalonFX, PIDController> velocityPidMap = new HashMap<TalonFX, PIDController>();

    public final TrapezoidProfile.Constraints angleConstraints = new TrapezoidProfile.Constraints(LAUNCHER_HOOD_MAX_VEL,
            LAUNCHER_HOOD_MAX_ACC);

    public final ProfiledPIDController anglePidController = new ProfiledPIDController(LAUNCHER_P, LAUNCHER_I,
            LAUNCHER_D,
            angleConstraints);

    public final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(40);

    public final TalonFXConfiguration configuration = new TalonFXConfiguration()
            .withCurrentLimits(currentLimitsConfigs);

    public double manualAnglePosition = LAUNCHER_STOW_POSITION;
    public double manualFlywheelVelocity = 0;

    public static double angleTrim = 0;

    public LauncherSubsystem() {
        commands = new LauncherCommands(this);

        kickerMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        flyWheelMotors.parallelStream().map((TalonFX motor) -> {
            motor.getConfigurator().apply(configuration);
            motor.setNeutralMode(NeutralModeValue.Coast);
            return motor;
        })
        .forEach(m -> velocityPidMap.put(m, new PIDController(FLYWHEEL_P, FLYWHEEL_I,
            FLYWHEEL_D)));

        launcherHoodActuators.parallelStream().forEach(s -> s.enableDeadbandElimination(true));

        flyWheelMotors.parallelStream().forEach(m -> {
            PIDController v = velocityPidMap.get(m);
            v.reset();
            v.setSetpoint(0);
            v.setTolerance(FLYWHEEL_TOLERANCE);
        });

        setCurrentState(State.STOW);

        Optional<double[]> _range = estimateMinMaxRange();
        if (_range.isPresent()) {
            range = _range.get();
        }

        DogLog.log("Launcher/Range", "(" + String.valueOf(range[0]) + " : " + String.valueOf(range[1] + ")"));

        homeLauncherHood();

        anglePidController.setGoal(LAUNCHER_STOW_POSITION);
        anglePidController.setTolerance(LAUNCHER_TOLERANCE);
        anglePidController.reset(getLauncherHoodAngle().in(Degrees));
    }

    @Override
    public void periodic() {
        publish("State", currentState.name());

        putPose2d(currentTarget.toPose2d(), "LauncherTarget");

        switch (currentState) {
            case READY:
            case IDLE:
                setHoodAngleGoal(calculateAngleToInternalTarget().in(Degrees));
                break;
            default:
                break;
        }

        updatePID();
    }

    public static void runKicker() {
        setKicker(KickerState.FIRE);
    }

    public static void stopKicker() {
        setKicker(KickerState.STOP);
    }

    public static void setKicker(KickerState state) {
        kickerMotor.set(-state.speed);
    }

    public static boolean isKicking() {
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
                setHoodAngleGoal(LAUNCHER_STOW_POSITION);
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
        return flyWheelMotors.parallelStream().map(m -> velocityPidMap.get(m).atSetpoint())
                .allMatch(b -> Boolean.TRUE.equals(b)) && anglePidController.atGoal();
    }

    /**
     * Updates the flywheel and launcher hood motor outputs using the PID
     * controllers. Utility class for {@link #periodic()}.
     */
    public void updatePID() {
        publish("Launcher RPM", getFlywheelAverageVelocity().in(RPM));
        publish("Launcher Differential RPM",
                flyWheelMotors.parallelStream().map(TalonFX::getDifferentialAverageVelocity)
                        .map(StatusSignal::getValueAsDouble).collect(Collectors.toList()).toString());

        publish("PID out",
                flyWheelMotors.parallelStream()
                        .map(m -> velocityPidMap.get(m).calculate(getFlywheelVelocity(m).in(RPM)))
                        .map(String::valueOf).collect(Collectors.joining(" | ")));

        publish("PID goal",
                flyWheelMotors.parallelStream()
                        .map(m -> velocityPidMap.get(m).getSetpoint())
                        .map(String::valueOf).collect(Collectors.joining(" | ")));

        if (currentState == State.STOW) {
            flyWheelMotors.parallelStream().forEach((m) -> m.set(0));
            homeLauncherHood();
            return;
        }

        flyWheelMotors.parallelStream().forEach(this::setFlywheelMotors);

        setHoodAngle(Angle.ofBaseUnits(anglePidController.calculate(getLauncherHoodAngle().in(Degrees)) + angleTrim,
                Degrees));
    }

    public void setFlywheelMotors(TalonFX motor) {
        motor.set(
                Utils.clamp(-0.2d, 1d,
                        velocityPidMap.get(motor).calculate(getFlywheelVelocity(motor).in(RPM))));
    }

    public static void trim(double val) {
        angleTrim += val;
    }

    /**
     * Sets the flywheel's target velocity for the PID controller. See
     * {@link PIDController#setSetpoint setSetpoint} for more information.
     */
    public void setFlywheelGoal(double goalVelocity) {
        flyWheelMotors.parallelStream().forEach(m -> velocityPidMap.get(m).setSetpoint(goalVelocity));
    }

    /**
     * Sets the launcher's target angle for the PID controller. See
     * {@link PIDController#setGoal setGoal} for more information.
     */
    public void setHoodAngleGoal(double goalAngle) {
        anglePidController.setGoal(goalAngle);
    }

    /**
     * @return Returns the *AVERAGE* velocity!!
     */
    public AngularVelocity getFlywheelAverageVelocity() {
        return AngularVelocity.ofBaseUnits(flyWheelMotors.parallelStream().map(TalonFX::getVelocity)
                .collect(Collectors.averagingDouble(StatusSignal::getValueAsDouble)), RotationsPerSecond);
    }

    /**
     * @return Returns the velocity for the given motor.
     */
    public AngularVelocity getFlywheelVelocity(TalonFX motor) {
        return AngularVelocity.ofBaseUnits(motor.getVelocity().getValueAsDouble(), RotationsPerSecond);
    }

    /**
     * @return The current angle of the launcher hood.
     */
    public Angle getLauncherHoodAngle() {
        return Angle.ofBaseUnits(
                LauncherMaps.AngleToExtensionMap
                        .getAngleForExtensionPercent(launcherHoodActuators.parallelStream().map(Servo::get)
                                .collect(Collectors.averagingDouble(num -> Double.valueOf(num)))),
                Degrees);
    }

    public void setHoodAngle(Angle angle) {
        setServos(LauncherMaps.AngleToExtensionMap.getExtensionForAngle(angle.in(Degrees)));
    }

    public void setServos(double amount) {
        launcherHoodActuators.parallelStream().forEach((Servo servo) -> {
            servo.set(amount);
        });
    }

    /**
     * @return The current angle of the shooter manually set in
     *         {@link SmartDashboard#getNumber() SmartDashboard}.
     */
    public double getManualLauncherAngle() {
        return SmartDashboard.getNumber("manualLauncherAngle", manualAnglePosition);
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
        setHoodAngle(Angle.ofBaseUnits(LAUNCHER_STOW_POSITION, Degrees));
    }
}
