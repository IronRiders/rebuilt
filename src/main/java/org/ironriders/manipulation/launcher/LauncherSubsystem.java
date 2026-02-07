package org.ironriders.manipulation.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.ironriders.lib.BallisticsUtils.calculateAngleToInternalTarget;
import static org.ironriders.lib.BallisticsUtils.calculateAngleToTarget;
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

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.manipulation.launcher.LauncherConstants.State;
import org.ironriders.manipulation.launcher.LauncherConstants.TargetingMode;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

    public static TargetingMode targetingMode = TargetingMode.OUT_OF_RANGE;

    public static Pose3d currentTarget = FieldPositions.get(ElementType.HUB);

    public static double[] range;

    // Motors
    public final List<TalonFX> flyWheelMotors = List.of(new TalonFX(9990), new TalonFX(9991), new TalonFX(9992)); // IDs
    public final List<Servo> launcherHoodActuators = List.of(new Servo(0), new Servo(1));

    // PID Controllers
    public final PIDController velocityPidController = new PIDController(FLYWHEEL_P, FLYWHEEL_I,
            FLYWHEEL_D);

    public final TrapezoidProfile.Constraints AngleConstraints = new TrapezoidProfile.Constraints(LAUNCHER_HOOD_MAX_VEL,
            LAUNCHER_HOOD_MAX_ACC);
    public final ProfiledPIDController anglePidController = new ProfiledPIDController(LAUNCHER_P, LAUNCHER_I,
            LAUNCHER_D,
            AngleConstraints);

    public final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(40);

    public final TalonFXConfiguration configuration = new TalonFXConfiguration()
            .withCurrentLimits(currentLimitsConfigs);

    public double manualAnglePosition = LAUNCHER_STOW_POSITION;
    public double manualFlywheelVelocity = 0;

    public LauncherSubsystem() {
        commands = new LauncherCommands(this);

        flyWheelMotors.stream().forEach(this::configureFlywheelMotor);

        launcherHoodActuators.stream().forEach(this::configureServo);

        velocityPidController.reset();
        velocityPidController.setSetpoint(0);
        velocityPidController.setTolerance(FLYWHEEL_TOLERANCE);

        anglePidController.reset(getLauncherHoodAngle().in(Degrees));
        anglePidController.setGoal(LAUNCHER_STOW_POSITION);
        anglePidController.setTolerance(LAUNCHER_TOLERANCE);

        for (double i = 0; i <= 15; i += 1) {
            DogLog.log("Launcher/Launcher-test",
                    String.valueOf(i) + " | Radius: "
                            + calculateAngleToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP), i)
                                    .in(Radians)
                            + " Degrees: "
                            + calculateAngleToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP), i)
                                    .in(Degrees));
        }

        setCurrentState(State.IDLE);

        Optional<double[]> _range = estimateMinMaxRange();
        if (_range.isPresent()) {
            range = _range.get();
        }

        DogLog.log("Launcher/Range", "(" + String.valueOf(range[0]) + " : " + String.valueOf(range[1] + ")"));
    }

    public void configureFlywheelMotor(TalonFX motor) {
        motor.getConfigurator().apply(configuration);
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void configureServo(Servo servo) {
        servo.enableDeadbandElimination(true);
    }

    @Override
    public void periodic() {
        DriveSubsystem.getSwerveDrive().field.getObject("LauncherTarget").setPose(Utils.flattenPose3d(currentTarget));

        switch (currentState) {
            case READY:
            case IDLE:
                setLauncherGoal(calculateAngleToTarget(currentTarget).in(Degrees));
                break;
            default:
                break;
        }

        updatePID();
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
                setLauncherGoal(LAUNCHER_STOW_POSITION);
                return;

            case IDLE:
                setFlywheelGoal(FLYWHEEL_MAX_VEL / 2);

            case READY:
                setFlywheelGoal(FLYWHEEL_MAX_VEL);
        }

        setLauncherGoal(calculateAngleToInternalTarget().in(Degrees));
    }

    /**
     * Sets the launcher's {@link Pose3d target} for shooter angle
     * calculations.
     * 
     * @param target The target for targeting the shooter.
     */
    public void setTarget(Pose3d target) {
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
     * Checks if the launcher is ready to shoot (if shooter & flywheel are at their
     * target and setpoint, respectively).
     * 
     * @return True if the launcher is ready, false otherwise.
     */
    public boolean isReady() {
        return velocityPidController.atSetpoint() && anglePidController.atGoal();
    }

    /**
     * Updates the flywheel and launcher hood motor outputs using the PID
     * controllers. Utility class for {@link #periodic()}.
     */
    public void updatePID() {
        publish("Launcher RPM", getFlywheelVelocity().in(RPM));
        publish("Launcher Differential RPM", flyWheelMotors.stream().map(TalonFX::getDifferentialAverageVelocity)
                .map(StatusSignal::getValueAsDouble).collect(Collectors.toList()).toString());

        flyWheelMotors.stream().forEach(this::setFlywheelMotors);
        setHoodAngle(Angle.ofBaseUnits(anglePidController.calculate(getLauncherHoodAngle().in(Degrees)), Degrees));
    }

    public void setFlywheelMotors(TalonFX motor) {
        motor.set(
                Utils.clamp(0, 1,
                        velocityPidController.calculate(getFlywheelVelocity().in(DegreesPerSecond))));
    }

    /**
     * Sets the flywheel's target velocity for the PID controller. See
     * {@link PIDController#setSetpoint setSetpoint} for more information.
     */
    public void setFlywheelGoal(double goalVelocity) {
        velocityPidController.setSetpoint(goalVelocity);
    }

    /**
     * Sets the launcher's target angle for the PID controller. See
     * {@link PIDController#setGoal setGoal} for more information.
     */
    public void setLauncherGoal(double goalAngle) {
        anglePidController.setGoal(goalAngle);
    }

    /*
     * @return *AVERAGE* velocity!!
     */
    public AngularVelocity getFlywheelVelocity() {
        return AngularVelocity.ofBaseUnits(flyWheelMotors.stream().map(TalonFX::getVelocity)
                .collect(Collectors.averagingDouble(StatusSignal::getValueAsDouble)), RotationsPerSecond);
    }

    /**
     * @return The current angle of the launcher hood.
     */
    public Angle getLauncherHoodAngle() {
        return Angle.ofBaseUnits(
                LauncherMaps.AngleToExtensionMap.getAngleForExtension(launcherHoodActuators.stream().map(Servo::get)
                        .collect(Collectors.averagingDouble(num -> Double.valueOf(num)))),
                Degrees);
    }

    public void setHoodAngle(Angle angle) {
        setServos(LauncherMaps.AngleToExtensionMap.getExtensionForAngle(angle.in(Degrees)));
    }

    public void setServos(double amount) {
        launcherHoodActuators.stream().forEach((Servo servo) -> {
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
     * Homes the launcher hood to its default position.
     */
    public void homeLauncherHood() {
        // TODO: implement
    }
}
