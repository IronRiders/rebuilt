package org.ironriders.manipulation.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static org.ironriders.lib.BallisticsUtils.calculateAngleToHub;
import static org.ironriders.lib.BallisticsUtils.calculateAngleToInternalTarget;
import static org.ironriders.lib.BallisticsUtils.calculateAngleToTarget;
import static org.ironriders.lib.BallisticsUtils.estimateMinMaxRange;
import static org.ironriders.lib.BallisticsUtils.getPosition;
import static org.ironriders.lib.BallisticsUtils.calculateOptimalAngles;

import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_D;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_I;
import static org.ironriders.manipulation.launcher.LauncherConstants.FLYWHEEL_MAX_ACC;
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

import java.util.Optional;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.manipulation.launcher.LauncherConstants.State;
import org.ironriders.manipulation.launcher.LauncherConstants.TargetingMode;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LauncherSubsystem extends IronSubsystem {
    private LauncherCommands commands;

    public static State currentState = State.STOW;

    public static TargetingMode targetingMode = TargetingMode.OUT_OF_RANGE;

    public static Pose3d currentTarget = FieldPositions.get(ElementType.HUB);

    public static double[] range;

    // Motors
    public final TalonFX flyWheelMotor = new TalonFX(999); // TODO set the actual CAN ID
    public final TalonFX launcherHoodMotor = new TalonFX(898); // TODO set the actual CAN ID

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

    public double manualAnglePosition = LAUNCHER_STOW_POSITION;
    public double manualFlywheelVelocity = 0;

    public LauncherSubsystem() {
        commands = new LauncherCommands(this);

        flyWheelMotor.getConfigurator().apply(currentLimitsConfigs);
        flyWheelMotor.setNeutralMode(NeutralModeValue.Coast);

        launcherHoodMotor.getConfigurator().apply(currentLimitsConfigs);
        launcherHoodMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(1));

        velocityPidController.reset();
        velocityPidController.setSetpoint(0);
        velocityPidController.setTolerance(FLYWHEEL_TOLERANCE);

        anglePidController.reset(getLauncherHoodAngle().in(Degrees));
        anglePidController.setGoal(LAUNCHER_STOW_POSITION);
        anglePidController.setTolerance(LAUNCHER_TOLERANCE);

        for (double i = 0; i <= 15; i += 1) {
            DogLog.log("Launcher-test",
                    String.valueOf(i) + " | Rad: "
                            + calculateAngleToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP), i)
                                    .in(Radians)
                            + " Deg: "
                            + calculateAngleToTarget(FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP), i)
                                    .in(Degrees));
        }
        DogLog.log("Launcher-test-pose", FieldPositions.get(ElementType.HUB).toString());
        DogLog.log("Pose-convert", FieldPositions.prepareInchesPose(FieldPositions.Hub.HUB_TOP).toString());
        DogLog.log("Launcher-our-pose", getPosition().toString());
        DogLog.log("Launcher-real-test", String.valueOf(calculateAngleToHub().in(Degrees)));
        DogLog.log("Launcher-range", String.valueOf(estimateMinMaxRange().get()[0]) + " | "
                + String.valueOf(estimateMinMaxRange().get()[1]));

        double[] a = calculateOptimalAngles();
        DogLog.log("Optimal", "Range: (" + String.valueOf(a[0]) + ", " + String.valueOf(a[1]) + ") Min: " + String.valueOf(a[2]) + " Max: " + String.valueOf(a[3]));

        setCurrentState(State.IDLE);

        //Optional<double[]> _range = estimateMinMaxRange();
        //if (_range.isPresent()) {
          //  range = _range.get();
       // }
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

    public LauncherCommands getCommands() {
        return commands;
    }

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

    public void setTarget(Pose3d target) {
        currentTarget = target;
    }

    public void setTarget(Pose2d target) {
        currentTarget = Utils.expandPose2d(target);
    }

    public boolean isReady() {
        return velocityPidController.atSetpoint() && anglePidController.atGoal();
    }

    public void updatePID() {
        publish("Launcher RPM", getFlywheelVelocity().in(RPM));
        publish("Launcher Differential RPM", flyWheelMotor.getDifferentialAverageVelocity().getValue().in(RPM));

        flyWheelMotor.set(
                Utils.clamp(0, 1,
                        velocityPidController.calculate(getFlywheelVelocity().in(DegreesPerSecond))));
        launcherHoodMotor.set(anglePidController.calculate(getLauncherHoodAngle().in(Degrees)));
    }

    public void setFlywheelGoal(double goalVelocity) {
        velocityPidController.setSetpoint(goalVelocity);
    }

    public void setLauncherGoal(double goalAngle) {
        anglePidController.setGoal(goalAngle);
    }

    public AngularVelocity getFlywheelVelocity() {
        return flyWheelMotor.getVelocity().getValue();
    }

    public Angle getLauncherHoodAngle() {
        return launcherHoodMotor.getPosition().getValue();
    }

    public double getManualLauncherAngle(){
        return SmartDashboard.getNumber("manualLauncherAngle", manualAnglePosition);
    }

    public double getmanualFlywheelyVelocity(){
        return SmartDashboard.getNumber("manualFlywheelVelocity", manualFlywheelVelocity);
    }

    public void homeLauncherHood() {
    }
}
