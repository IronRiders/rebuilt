package org.ironriders.manipulation.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static org.ironriders.manipulation.shooter.ShooterConstants.FLYWHEEL_D;
import static org.ironriders.manipulation.shooter.ShooterConstants.FLYWHEEL_I;
import static org.ironriders.manipulation.shooter.ShooterConstants.FLYWHEEL_MAX_ACC;
import static org.ironriders.manipulation.shooter.ShooterConstants.FLYWHEEL_MAX_VEL;
import static org.ironriders.manipulation.shooter.ShooterConstants.FLYWHEEL_P;
import static org.ironriders.manipulation.shooter.ShooterConstants.G;
import static org.ironriders.manipulation.shooter.ShooterConstants.HEIGHT_DIFFERENCE_HUB_TO_SHOOTER;
import static org.ironriders.manipulation.shooter.ShooterConstants.MAX_ROTATION;
import static org.ironriders.manipulation.shooter.ShooterConstants.MIN_ROTATION;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_D;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_HOOD_MAX_ACC;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_HOOD_MAX_VEL;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_I;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_P;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_STOW_POSITION;
import static org.ironriders.manipulation.shooter.ShooterConstants.TARGET_BALL_VELOCITY;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement;
import org.ironriders.manipulation.shooter.ShooterConstants.State;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterSubsystem extends IronSubsystem {
    private ShooterCommands commands;

    public State currentState = State.STOW;

    public final TalonFX flyWheelMotor = new TalonFX(999); // TODO set the actual CAN ID

    public final TalonFX shooterHoodMotor = new TalonFX(898); // TODO set the actual CAN ID

    public final TrapezoidProfile.Constraints flyWheelConstraints = new TrapezoidProfile.Constraints(FLYWHEEL_MAX_VEL,
            FLYWHEEL_MAX_ACC);
    public final ProfiledPIDController velocityPidController = new ProfiledPIDController(FLYWHEEL_P, FLYWHEEL_I,
            FLYWHEEL_D, flyWheelConstraints);

    public final TrapezoidProfile.Constraints AngleConstraints = new TrapezoidProfile.Constraints(SHOOTER_HOOD_MAX_VEL,
            SHOOTER_HOOD_MAX_ACC);
    public final ProfiledPIDController anglePidController = new ProfiledPIDController(SHOOTER_P, SHOOTER_I, SHOOTER_D,
            AngleConstraints);

    public final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(40);

    public ShooterSubsystem() {
        commands = new ShooterCommands(this);

        flyWheelMotor.getConfigurator().apply(currentLimitsConfigs);

        shooterHoodMotor.getConfigurator().apply(currentLimitsConfigs);
        shooterHoodMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(1));

        velocityPidController.reset(getFlywheelVelocity().in(DegreesPerSecond));
        velocityPidController.setGoal(0);

        anglePidController.reset(getShooterHoodAngle().in(Degrees));
        // TODO: Maybe we should spin up the wheel and never stop it? idk sounds kinda
        // like a bad idea.
        anglePidController.setGoal(0);

        for (double i = 0; i <= 15; i++) {
            DogLog.log("Shooter-test", i + " : " + calculateShooterAngle(i).toString());
            DogLog.log("Shooter-test", calculateDistanceToHub());
        }
    }

    @Override
    public void periodic() {
        switch (currentState) {
            default: // If we don't recognize the state fall through to STOW
            case STOW:
                setFlywheelGoal(0);
                setAngleGoal(SHOOTER_STOW_POSITION);
                break;
            case READY:
                setFlywheelGoal(FLYWHEEL_MAX_VEL);
                setAngleGoal(calculateShooterAngle(calculateDistanceToHub()));
                break;
        }

        updatePID();
    }

    public void setCurrentState(State state) {
        currentState = state;
    }

    public double calculateDistanceToHub() {
        return Utils
                .getPoseDifference(DriveSubsystem.getSwerveDrive().getPose(), Utils.flattenPose3d(FieldElement
                        .nearestTo(DriveSubsystem.getSwerveDrive().getPose(), FieldElement.ElementType.HUB).get().pose))
                .getNorm();
    }

    public ShooterCommands getCommands() {
        return commands;
    }

    public void updatePID() {
        publish("Shooter RPM", getFlywheelVelocity().in(RPM));
        publish("Shooter Differential RPM", flyWheelMotor.getDifferentialAverageVelocity().getValue().in(RPM));

        flyWheelMotor.set(velocityPidController.calculate(getFlywheelVelocity().in(DegreesPerSecond)));
        shooterHoodMotor.set(anglePidController.calculate(getShooterHoodAngle().in(Degrees)));
    }

    public Angle getShooterHoodAngle() {
        return shooterHoodMotor.getPosition().getValue();
    }

    public void homeShooterHood() {

    }

    public void setFlywheelGoal(double goal) {
        velocityPidController.setGoal(goal);
    }

    public void setAngleGoal(double goal) {
        anglePidController.setGoal(goal);
    }

    public AngularVelocity getFlywheelVelocity() {
        return flyWheelMotor.getVelocity().getValue();
    }

    public Double calculateShooterAngle(double distance) {
        double v = TARGET_BALL_VELOCITY;
        double y = HEIGHT_DIFFERENCE_HUB_TO_SHOOTER;

        double inner = Math.pow(v, 4) - G * (G * distance * distance + 2 * y * v * v);

        if (inner < 0) {
            return 0d; // Target unreachable
        }

        double sqrt = Math.sqrt(inner);

        double lowAngle = Units.radiansToDegrees(Math.atan((v * v - sqrt) / (G * distance)));
        double highAngle = Units.radiansToDegrees(Math.atan((v * v + sqrt) / (G * distance)));

        if (Utils.inRange(MIN_ROTATION, MAX_ROTATION, lowAngle)) {
            return lowAngle;
        } else if (Utils.inRange(MIN_ROTATION, MAX_ROTATION, highAngle)) {
            return highAngle;
        } else {
            return 0d; // Bad angle
        }
    }

}
