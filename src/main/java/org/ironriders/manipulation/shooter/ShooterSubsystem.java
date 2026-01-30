package org.ironriders.manipulation.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static org.ironriders.manipulation.shooter.ShooterConstants.ESTIMATION_STARTING_DISTANCE;
import static org.ironriders.manipulation.shooter.ShooterConstants.FLYWHEEL_D;
import static org.ironriders.manipulation.shooter.ShooterConstants.FLYWHEEL_I;
import static org.ironriders.manipulation.shooter.ShooterConstants.FLYWHEEL_MAX_ACC;
import static org.ironriders.manipulation.shooter.ShooterConstants.FLYWHEEL_MAX_VEL;
import static org.ironriders.manipulation.shooter.ShooterConstants.FLYWHEEL_P;
import static org.ironriders.manipulation.shooter.ShooterConstants.FLYWHEEL_TOLERANCE;
import static org.ironriders.manipulation.shooter.ShooterConstants.G;
import static org.ironriders.manipulation.shooter.ShooterConstants.HEIGHT_DIFFERENCE_HUB_TO_SHOOTER;
import static org.ironriders.manipulation.shooter.ShooterConstants.MAX_ROTATION;
import static org.ironriders.manipulation.shooter.ShooterConstants.MIN_ROTATION;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_D;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_HOOD_MAX_ACC;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_HOOD_MAX_VEL;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_I;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_MAX_RANGE;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_P;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_STOW_POSITION;
import static org.ironriders.manipulation.shooter.ShooterConstants.SHOOTER_TOLERANCE;
import static org.ironriders.manipulation.shooter.ShooterConstants.TARGET_BALL_VELOCITY;

import java.util.Optional;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.manipulation.shooter.ShooterConstants.State;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterSubsystem extends IronSubsystem {
    private ShooterCommands commands;

    public State currentState = State.STOW;

    public boolean inRange = false;

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

    public final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(40);

    public ShooterSubsystem() {
        commands = new ShooterCommands(this);

        flyWheelMotor.getConfigurator().apply(currentLimitsConfigs);
        flyWheelMotor.setNeutralMode(NeutralModeValue.Coast);

        shooterHoodMotor.getConfigurator().apply(currentLimitsConfigs);
        shooterHoodMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(1));

        velocityPidController.reset(getFlywheelVelocity().in(DegreesPerSecond));
        velocityPidController.setGoal(0);
        velocityPidController.setTolerance(FLYWHEEL_TOLERANCE);

        anglePidController.reset(getShooterHoodAngle().in(Degrees));
        anglePidController.setGoal(0);
        anglePidController.setTolerance(SHOOTER_TOLERANCE);

        for (double i = 0; i <= 15; i += 0.5) {
            DogLog.log("Shooter-test", i + " | Rad: " + String.valueOf(calculateShooterAngle(i).in(Radians)) + " Deg: "
                    + String.valueOf(calculateShooterAngle(i).in(Degrees)));
        }
        DogLog.log("Shooter-test-pose", FieldPositions.get(ElementType.HUB).toString());
        DogLog.log("Shooter-our-pose", DriveSubsystem.getSwerveDrive().getPose().toString());
        DogLog.log("Shooter-real-test", String.valueOf(calculateShooterAngle(calculateDistanceToHub()).in(Degrees)));
        DogLog.log("Shooter-max-range", estimateMaxRange().get());

        setCurrentState(State.IDLE);

        Optional<Double> range = estimateMaxRange();

        if (range.isPresent()) {
            SHOOTER_MAX_RANGE = range.get();
        }
    }

    @Override
    public void periodic() {
        updatePID();

        if (calculateDistanceToHub() < SHOOTER_MAX_RANGE && !inRange) {
            inRange = true;
            setCurrentState(State.READY);
        } else if (inRange && calculateDistanceToHub() > SHOOTER_MAX_RANGE) {
            inRange = false;
            setCurrentState(State.IDLE);
        }
    }

    public void setCurrentState(State state) {
        currentState = state;

        switch (currentState) {
            default: // If we don't recognize the state fall through to STOW
            case STOW:
                setFlywheelGoal(0);
                setAngleGoal(SHOOTER_STOW_POSITION);
                break;

            case IDLE:
                setFlywheelGoal(FLYWHEEL_MAX_VEL / 2);
                setAngleGoal(calculateShooterAngle(calculateDistanceToHub()).in(Degrees));
                break;

            case READY:
                setFlywheelGoal(FLYWHEEL_MAX_VEL);
                setAngleGoal(calculateShooterAngle(calculateDistanceToHub()).in(Degrees));
                break;
        }

    }

    public double calculateDistanceToHub() {
        return Utils
                .getPoseDifference(DriveSubsystem.getSwerveDrive().getPose(),
                        Utils.flattenPose3d(FieldPositions.get(ElementType.HUB)))
                .getNorm();
    }

    public ShooterCommands getCommands() {
        return commands;
    }

    public void updatePID() {
        publish("Shooter RPM", getFlywheelVelocity().in(RPM));
        publish("Shooter Differential RPM", flyWheelMotor.getDifferentialAverageVelocity().getValue().in(RPM));

        flyWheelMotor
                .set(Utils.clamp(0, 1, velocityPidController.calculate(getFlywheelVelocity().in(DegreesPerSecond)))); // Don't
                                                                                                                      // brake
        shooterHoodMotor.set(anglePidController.calculate(getShooterHoodAngle().in(Degrees)));
    }

    public Angle getShooterHoodAngle() {
        return shooterHoodMotor.getPosition().getValue();
    }

    public void homeShooterHood() {

    }

    public Optional<Double> estimateMaxRange() { // Try to find the maximum range from the provided ball speed.
        double low = ESTIMATION_STARTING_DISTANCE;
        double high = ESTIMATION_STARTING_DISTANCE;
        double step = 1d;
        double loops = 0d;

        while (loops <= 25) {
            Double result = calculateShooterAngle(high).in(Radians);

            if (result == -1) {
                break;
            }

            low = high; // Last known good (or too low)
            high += step;
            step *= 2;
            loops += 1;
        }

        if (loops > 25) {
            return Optional.empty();
        }

        for (int i = 0; i < 50; i++) {
            double mid = (low + high) / 2.0;
            Double result = calculateShooterAngle(mid).in(Radians);

            if (result == -1) {
                high = mid;
            } else {
                low = mid;
            }
        }

        return Optional.of(low);
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

    public Angle calculateShooterAngle(double distance) {
        double v = TARGET_BALL_VELOCITY;
        double y = HEIGHT_DIFFERENCE_HUB_TO_SHOOTER;

        double inner = Math.pow(v, 4) - G * (G * distance * distance + 2 * y * v * v);

        if (inner < 0) {
            return Angle.ofBaseUnits(-1d, Degrees); // Target unreachable
        }

        double sqrt = Math.sqrt(inner);

        double lowAngle = Math.atan((v * v - sqrt) / (G * distance));
        double highAngle = Math.atan((v * v + sqrt) / (G * distance));

        DogLog.log("Shooter-angles-test", "High: " + String.valueOf(highAngle) + " | Low: " + String.valueOf(lowAngle));

        if (Utils.inRange(MIN_ROTATION, MAX_ROTATION, highAngle)) {
            return Angle.ofBaseUnits(highAngle, Radians);
        } else if (Utils.inRange(MIN_ROTATION, MAX_ROTATION, lowAngle)) {
            return Angle.ofBaseUnits(lowAngle, Radians);
        } else {
            return Angle.ofBaseUnits(-2d, Degrees); // Bad angle
        }
    }
}
