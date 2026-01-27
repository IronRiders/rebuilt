package org.ironriders.shooter;

import org.ironriders.lib.IronSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static org.ironriders.shooter.ShooterConstants.FLYWHEEL_MOTOR_MAX_ACCELERATION_CONSTRAINT;
import static org.ironriders.shooter.ShooterConstants.FLYWHEEL_MOTOR_MAX_VELOCITY_CONSTRAINT;
import static org.ironriders.shooter.ShooterConstants.GRAVITYCONSTANT;
import static org.ironriders.shooter.ShooterConstants.HEIGHTDIFFERENCEHUBTOSHOOTER;
import static org.ironriders.shooter.ShooterConstants.SHOOTERHOOD_MOTOR_MAX_ACCELERATION_CONSTRAINT;
import static org.ironriders.shooter.ShooterConstants.SHOOTERHOOD_MOTOR_MAX_VELOCITY_CONSTRAINT;


public class ShooterSubsystem extends IronSubsystem{
    
    private ShooterCommands commands;
    TalonFX flyWheelMotor = new TalonFX(0); //TODO set the actual CAN ID

    TalonFX shooterHoodMotor = new TalonFX(0); //TODO set the actual CAN ID

    TrapezoidProfile.Constraints flyWheelConstraints = new TrapezoidProfile.Constraints(FLYWHEEL_MOTOR_MAX_VELOCITY_CONSTRAINT, FLYWHEEL_MOTOR_MAX_ACCELERATION_CONSTRAINT);
    ProfiledPIDController velocityPidController = new ProfiledPIDController(0, 0, 0, flyWheelConstraints);

    TrapezoidProfile.Constraints AngleConstraints = new TrapezoidProfile.Constraints(SHOOTERHOOD_MOTOR_MAX_VELOCITY_CONSTRAINT, SHOOTERHOOD_MOTOR_MAX_ACCELERATION_CONSTRAINT);
    ProfiledPIDController AnglePidController = new ProfiledPIDController(0, 0, 0, AngleConstraints);
    public ShooterSubsystem (){

        //Need to add motor //TODO
        commands = new ShooterCommands(this);

        flyWheelMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(40));
        shooterHoodMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(40));
        shooterHoodMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(1));
        velocityPidController.setGoal(0);
        AnglePidController.setGoal(SHOOTERHOOD_MOTOR_MAX_VELOCITY_CONSTRAINT);
    }

    @Override
    public void periodic() {
        updatePID();
    }

    public ShooterCommands getCommands(){
        return commands;
    }

    public void updatePID(){
        publish("Shooter RPM", flyWheelMotor.getVelocity().getValue().in(RPM));
        publish("Shooter Differntial RPM", flyWheelMotor.getDifferentialAverageVelocity().getValue().in(RPM));

       
       flyWheelMotor.set(velocityPidController.calculate(flyWheelMotor.getVelocity().getValue().in(DegreesPerSecond)));

    }

    public double getShooterHoodAngle(){
        return shooterHoodMotor.getPosition().getValue().in(Degrees);
    }

    public void homeShooterHood(){
        
    }

    public double[] getShooterAngle(double distance, double flyWheelVelocity ){ //distance in meters, velocity in meters per second
        
        double discriminant = Math.sqrt(
            Math.pow(distance, 2)
            - Math.pow(GRAVITYCONSTANT, 2) * Math.pow(distance, 4) / Math.pow(flyWheelVelocity, 4)
            - 2 * HEIGHTDIFFERENCEHUBTOSHOOTER * GRAVITYCONSTANT * Math.pow(distance, 2) / Math.pow(flyWheelVelocity, 2)
        );
        double firstAngle = Math.atan((distance + discriminant) * Math.pow(flyWheelVelocity, 2) / GRAVITYCONSTANT / Math.pow(distance, 2)) ;
        double secondAngle = Math.atan((distance - discriminant) * Math.pow(flyWheelVelocity, 2) / GRAVITYCONSTANT / Math.pow(distance, 2)) ;

        double[] angles = {firstAngle,secondAngle};
        return angles;
    }


}
