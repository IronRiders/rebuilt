package org.ironriders.intake;

import static org.ironriders.intake.IntakeConstants.INTAKE_MOTOR_LEFT;
import static org.ironriders.intake.IntakeConstants.INTAKE_MOTOR_LEFT_INVERSION;
import static org.ironriders.intake.IntakeConstants.INTAKE_MOTOR_RIGHT;
import static org.ironriders.intake.IntakeConstants.INTAKE_MOTOR_RIGHT_INVERSION;
import static org.ironriders.intake.IntakeConstants.INTAKE_MOTOR_ROLLER_INVERSION;
import static org.ironriders.intake.IntakeConstants.INTAKE_MOTOR_TOP;
import static org.ironriders.intake.IntakeConstants.INTAKE_NEUTRAL_MODE;
import static org.ironriders.intake.IntakeConstants.INTAKE_STATOR_CURRENT;
import static org.ironriders.intake.IntakeConstants.INTAKE_SUPPLY_CURRENT;
import static org.ironriders.intake.IntakeConstants.INTAKE_SUPPLY_CURRENT_LOWER_LIMIT;
import static org.ironriders.intake.IntakeConstants.INTAKE_SUPPLY_CURRENT_LOWER_TIME;
import static org.ironriders.intake.IntakeConstants.LEFT_SPEED_MUL;
import static org.ironriders.intake.IntakeConstants.RIGHT_SPEED_MUL;
import static org.ironriders.intake.IntakeConstants.ROLLER_SPEED_MUL;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import org.ironriders.intake.IntakeConstants.IntakeState;
import org.ironriders.lib.IronSubsystem;

public class IntakeSubsystem extends IronSubsystem {

    private final IntakeCommands commands;

    private final TalonFX rightIntake = new TalonFX(INTAKE_MOTOR_RIGHT);
    private final TalonFX leftIntake = new TalonFX(INTAKE_MOTOR_LEFT);
    private final TalonFX rollerIntake = new TalonFX(INTAKE_MOTOR_TOP);

    private double average = 0;

    // private final DigitalInput beamBreak = new DigitalInput(INTAKE_BEAMBREAK);
    // Broken

    public IntakeSubsystem() {
        TalonFXConfiguration mainConfig = new TalonFXConfiguration();
        mainConfig
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimit(INTAKE_STATOR_CURRENT)
                                .withSupplyCurrentLimit(INTAKE_SUPPLY_CURRENT)
                                .withSupplyCurrentLowerLimit(INTAKE_SUPPLY_CURRENT_LOWER_LIMIT)
                                .withSupplyCurrentLowerTime(INTAKE_SUPPLY_CURRENT_LOWER_TIME))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(INTAKE_NEUTRAL_MODE));

        // TODO: This is ugly as hell
        leftIntake.getConfigurator().apply(mainConfig);
        leftIntake
                .getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(INTAKE_MOTOR_LEFT_INVERSION));

        rightIntake.getConfigurator().apply(mainConfig);
        rightIntake
                .getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(INTAKE_MOTOR_RIGHT_INVERSION));

        rollerIntake.getConfigurator().apply(mainConfig);
        rollerIntake
                .getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(INTAKE_MOTOR_ROLLER_INVERSION));

        commands = new IntakeCommands(this);
    }

    @Override
    public void periodic() {
        debugPublish("Left Velocity", leftIntake.getVelocity().getValue().in(Units.DegreesPerSecond));
        debugPublish("Right Velocity", rightIntake.getVelocity().getValue().in(Units.DegreesPerSecond));
        debugPublish("Beam Break Triggered", beamBreakTriggered());
        average = (leftIntake.getTorqueCurrent().getValueAsDouble()
                + rightIntake.getTorqueCurrent().getValueAsDouble())
                / 2f;
        debugPublish("Current average", average);
    }

    public void set(IntakeState state) {
        debugPublish("Intake State", state.toString());
        // logMessage("goes to " + state.toString());
        leftIntake.set(state.speed * outputDifferential(state, LEFT_SPEED_MUL));
        rightIntake.set(state.speed * outputDifferential(state, RIGHT_SPEED_MUL));
        rollerIntake.set(state.speed * outputDifferential(state, ROLLER_SPEED_MUL));
    }

    public double outputDifferential(IntakeState state, double controlSpeedMultipler) {
        if (state.toString().equals(IntakeState.GRAB.toString())) {
            return controlSpeedMultipler;
        }
        return 1;
    }

    public void setMotorsNoDiff(double speed) {
        leftIntake.set(speed);
        rightIntake.set(speed);
        rollerIntake.set(speed);
    }

    public boolean hasHighCurrent() {
        return false;
        // return average > 12 && !beamBreak.get(); disabled because beam break made it
        // hard to intake
    }

    public boolean beamBreakTriggered() {
        return false;
    }

    public IntakeCommands getCommands() {
        return commands;
    }
}
