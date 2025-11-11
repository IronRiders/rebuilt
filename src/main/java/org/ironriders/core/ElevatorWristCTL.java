package org.ironriders.core;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironriders.elevator.ElevatorCommands;
import org.ironriders.elevator.ElevatorSubsystem;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.wrist.WristCommands;
import org.ironriders.wrist.WristSubsystem;

// This class contains all the state for the moving the elevator and wrist together. You should not
// call the wrist or elevator commands independently
public class ElevatorWristCTL extends IronSubsystem {
    private ElevatorWristState currentState = ElevatorWristState.HOLD;

    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final WristCommands wristCommands = wristSubsystem.getCommands();

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ElevatorCommands elevatorCommands = elevatorSubsystem.getCommands();

    private final String diagnosticName = this.getClass().getSimpleName();
    private final String dashboardPrefix = "Subsystems/" + diagnosticName + "/";

    /**
     * Publishes commands to set wrist to various positions to SmartDashboard;
     * registers {@linkplain
     * #reset() Elevator Wrist Reset} with {@linkplain
     * com.pathplanner.lib.auto.NamedCommands#registerCommand(String, Command)
     * PathPlanner}
     */
    public ElevatorWristCTL() {
        debugPublish("Set to STOW", setElevatorWrist(ElevatorWristState.HOLD));
        debugPublish("Set to INTAKING", setElevatorWrist(ElevatorWristState.INTAKING));
        debugPublish("Set to L2", setElevatorWrist(ElevatorWristState.L2));
        debugPublish("Set to L3", setElevatorWrist(ElevatorWristState.L3));
        debugPublish("Set to L4", setElevatorWrist(ElevatorWristState.L4));

        SmartDashboard.putData("debug/" + dashboardPrefix + "Reset", reset());
        NamedCommands.registerCommand("Elevator Wrist Reset", (Command) reset());

        SmartDashboard.putString(dashboardPrefix + "Current State", currentState.toString());
    }

    /** position targets for elevator, all in inches. */
    public enum ElevatorLevel { // Position in inches
        DOWN(0),
        L2(19.5),
        L3(36.5),
        L4(53);

        public final double pos;

        ElevatorLevel(double pos) {
            this.pos = pos;
        }
    }

    /** angle targets for wrist, in degrees. */
    public enum WristRotation { // Position in degrees (zero is straight up)
        HOLD(0),
        INTAKING(-85),
        L2L3(40),
        L4(15);

        public final double pos;

        WristRotation(double pos) {
            this.pos = pos;
        }
    }

    /**
     * Combined targets for elevator and wrist, each with a wrist and elevetor
     * state.
     *
     * <ul>
     * <li>{@linkplain org.ironriders.core.ElevatorWristCTL.ElevatorWristState#HOLD
     * HOLD}:
     * {@linkplain org.ironriders.core.ElevatorWristCTL.ElevatorLevel#DOWN Elevator:
     * Down},
     * {@linkplain org.ironriders.core.ElevatorWristCTL.WristRotation#HOLD Wrist:
     * Hold}
     * <li>{@linkplain org.ironriders.core.ElevatorWristCTL.ElevatorWristState#INTAKING
     * INTAKING}:
     * {@linkplain org.ironriders.core.ElevatorWristCTL.ElevatorLevel#DOWN Elevator:
     * Down},
     * {@linkplain org.ironriders.core.ElevatorWristCTL.WristRotation#INTAKING
     * Wrist: Intaking}
     * <li>{@linkplain org.ironriders.core.ElevatorWristCTL.ElevatorWristState#L2
     * L2}: {@linkplain
     * org.ironriders.core.ElevatorWristCTL.ElevatorLevel#L2 Elevator: L2},
     * {@linkplain
     * org.ironriders.core.ElevatorWristCTL.WristRotation#L2L3 Wrist: L2/L3}
     * <li>{@linkplain org.ironriders.core.ElevatorWristCTL.ElevatorWristState#L3
     * L3}: {@linkplain
     * org.ironriders.core.ElevatorWristCTL.ElevatorLevel#L3 Elevator: L3},
     * {@linkplain
     * org.ironriders.core.ElevatorWristCTL.WristRotation#L2L3 Wrist: L2/L3}
     * <li>{@linkplain org.ironriders.core.ElevatorWristCTL.ElevatorWristState#L4
     * L4}: {@linkplain
     * org.ironriders.core.ElevatorWristCTL.ElevatorLevel#L4 Elevator: L4},
     * {@linkplain
     * org.ironriders.core.ElevatorWristCTL.WristRotation#L4 Wrist: L4}
     * </ul>
     */
    public enum ElevatorWristState {
        HOLD(ElevatorLevel.DOWN, WristRotation.HOLD),
        INTAKING(ElevatorLevel.DOWN, WristRotation.INTAKING),
        L2(ElevatorLevel.L2, WristRotation.L2L3),
        L3(ElevatorLevel.L3, WristRotation.L2L3),
        L4(ElevatorLevel.L4, WristRotation.L4);

        public final ElevatorLevel elevatorLevel;
        public final WristRotation wristRotation;

        ElevatorWristState(ElevatorLevel elevatorLevel, WristRotation wristRotation) {
            this.elevatorLevel = elevatorLevel;
            this.wristRotation = wristRotation;
        }
    }

    public double getWristRotation() {
        return wristSubsystem.getCurrentAngle();
    }

    public double getElevatorHight() {
        return elevatorSubsystem.getHeight();
    }

    public ElevatorSubsystem getElevatorSubsystem() {
        return elevatorSubsystem;
    }

    public WristSubsystem getWristSubsystem() {
        return wristSubsystem;
    }

    /*
     * This command sets both a elevator position and a wrist position.
     */

    public Command setElevatorWrist(ElevatorWristState state) {
        return Commands.sequence(
                wristCommands.set(WristRotation.HOLD),
                elevatorCommands.set(state.elevatorLevel),
                wristCommands.set(state.wristRotation));
    }

    /*
     * This command, in parallel, moves the wrist all the way in and does a PID
     * reset, as well as moving the elevator all the way down, rehoming it for good
     * measure, and then resetting it's PID.
     */

    public Command reset() {
        currentState = ElevatorWristState.HOLD;
        SmartDashboard.putString(dashboardPrefix + "Current State", currentState.toString());
        return Commands.sequence(
                logMessage("reseting"), wristCommands.stowReset(), elevatorCommands.home());
    }
}
