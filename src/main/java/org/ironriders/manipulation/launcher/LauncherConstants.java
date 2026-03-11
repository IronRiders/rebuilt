package org.ironriders.manipulation.launcher;

import static edu.wpi.first.units.Units.Rotations;

import org.ironriders.lib.field.FieldPositions;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

/** Constants for the {@link LauncherSubsystem} */
public class LauncherConstants {
    public static final double LAUNCHER_HIGHT = Units.inchesToMeters(20.0);

    public static final double G = 9.8; // m/s

    public static final double HEIGHT_DIFFERENCE_HUB_TO_LAUNCHER = Units
            .inchesToMeters(FieldPositions.Hub.HUB_TOP.getZ())
            - LAUNCHER_HIGHT; // m

    public static final double TARGET_BALL_VELOCITY = 11.4; // m/s (see https://www.reca.lc/flywheel)

    public static final double FLYWHEEL_MAX_VEL = 100; // RPS NOW (see https://www.reca.lc/flywheel)

    public static final double FLYWHEEL_TOLERANCE = 5; // rps

    public static final double FLYWHEEL_S = 0.2;
    public static final double FLYWHEEL_V = 0.118;
    public static final double FLYWHEEL_A = 0.01;

    public static final double FLYWHEEL_P = 0.1;

    public static final double LAUNCHER_P = 0.5;
    public static final double LAUNCHER_I = 0.0;
    public static final double LAUNCHER_D = 0.0;

    public static final double LAUNCHER_TOLERANCE = 0.1;

    public static final double MIN_RANGE = 0;
    public static final double MAX_RANGE = 10;

    public static final double hoodMotorOffset = 0.0;

    public static final Angle hoodBottomHardStop = Angle.ofBaseUnits(0.0, Rotations); // ONLY USE IF HARD STOP CHANGES

    public enum State {
        READY,
        IDLE,
        STOW,
        MANUAL;
    }

    public enum KickerState {
        FIRE(-1),
        STOP(0),
        EJECT(1);

        public double speed;

        KickerState(double speed) {
            this.speed = speed;
        }
    }
    public static final double INDEXER_SPEED=.5;
}
