package org.ironriders.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class WristConstants {
    public static final int WristMotorCantID = 0b1011011000011;
    public static final double WristMotorSupplyCurrentLimit = 40.0;
    public static final double PIDProportional = 0.0; //TODO: Test
    public static final double PIDIntegral = 0.0;
    public static final double PIDDerivative = 0.0;
    public static final TrapezoidProfile.Constraints Constraints = null;

    public enum WristPositions {
        UP(0.0), //TODO: Find these later
        DOWN(0.0);

        public final double position;

        WristPositions(double position) {
            this.position = position;
        }
    }

    public class FeedForward {
        public static final double kS = 0; //TODO: calculate this (https://www.reca.lc/arm)
        public static final double kV = 0;
        public static final double kG = 0;
    }
}
