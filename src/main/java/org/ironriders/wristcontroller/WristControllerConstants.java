package org.ironriders.wristcontroller;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class WristControllerConstants {
    public static final int WristMotorCantID = 0b1011011000011;
    public static final double WristMotorSupplyCurrentLimit = 40.0;
    public static final double PIDProportional = 0.0; //TODO: Test
    public static final double PIDIntegral = 0.0;
    public static final double PIDDerivative = 0.0;
    public static final TrapezoidProfile.Constraints Constraints = null;
    public enum WristControllerPositions {
        UP(0.0), //TODO: Set these later
        DOWN(0.0);

        public final double position;

        WristControllerPositions(double position) {
            this.position = position;
        }
    }
}
