package org.ironriders.climb;

public class ClimbConstants {

    public static final int CLIMBER_MOTOR_ID = 17;
    public static final int CURRENT_LIMIT = 40;

    public static final double MAX_ACC = 200;
    public static final double MAX_VEL = 200;

    public static final double ENCODER_SCALE = (1f / 100f); // `f` here signifies that the numbers are floats

    public static double P = 0.05; // proportion
    public static double I = 0; // integral
    public static double D = 0; // derivative
    public static double T = 0.02; // time to next step

    public static double TOLERANCE = 0.005;

    public enum Targets {
        MIN(0),
        MAX(-500), // max position upward
        CLIMBED(-60);

        public final double pos;

        Targets(double pos) {
            this.pos = pos;
        }
    }
}
