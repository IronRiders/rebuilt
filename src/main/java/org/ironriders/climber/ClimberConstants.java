package org.ironriders.climber;

/** Constants for the climber subsystem. */
public class ClimberConstants {
    public static final int PRIMARY_ID = 6767;
    public static final int SECONDARY_ID = 676789989;

    public static final int STALL_LIMIT = 40;

    public static final double P = 0.5;
    public static final double I = 0.0;
    public static final double D = 0.1;

    public static final double MAX_VEL = 180;
    public static final double MAX_ACC = 180;

    public static final double TORQUE_CURRENT_SPIKE_THRESHOLD = 10; //amps

    public static final double HOME_SPEED = 0.4;

    public enum State {
        CLIMBED(40),
        MIN(0),
        MAX(100);

        public double position;

        private State(double position) {
            this.position = position;
        }
    }
}
