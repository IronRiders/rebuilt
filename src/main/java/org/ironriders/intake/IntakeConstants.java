package org.ironriders.intake;

public class IntakeConstants {
    public static final int ID = 676767;

    public static final int INTAKE_MOTOR_STALL_LIMIT = 40;
    public static final double EJECT_WAIT_TIME = 1;
    public static final double CENTER_TIMEOUT = 1;

    public enum State {
        INTAKE(-0.5),
        STOP(0),
        BACK(1);

        public double speed;

        private State(double speed) {
            this.speed = speed;
        }
    }
}
