package org.ironriders.manipulation.intake;

public class IntakeConstants {
    public static final int ID = 12;

    public static final int INTAKE_MOTOR_STALL_LIMIT = 60; // If this isn't enough we might need to switch motors

    /**
     * Possible states for the intake. Each has a speed for the intake.
     * <ul>
     * <li>{@link #INTAKE}: Intakes balls; speed = -0.7</li>
     * <li>{@link #STOP}: Stops the intake; speed = 0</li>
     * <li>{@link #BACK}: Ejects balls; speed = 0.4</li>
     * </ul>
     */
    public enum State {
        INTAKE(-0.5),
        STOP(0),
        BACK(0.25);

        public double speed;

        private State(double speed) {
            this.speed = speed;
        }
    }
}
