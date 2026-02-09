package org.ironriders.manipulation.intake;

public class IntakeConstants {
    public static final int ID = 8;

    public static final int INTAKE_MOTOR_STALL_LIMIT = 40;

    /**
     * Possible states for the intake. Each has a speed for the intake.
     * <ul>
     * <li>{@link #INTAKE}: Intakes balls; speed = -1</li>
     * <li>{@link #STOP}: Stops the intake; speed = 0</li>
     * <li>{@link #BACK}: Ejects balls; speed = 1</li>
     * </ul>
     */
    public enum State {
        INTAKE(-1),
        STOP(0),
        BACK(1);

        public double speed;

        private State(double speed) {
            this.speed = speed;
        }
    }
}
