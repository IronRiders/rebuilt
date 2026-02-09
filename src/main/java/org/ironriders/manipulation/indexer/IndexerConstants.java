package org.ironriders.manipulation.indexer;

public class IndexerConstants {
    public static int ID = 7;
    public static double STALL_LIMIT = 40;

    /**
     * Possible states for the indexer. Each has a speed for the indexer.
     * <ul>
     * <li>{@link #INDEX} - Speed = -0.5, brings balls towards the shooter from the
     * intake</li>
     * <li>{@link #STOP} - Speed = 0, stops the indexer</li>
     * <li>{@link #BACK} - Speed = 1, runs the indexer backwards to unjam</li>
     * </ul>
     */
    public enum State {
        INDEX(-0.5),
        STOP(0),
        BACK(1);

        public double speed;

        private State(double speed) {
            this.speed = speed;
        }
    }

}
