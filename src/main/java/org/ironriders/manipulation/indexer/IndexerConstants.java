package org.ironriders.manipulation.indexer;

public class IndexerConstants {
    public static final int ID = 11;
    public static final double STALL_LIMIT = 40;
    
    /**
     * Possible states for the indexer. Each has a speed for the indexer.
     * <ul>
     * <li>{@link #INDEX} - Speed = -0.5, brings balls towards the launcher from the
     * intake</li>
     * <li>{@link #STOP} - Speed = 0, stops the indexer</li>
     * <li>{@link #REVERSE} - Speed = 1, runs the indexer backwards to unjam</li>
     * </ul>
     */
    public enum State {
        INDEX(-0.2),
        STOP(0),
        REVERSE(.3);

        public double speed;

        private State(double speed) {
            this.speed = speed;
        }
    }

}
