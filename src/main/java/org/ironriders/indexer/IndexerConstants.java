package org.ironriders.indexer;

public class IndexerConstants {
    public static int ID = 0x3a33;
    public static double STALL_LIMIT = 40;

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
