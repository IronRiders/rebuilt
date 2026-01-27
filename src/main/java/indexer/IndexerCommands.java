package indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import indexer.IndexerConstants.State;

public class IndexerCommands {
    private final IndexerSubsystem indexer;

    public IndexerCommands(IndexerSubsystem indexer) {
        this.indexer = indexer;

    }

    public Command set(State state) {
        return Commands.runOnce(() -> indexer.setState(state));
    }
}

