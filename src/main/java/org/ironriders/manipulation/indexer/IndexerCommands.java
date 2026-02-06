package org.ironriders.manipulation.indexer;

import org.ironriders.manipulation.indexer.IndexerConstants.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Commands for the {@link IndexerSubsystem indexer subsystem}. Has only one
 * method: {@linkplain IndexerCommands#set() set}, which sets the indexer's
 * target state.
 */
public class IndexerCommands {
    private final IndexerSubsystem indexer;

    public IndexerCommands(IndexerSubsystem indexer) {
        this.indexer = indexer;

    }

    /**
     * See
     * {@link org.ironriders.manipulation.indexer.IndexerSubsystem#setState(State
     * state) IndexerSubsystem's setState}
     * 
     * @param state {@linkplain org.ironriders.manipulation.indexer.IndexerSubsystem
     *              Indexer } target
     *              {org.ironriders.manipulation.indexer.IndexerConstants.State
     *              state}.
     * @return A command to set the indexer's target state
     */
    public Command set(State state) {
        return Commands.runOnce(() -> indexer.setState(state));
    }
}
