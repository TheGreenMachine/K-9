package com.team1816.frc2019.states;

import java.util.LinkedList;
import java.util.Optional;

public class SuperstructureMotionPlanner {

    class SubCommand {
        public SubCommand(SuperstructureState endState) {
            mEndState = endState;
        }

        public SuperstructureState mEndState;
        public static final int armPositionThreshold = 30;

        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState, armPositionThreshold);
        }
    }

    class WaitForCollectingSubCommand extends SubCommand {
        public WaitForCollectingSubCommand(SuperstructureState endState) {
            super(endState);
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return super.isFinished(currentState) && currentState.isCollectorDown;
        }
    }

    class WaitForLoadingPositionSubCommand extends SubCommand {

    }

    protected SuperstructureState mCommandedState = new SuperstructureState();
    protected SuperstructureState mIntermediateCommandState = new SuperstructureState();
    protected LinkedList<SubCommand> mCommandQueue = new LinkedList<>();
    protected Optional<SubCommand> mCurrentCommand = Optional.empty();


    public synchronized boolean setDesiredState(SuperstructureState desiredStateIn, SuperstructureState currentState) {
        SuperstructureState desiredState = new SuperstructureState(desiredStateIn);

        if (desiredState.inIllegalZone(true)) {
            return false;
        }

        mCommandQueue.clear();

        //TODO: Checks to see the position of the cargo shooter and the cargo collector
        // Create two classes (load down) and (raise up)

        boolean longUpwardsMove =
            desiredState.armPosition - currentState.armPosition > 50; // TODO: 12/19/2019 replace 50 with constant
        boolean wantCollectorDown = !longUpwardsMove;

        if (longUpwardsMove) {
            mCommandQueue.add();
        }

        return true;
    }

    void reset(SuperstructureState currentState) {
        mIntermediateCommandState = currentState;
        mCommandQueue.clear();
        mCurrentCommand = Optional.empty();
    }

    public boolean isFinished(SuperstructureState currentState) {
        return mCurrentCommand.isPresent() && mCommandQueue.isEmpty();
    }

    public SuperstructureState update(SuperstructureState currentState) {
        if (!mCurrentCommand.isPresent() && !mCommandQueue.isEmpty()) {
            mCurrentCommand = Optional.of(mCommandQueue.remove());
        }

        if (mCurrentCommand.isPresent()) {
            SubCommand subCommand = mCurrentCommand.get();
            mIntermediateCommandState = subCommand.mEndState;
            if (subCommand.isFinished(currentState) && !mCommandQueue.isEmpty()) {
                // Let the current command persist until there is something in the queue. or not. desired outcome
                // unclear.
                mCurrentCommand = Optional.empty();
            }
        } else {
            mIntermediateCommandState = currentState;
        }

        return mCommandedState;
    }
}
}
