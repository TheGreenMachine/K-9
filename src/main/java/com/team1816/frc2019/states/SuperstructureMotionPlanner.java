package com.team1816.frc2019.states;

import com.team254.lib.util.Util;

import java.util.LinkedList;
import java.util.Optional;

public class SuperstructureMotionPlanner {

    class SubCommand {
        public SubCommand(SuperstructureState endState) {
            mEndState = endState;
        }

        public SuperstructureState mEndState;
        public int armPositionThreshold = 30;


        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState, armPositionThreshold);
        }
    }

    protected SuperstructureState mCommandedState = new SuperstructureState();
    protected SuperstructureState mIntermediateCommandState = new SuperstructureState();
    protected LinkedList<SubCommand> mCommandQueue = new LinkedList<>();
    protected Optional<SubCommand> mCurrentCommand = Optional.empty();


    public synchronized boolean setDesiredState(SuperstructureState desiredStateIn, SuperstructureState currentState) {
        SuperstructureState desiredState = new SuperstructureState(desiredStateIn);

        if (desiredState.inIllegalZone(true)) { return false;
        }
        mCommandQueue.clear();

        //TODO: Checks to see the position of the cargo shooter and the cargo collector
        // Create two classes (load down) and (raise up)

        final boolean longUpwardsMove = desiredState.height - currentState.height > SuperstructureConstants
            .kElevatorLongRaiseDistance;
        final double firstWristAngle = longUpwardsMove ? Math.min(desiredState.angle, SuperstructureConstants
            .kStowedAngle) : desiredState.angle;

        final boolean

        if (longUpwardsMove) {
            if (mUpwardsSubcommandEnabled) {
                mCommandQueue.add(new WaitForElevatorApproachingSubcommand(new SuperstructureState(desiredState.height,
                    firstWristAngle, true)));
            }
        }

        return true;
    }

    void reset(SuperstructureState currentState) {
        mIntermediateCommandState = currentState;
        mCommandQueue.clear();
        mCurrentCommand = Optional.empty();
    }

    public boolean isFinished(SuperstructureState currentState) {
        return mCurrentCommand.isPresent() && mCommandQueue.isEmpty() && currentState.wristSentLastTrajectory &&
            currentState.elevatorSentLastTrajectory;
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

        mCommandedState.angle = Util.limit(mIntermediateCommandState.angle, SuperstructureConstants.kWristMinAngle,
            SuperstructureConstants.kWristMaxAngle);
        mCommandedState.height = Util.limit(mIntermediateCommandState.height, SuperstructureConstants
            .kElevatorMinHeight, SuperstructureConstants.kElevatorMaxHeight);

        return mCommandedState;
    }
}
}
