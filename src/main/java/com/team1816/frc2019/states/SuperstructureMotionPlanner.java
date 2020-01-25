package com.team1816.frc2019.states;

import com.team1816.frc2019.subsystems.CargoShooter;
import com.team254.lib.util.Util;

import java.util.LinkedList;
import java.util.Optional;

public class SuperstructureMotionPlanner {

    static class SubCommand {
        public SubCommand(SuperstructureState endState) {
            mEndState = endState;
        }

        public SuperstructureState mEndState;
        public static final int armPositionThreshold = 30;

        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState, armPositionThreshold);
        }
    }

    static class WaitForCollectingSubCommand extends SubCommand {
        public WaitForCollectingSubCommand(SuperstructureState endState) {
            super(endState);
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return super.isFinished(currentState) && currentState.isCollectorDown;
        }
    }

    static class WaitForEndCollectingSubCommand extends SubCommand {
        public WaitForEndCollectingSubCommand(SuperstructureState endState) {
            super(endState);
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return super.isFinished(currentState) && !currentState.isCollectorDown;
        }
    }

    protected SuperstructureState mCommandedState = new SuperstructureState();
    protected SuperstructureState mIntermediateCommandState = new SuperstructureState();
    protected LinkedList<SubCommand> mCommandQueue = new LinkedList<>();
    protected Optional<SubCommand> mCurrentCommand = Optional.empty();


    public synchronized boolean setDesiredState(SuperstructureState desiredState, SuperstructureState currentState) {

        // Limit illegal inputs.
        desiredState.armPosition = Util.limit(desiredState.armPosition,
                CargoShooter.ARM_POSITION_UP, CargoShooter.ARM_POSITION_DOWN);

        if (desiredState.inIllegalZone(true)) {
            System.err.println("Desired State in Illegal Zone!");
            return false;
        }

        mCommandQueue.clear();

        mCommandQueue.add(new SubCommand(desiredState));


        if (
            (desiredState.armPosition > CargoShooter.ARM_POSITION_MID)
            || (currentState.armPosition > CargoShooter.ARM_POSITION_MID)
        ) {
            // Target or current below mid position - arm will be moving through collector box
            // Ensure collector down
            mCommandQueue.add(new WaitForCollectingSubCommand(desiredState));
            System.out.println("Queuing WaitForCollectingSubCommand - arm will be moving through collector box");
        } else if (
            (desiredState.armPosition <= CargoShooter.ARM_POSITION_MID)
        ) {
            // Lift collector if target position above or equal to mid position
            mCommandQueue.add(new WaitForEndCollectingSubCommand(desiredState));
            System.out.println("Queuing WaitForENDCollectingSubCommand - arm will be above collector box");
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
        if (mCurrentCommand.isEmpty() && !mCommandQueue.isEmpty()) {
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

        mCommandedState.armPosition =
            Util.limit(mIntermediateCommandState.armPosition,
                CargoShooter.ARM_POSITION_UP, CargoShooter.ARM_POSITION_DOWN);
        mCommandedState.isCollectorDown = mIntermediateCommandState.isCollectorDown;

        return mCommandedState;
    }
}
