package com.team1816.frc2019.states;

import com.team1816.frc2019.subsystems.CargoShooter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;
import java.util.Optional;

public class SuperstructureMotionPlanner {

    protected static boolean isFinished;

    static class SubCommand {
        public SubCommand(SuperstructureState endState) {
            mEndState = endState;
        }

        public SubCommand() { }

        public SuperstructureState mEndState;

        public boolean isFinished(SuperstructureState currentState, int armPositionThreshold) {
            return mEndState.isInRange(currentState, armPositionThreshold);
        }

        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState);
        }

        public boolean isFinished() {
            return isFinished;
        }
    }

    static class WaitForCollectorSubCommand extends SubCommand {
        public WaitForCollectorSubCommand(boolean isCollectorDown) {
            mEndState.isCollectorDown = isCollectorDown;
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return super.isFinished(currentState);
        }
    }

    static class WaitForShooterSubCommand extends SubCommand {
        public WaitForShooterSubCommand(int armPosition) {
            mEndState.armPosition = armPosition;
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return super.isFinished(currentState, 30);
        }
    }

    static class WaitForTime extends SubCommand {
        double waitTime;
        boolean isFinished;

        public WaitForTime(double waitTime) {
            this.waitTime = waitTime;
            double start = Timer.getFPGATimestamp();
            double timeElapsed = Timer.getFPGATimestamp() - start;
            while (timeElapsed < waitTime) {
                timeElapsed = Timer.getFPGATimestamp() - start;
         //       System.out.println("Time elapsed: " + timeElapsed + "isFinished" + isFinished);
                isFinished = false;
            }
            isFinished = true;
        //    System.out.println("Time elapsed: " + timeElapsed + "isFinished" + isFinished);
        }

        @Override
        public boolean isFinished() {
            return super.isFinished();
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

        if (
            (desiredState.armPosition > CargoShooter.ARM_POSITION_MID)
            || (currentState.armPosition > CargoShooter.ARM_POSITION_MID)
        ) {
            // Target or current below mid position - arm will be moving through collector box
            // Ensure collector down
            mCommandQueue.add(new WaitForCollectorSubCommand(desiredState.isCollectorDown));
            mCommandQueue.add(new WaitForShooterSubCommand(desiredState.armPosition));
            mCommandQueue.add(new WaitForTime(1));
            System.out.println("Queuing WaitForCollectingSubCommand - arm will be moving through collector box");
        } else if (
            (desiredState.armPosition <= CargoShooter.ARM_POSITION_MID)
        ) {
            // Lift collector if target position above or equal to mid position
            mCommandQueue.add(new WaitForShooterSubCommand(desiredState.armPosition));
            mCommandQueue.add(new WaitForTime(1));
            System.out.println("Queuing WaitForENDCollectingSubCommand - arm will be above collector box");
            mCommandQueue.add(new WaitForCollectorSubCommand(desiredState.isCollectorDown));
        }

         mCurrentCommand = Optional.empty();

        return true;
    }

    public void reset(SuperstructureState currentState) {
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
            System.out.println("Currently on this command: " + mCurrentCommand);
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
        System.out.println("Commanded arm position: " + mCommandedState.armPosition);
        mCommandedState.isCollectorDown = mIntermediateCommandState.isCollectorDown;
        System.out.println("Commanded collector down: " + mCommandedState.isCollectorDown);

        return mCommandedState;
    }
}
