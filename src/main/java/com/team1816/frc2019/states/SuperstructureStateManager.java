package com.team1816.frc2019.states;


import com.team1816.frc2019.subsystems.CargoShooter;

public class SuperstructureStateManager {
    public enum WantedAction {
        IDLE,
        GO_TO_POSITION,
        WANT_MANUAL,
    }
    
    public enum SubsystemState {
        WANTED_POSITION,
        MOVING_TO_POSITION,
        MANUAL
    }

    private SubsystemState systemState = SubsystemState.WANTED_POSITION;

    private SuperstructureMotionPlanner planner = new SuperstructureMotionPlanner();

    private SuperstructureCommand mCommand = new SuperstructureCommand();
    private SuperstructureState mCommandedState = new SuperstructureState();
    private SuperstructureState mDesiredEndState = new SuperstructureState();

    private CargoShooter.ArmPosition ROCKET_POSITION = CargoShooter.ArmPosition.ROCKET;
    private CargoShooter.ArmPosition CARGO_POSITION = CargoShooter.ArmPosition.UP;
    private CargoShooter.ArmPosition INTAKE_POSITION = CargoShooter.ArmPosition.DOWN;


    // TODO: Set cargo collector and shooter positions as one state
    //  Superstructure class should update the "state" of the cargo collector and shooter (looking at the subsystems holistically)

    
    public synchronized SuperstructureCommand update(double timestamp, SuperstructureState currentState) {
        synchronized (SuperstructureStateManager.this) {
            SubsystemState newState;
            
            switch (systemState) {
                case WANTED_POSITION: 
                    //
                case MOVING_TO_POSITION:
                    //
                case MANUAL:
                    //
            }

        }
        
        return mCommand;
    }

    private void updateMotionPlannerDesired(SuperstructureState currentState) {
        mDesiredEndState.armPosition = armPosition;
        mDesiredEndState.isCollectorDown = collectorDown;

        System.out.println("Setting motion planner to height: " + mDesiredEndState.height
            + " angle: " + mDesiredEndState.angle);

        // Push into elevator planner.
        if (!planner.setDesiredState(mDesiredEndState, currentState)) {
            System.out.println("Unable to set elevator planner!");
        }

        armPosition = mDesiredEndState.angle;
        collectorDown = mDesiredEndState.height;
    }

    private SubsystemState handleDefaultTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if (wantedAction == WantedAction.GO_TO_POSITION) {
            if (scoringPositionChanged()) {
                updateMotionPlannerDesired(currentState);
            } else if (mPlanner.isFinished(currentState)) {
                return SubsystemState.WANTED_POSITION;
            }
            return SubsystemState.MOVING_TO_POSITION;
        } else if (wantedAction == WantedAction.WANT_MANUAL) {
            return SubsystemState.MANUAL;
        } else {
            if (mSubsystemState == SubsystemState.MOVING_TO_POSITION && !mPlanner.isFinished(currentState)) {
                return SubsystemState.MOVING_TO_POSITION;
            } else {
                return SubsystemState.WANTED_POSITION;
            }
        }
    }

    private SubsystemState handleHoldingPositionTransitions(WantedAction wantedAction,
                                                         SuperstructureState currentState) {
        return handleDefaultTransitions(wantedAction, currentState);
    }

    private void setHoldingPositionCommandedState() {

    }

    // MOVING_TO_POSITION
    private SubsystemState handleMovingToPositionTransitions(WantedAction wantedAction,
                                                          SuperstructureState currentState) {

        return handleDefaultTransitions(wantedAction, currentState);
    }

    private void setMovingToPositionCommandedState() {

    }

    // MANUAL
    private SubsystemState handleManualTransitions(WantedAction wantedAction,
                                                SuperstructureState currentState) {
        if (wantedAction != WantedAction.WANT_MANUAL) {
            // Freeze height.
            mScoringAngle = currentState.angle;
            mScoringHeight = currentState.height;
            return handleDefaultTransitions(WantedAction.GO_TO_POSITION, currentState);
        }
        return handleDefaultTransitions(wantedAction, currentState);
    }

    private void setManualCommandedState() {

    }

    public SubsystemState getSubsystemState() {
        return systemState;
    }



}



