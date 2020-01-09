package com.team1816.frc2019.states;


import com.team1816.frc2019.subsystems.CargoShooter;
import com.team254.lib.util.Util;

public class SuperstructureStateManager {
    public enum WantedAction {
        IDLE,
        GO_TO_POSITION
    }
    
    public enum SubsystemState {
        WANTED_POSITION,
        MOVING_TO_POSITION
    }

    private SubsystemState systemState = SubsystemState.WANTED_POSITION;

    private SuperstructureMotionPlanner planner = new SuperstructureMotionPlanner();

    private SuperstructureCommand command = new SuperstructureCommand();
    private SuperstructureState commandedState = new SuperstructureState();
    private SuperstructureState desiredEndState = new SuperstructureState();

    private CargoShooter.ArmPosition ROCKET_POSITION = CargoShooter.ArmPosition.ROCKET;
    private CargoShooter.ArmPosition CARGO_POSITION = CargoShooter.ArmPosition.UP;
    private CargoShooter.ArmPosition INTAKE_POSITION = CargoShooter.ArmPosition.DOWN;

    private int armPosition;
    private boolean isCollectorDown;

    public boolean scoringPositionChanged() {
        return !Util.epsilonEquals(desiredEndState.armPosition, armPosition) ||
            !(desiredEndState.isCollectorDown == isCollectorDown);
    }

    public synchronized SuperstructureCommand update(double timestamp, WantedAction wantedAction,
                                                     SuperstructureState currentState) {
        synchronized (SuperstructureStateManager.this) {
            SubsystemState newState;

            switch (wantedAction) {
                case IDLE:
                    newState = handleHoldingTransitions(currentState);
                    break;
                case GO_TO_POSITION:
                    newState = handleMovingToPositionTransitions(currentState);
                    break;
                default:
                    System.out.println("major bruh alert: " + systemState);
                    newState = systemState;
                    break;
            }

            if (newState != systemState) {
                System.out.println(timestamp +": Superstructure changed state: " + systemState + " -> " + newState);
                systemState = newState;
            }

            commandedState = planner.update(currentState);
            command.armPosition = Util.limit(commandedState.armPosition,
                CargoShooter.ARM_POSITION_UP, CargoShooter.ARM_POSITION_DOWN);;
            command.collectorDown = commandedState.isCollectorDown;

            return command;
        }
    }

    private void updateMotionPlannerDesired(SuperstructureState currentState) {
        desiredEndState.armPosition = armPosition;
        desiredEndState.isCollectorDown = isCollectorDown;

        System.out.println("Setting motion planner to armPosition: " + desiredEndState.armPosition
            + " collectorDown: " + desiredEndState.isCollectorDown);

        // Push into elevator planner.
        if (!planner.setDesiredState(desiredEndState, currentState)) {
            System.out.println("Unable to set elevator planner!");
        }

        armPosition = desiredEndState.armPosition;
        isCollectorDown = desiredEndState.isCollectorDown;
    }

    private SubsystemState handleHoldingTransitions(SuperstructureState currentState) {
        if (scoringPositionChanged()) {
            updateMotionPlannerDesired(currentState);
        } else if (planner.isFinished(currentState)) {
            return SubsystemState.WANTED_POSITION;
        }
        return SubsystemState.MOVING_TO_POSITION;
    }

    // MOVING_TO_POSITION
    private SubsystemState handleMovingToPositionTransitions(SuperstructureState currentState) {
        if (systemState == SubsystemState.MOVING_TO_POSITION && planner.isFinished(currentState)) {
            return SubsystemState.MOVING_TO_POSITION;
        } else {
            return SubsystemState.WANTED_POSITION;
        }
    }

    public SubsystemState getSubsystemState() {
        return systemState;
    }
}



