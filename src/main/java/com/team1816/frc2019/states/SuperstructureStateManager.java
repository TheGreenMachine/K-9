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

    private int armPosition = CargoShooter.getInstance().getArmEncoderPosition();
    private boolean isCollectorDown;

    public boolean scoringPositionChanged() {
        var scoringPositionChanged = !Util.epsilonEquals(desiredEndState.armPosition, armPosition) ||
            desiredEndState.isCollectorDown != isCollectorDown;

//        System.out.println("######################### SuperstructureStateManager::scoringPositionChanged() ############################");
//        System.out.println("---> scoringPositionChanged: " + scoringPositionChanged);
//        System.out.println("---> desiredEndState.armPosition: " + desiredEndState.armPosition);
//        System.out.println("---> armPosition: " + armPosition);
//        System.out.println("---> !Util.epsilonEquals(dES.aP, aP): " + !Util.epsilonEquals(desiredEndState.armPosition, armPosition));
//        System.out.println("---> dES.isCollectorDown != isCollectorDown: " + (desiredEndState.isCollectorDown != isCollectorDown));
//        System.out.println("####################################################################################");

        return scoringPositionChanged;
    }

    public synchronized SuperstructureCommand update(double timestamp, WantedAction wantedAction,
                                                     SuperstructureState currentState) {
        synchronized (SuperstructureStateManager.this) {
            SubsystemState newState;

            switch (systemState) {
                case WANTED_POSITION:
                    newState = handleDefaultTransitions(wantedAction, currentState);
                    System.out.println("State in IDLE:" + newState);
                    break;
                case MOVING_TO_POSITION:
                    newState = handleDefaultTransitions(wantedAction, currentState);
                    System.out.println("State in GO_TO_POSITION:" + newState);
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
            + " || collectorDown: " + desiredEndState.isCollectorDown);

        var desiredStateReturnValue = planner.setDesiredState(desiredEndState, currentState);
        System.out.println("SuperstructureStateManager::updateMotionPlannerDesired() -> desiredStateReturnValue: "
            + desiredStateReturnValue);
        // Push into elevator planner.
        if (!desiredStateReturnValue) {
            System.out.println("Unable to set cargo shooter/collector planner!");
        }

        armPosition = desiredEndState.armPosition;
        isCollectorDown = desiredEndState.isCollectorDown;
    }

    public synchronized void setArmPosition(int armPosition) {
        this.armPosition = armPosition;
    }

    public int getArmPosition() {
        return armPosition;
    }

    public synchronized void setCollectorDown(boolean collectorDown) {
        isCollectorDown = collectorDown;
    }

    public boolean isCollectorDown() {
        return isCollectorDown;
    }


    private SubsystemState handleDefaultTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if (wantedAction == WantedAction.GO_TO_POSITION) {
            if (scoringPositionChanged()) {
                updateMotionPlannerDesired(currentState);
            } else if (planner.isFinished(currentState)) {
                return SubsystemState.WANTED_POSITION;
            }
            return SubsystemState.MOVING_TO_POSITION;
        } else {
            if (systemState == SubsystemState.MOVING_TO_POSITION && !planner.isFinished(currentState)) {
                return SubsystemState.MOVING_TO_POSITION;
            } else {
                return SubsystemState.WANTED_POSITION;
            }
        }
    }

    // WANTED_POSITION / IDLE
    private SubsystemState handleHoldingTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if (wantedAction == WantedAction.GO_TO_POSITION) {
            if (scoringPositionChanged()) {
                updateMotionPlannerDesired(currentState);
            } else if (planner.isFinished(currentState)) {
                return SubsystemState.WANTED_POSITION;
            }
        }
        return SubsystemState.MOVING_TO_POSITION;
    }

    // MOVING_TO_POSITION
    private SubsystemState handleMovingToPositionTransitions(SuperstructureState currentState) {
        if (systemState == SubsystemState.MOVING_TO_POSITION  && !planner.isFinished(currentState)) {
            return SubsystemState.MOVING_TO_POSITION;
        } else {
            return SubsystemState.WANTED_POSITION;
        }
    }

    public SubsystemState getSubsystemState() {
        return systemState;
    }
}



