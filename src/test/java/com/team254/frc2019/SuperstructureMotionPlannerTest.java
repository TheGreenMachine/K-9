package com.team254.frc2019;

import com.team1816.frc2019.states.SuperstructureMotionPlanner;
import com.team1816.frc2019.states.SuperstructureState;
import com.team1816.frc2019.subsystems.CargoShooter;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;


public class SuperstructureMotionPlannerTest {

    SuperstructureMotionPlanner planner;
    SuperstructureState desiredState;
    SuperstructureState simulatedState;

    @Before
    public void initializeVariables() {
        planner = new SuperstructureMotionPlanner();
        desiredState = new SuperstructureState();
        simulatedState = new SuperstructureState();
    }

    @Test
    public void collectorDownTest() {

       SuperstructureState currentState = new SuperstructureState();

        desiredState.armPosition = CargoShooter.ARM_POSITION_UP;
        desiredState.isCollectorDown = true;

        simulatedState.armPosition = CargoShooter.ARM_POSITION_DOWN;
        simulatedState.isCollectorDown = false;

        planner.setDesiredState(desiredState, simulatedState);

        SuperstructureState commandedState = planner.update(currentState);
        while (!planner.isFinished(currentState)) {
            commandedState = planner.update(currentState);
            System.out.println(commandedState);
            currentState = new SuperstructureState(commandedState.armPosition, commandedState.isCollectorDown);
            System.out.println("commanded state collector down: " + commandedState.isCollectorDown +
                "commanded state arm position: " + commandedState.armPosition);
        }

        assertEquals("Expected shooterPosition to be within 30 ticks of 2059", desiredState.armPosition, commandedState.armPosition, 30);
        assertEquals("Expected collectorDown true", desiredState.isCollectorDown, commandedState.isCollectorDown);
    }
}
