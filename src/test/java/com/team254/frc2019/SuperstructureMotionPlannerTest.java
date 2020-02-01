package com.team254.frc2019;

import com.team1816.frc2019.states.SuperstructureMotionPlanner;
import com.team1816.frc2019.states.SuperstructureState;
import com.team1816.frc2019.subsystems.CargoShooter;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import static org.junit.Assert.assertEquals;


public class SuperstructureMotionPlannerTest {


    static public double shooterPositionDown = 4027;
    static public boolean collectorDown = true;


    static public double shooterPositionHigh = 3015;
    static public boolean collectorUp = false;

    static public double shooterPositionMid = 3230;

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

        desiredState.armPosition = CargoShooter.ARM_POSITION_DOWN;
        desiredState.isCollectorDown = true;

        simulatedState.armPosition = CargoShooter.ARM_POSITION_UP;
        simulatedState.isCollectorDown = false;

        planner.setDesiredState(desiredState, simulatedState);

     //   SuperstructureState commandedState = planner.update(currentState);
        for (int i = 0; i < 10; i++) {
            SuperstructureState commandedState = planner.update(currentState);
            System.out.println(commandedState);
            currentState = new SuperstructureState(commandedState.armPosition, commandedState.isCollectorDown);
            System.out.println("commanded state collector down: " + commandedState.isCollectorDown +
                "commanded state arm position: " + commandedState.armPosition);

            assertEquals("Expected shooterPosition to be within 30 ticks of 4027", shooterPositionDown, commandedState.armPosition, 30);
            assertEquals("Expected collectorDown true", collectorDown, commandedState.isCollectorDown);

        }
    }
}
