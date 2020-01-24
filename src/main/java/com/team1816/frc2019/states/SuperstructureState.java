package com.team1816.frc2019.states;

import com.team1816.frc2019.subsystems.CargoShooter;
import com.team254.lib.util.Util;

public class SuperstructureState {
    public int armPosition;
    public boolean isCollectorDown;

    public SuperstructureState(int armPosition, boolean isCollectorDown) {
        this.armPosition = armPosition;
        this.isCollectorDown = isCollectorDown;
    }
//    maxPos: 4027
//    midPos: 3230
//    minPos: 3015

    public SuperstructureState() {
        this(CargoShooter.ARM_POSITION_UP, false);
    }

    public boolean inIllegalZone(boolean allowSmallErrors) {
        int allowableArmPositionError = allowSmallErrors ? 30 : 0;
        // Highest position to be considered "down".
        int minDownPosition = ((CargoShooter.ARM_POSITION_MID + CargoShooter.ARM_POSITION_DOWN) / 2) - allowableArmPositionError;
        // Highest allowable position.
        int minAllowablePosition = CargoShooter.ARM_POSITION_UP - allowableArmPositionError;
        // Lowest allowable position.
        int maxAllowablePosition = CargoShooter.ARM_POSITION_DOWN + allowableArmPositionError;


        return (
            (
                // Illegal if collector is up and arm is down.
                !isCollectorDown
                && (armPosition >= minDownPosition)
            )
            || (armPosition <= minAllowablePosition) // Illegal if arm is above the highest position.
            || (armPosition >= maxAllowablePosition) // Illegal if arm is below the lowest position.
        );
    }

    public boolean isInRange(SuperstructureState otherState, int armPositionThreshold) {
        return Util.epsilonEquals(otherState.armPosition, armPosition, armPositionThreshold);

    }

    @Override
    public String toString() {
        return
            "SuperstructureState {"
                + "  armPosition = " + armPosition
                + "  collectorDown = " + isCollectorDown
                + "}";
    }
}
