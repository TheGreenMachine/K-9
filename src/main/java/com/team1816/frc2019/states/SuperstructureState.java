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

    public SuperstructureState(SuperstructureState other) {
        this.armPosition = other.armPosition;
    }

    public boolean inIllegalZone(boolean allowSmallErrors) {
        int allowableArmPositionError = allowSmallErrors ? 30 : 0;
        int minAllowablePosition = CargoShooter.ARM_POSITION_DOWN - allowableArmPositionError;
        int maxAllowablePosition = CargoShooter.ARM_POSITION_DOWN + allowableArmPositionError;

        return !isCollectorDown
            && (armPosition >= minAllowablePosition)
            && (armPosition <= maxAllowablePosition);
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
