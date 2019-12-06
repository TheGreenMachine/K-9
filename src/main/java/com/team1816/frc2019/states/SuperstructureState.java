package com.team1816.frc2019.states;

import com.team1816.frc2019.subsystems.CargoShooter;

public class SuperstructureState {
    public int armPosition;
    public boolean collectorDown;

    public SuperstructureState(int armPosition, boolean collectorDown) {
        this.armPosition = armPosition;
        this.collectorDown = collectorDown;
    }
//    maxPos: 4027
//    midPos: 3230
//    minPos: 3015

    public SuperstructureState() {
        this(CargoShooter.ARM_POSITION_UP, false);
    }

    public boolean inIllegalZone(boolean allowSmallErrors) {
        int allowableArmPositionError = allowSmallErrors ? 30 : 0;
        int minAllowablePosition = CargoShooter.ARM_POSITION_DOWN - allowableArmPositionError;
        int maxAllowablePosition = CargoShooter.ARM_POSITION_DOWN + allowableArmPositionError;

        return !collectorDown
            && (armPosition >= minAllowablePosition)
            && (armPosition <= maxAllowablePosition);
    }

    @Override
    public String toString() {
        return
            "SuperstructureState {"
                + "  armPosition = " + armPosition
                + "  collectorDown = " + collectorDown
                + "}";
    }
}
