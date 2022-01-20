package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.paths.TrajectorySet;

public class LivingRoomMode extends AutoModeBase {

    private final DriveTrajectory mDriveTrajectory;

    public LivingRoomMode() {
        var trajectory = TrajectorySet.LIVING_ROOM;
        mDriveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Living Room Mode");
        runAction(new WaitAction(.5));
        runAction(mDriveTrajectory);
    }
}
