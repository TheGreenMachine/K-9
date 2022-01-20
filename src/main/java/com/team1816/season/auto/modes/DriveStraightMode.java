package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.paths.TrajectorySet;

public class DriveStraightMode extends AutoModeBase {

    private final DriveTrajectory mDriveStraight;

    public DriveStraightMode() {
        mDriveStraight = new DriveTrajectory(TrajectorySet.DRIVE_STRAIGHT, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Drive Straight Mode");
        runAction(new WaitAction(.5));
        runAction(mDriveStraight);
    }
}
