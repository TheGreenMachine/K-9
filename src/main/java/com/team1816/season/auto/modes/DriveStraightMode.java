package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.paths.TrajectorySet;

public class DriveStraightMode extends AutoModeBase {

    private DriveTrajectory mDriveStraight;

    public DriveStraightMode() {
        var trajectory = TrajectorySet.getInstance().DRIVE_STRAIGHT;
        mDriveStraight = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Drive Straight Mode");
        runAction(new WaitAction(.5));
        runAction(mDriveStraight);
    }
}
