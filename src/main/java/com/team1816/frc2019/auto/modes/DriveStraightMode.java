package com.team1816.frc2019.auto.modes;

import com.team1816.frc2019.paths.TrajectoryGenerator;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class DriveStraightMode extends AutoModeBase{

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private DriveTrajectory mDriveStraight;

    public DriveStraightMode() {
        var trajectory = mTrajectoryGenerator.getTrajectorySet().driveStraight;
        mDriveStraight = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Drive Straight Mode");;
        runAction(new WaitAction(.5));
        runAction(mDriveStraight);
    }
}

