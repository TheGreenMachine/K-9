package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.Constants;
import com.team1816.season.paths.DriveStraight;

import javax.inject.Singleton;

@Singleton
public class TuneDrivetrainMode extends AutoModeBase {

    public TuneDrivetrainMode(){
        var ds = new DriveStraight(Constants.StartingPose, 90).generateTrajectory();
        trajectory = new TrajectoryAction(ds);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Tune Drivetrain path");
        runAction(trajectory);
        var ds = new DriveStraight(-84).generateTrajectory();
        trajectory = new TrajectoryAction(ds);
        runAction(new WaitAction(1));
        runAction(trajectory);
        ds = new DriveStraight(84).generateTrajectory();
        trajectory = new TrajectoryAction(ds);
        runAction(new WaitAction(1));
        runAction(trajectory);
    }
}
