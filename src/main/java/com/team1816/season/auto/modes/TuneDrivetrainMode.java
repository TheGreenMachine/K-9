package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.paths.TrajectorySet;

public class TuneDrivetrainMode extends AutoModeBase {

    public TuneDrivetrainMode() {
        trajectory = new TrajectoryAction(TrajectorySet.TUNE_DRIVETRAIN);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Tune Drivetrain path");
        runAction(new WaitAction(1));
        runAction(trajectory);
    }
}
