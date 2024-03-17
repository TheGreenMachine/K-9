package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.trajectory.Trajectory;

public class DoNothingMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("doing nothing");
    }

    @Override
    public Trajectory getTrajectory() {
        return new Trajectory();
    }
}
