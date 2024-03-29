package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.paths.TrajectorySet;

public class LivingRoomMode extends AutoModeBase {

    public LivingRoomMode() {
        trajectory = new TrajectoryAction(TrajectorySet.LIVING_ROOM);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Living Room Mode");
        runAction(new WaitAction(.5));
        runAction(trajectory);
    }
}
