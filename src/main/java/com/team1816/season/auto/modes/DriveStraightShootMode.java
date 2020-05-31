package com.team1816.season.auto.modes;

import com.team1816.season.auto.actions.PrepareToShootAction;
import com.team1816.season.auto.actions.ShootAction;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class DriveStraightShootMode extends AutoModeBase {

    private DriveTrajectory mDriveTrajectory;

    public DriveStraightShootMode() {
        var trajectory = TrajectorySet.getInstance().DRIVE_STRAIGHT;
        mDriveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                mDriveTrajectory,
                new PrepareToShootAction(Turret.CARDINAL_SOUTH),
                new ShootAction(false)
            )
        );
    }
}
