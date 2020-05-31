package com.team1816.season.auto.modes;

import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.actions.PrepareToShootAction;
import com.team1816.season.auto.actions.ShootAction;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.geometry.Translation2d;

public class EightBallOpposeMode extends AutoModeBase {
    private DriveTrajectory mDriveTrajectoryA;
    private DriveTrajectory mDriveTrajectoryB;

    public EightBallOpposeMode() {
        var trajectoryA = TrajectorySet.getInstance().EIGHT_BALL_AUTO_OPPOSEA;
        var trajectoryB = TrajectorySet.getInstance().EIGHT_BALL_AUTO_OPPOSEB;
        mDriveTrajectoryA = new DriveTrajectory(trajectoryA, true);
        mDriveTrajectoryB = new DriveTrajectory(trajectoryB, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running 8 Ball Opposition Side Auto Trench Mode");
        runAction(
            new SeriesAction(
                new ParallelAction(
                    mDriveTrajectoryA,
                    new SeriesAction(
                        new WaitUntilInsideRegion(new Translation2d(70, 0), new Translation2d(125, 0)),
                        new CollectAction(true),
                        new WaitUntilInsideRegion(new Translation2d(50, 0), new Translation2d(70,0)),
                        new CollectAction(false)
                    )
                ),
                new PrepareToShootAction(Turret.MAX_ANGLE),
                new ShootAction(true),
                new ParallelAction(
                    mDriveTrajectoryB,
                    new CollectAction(true)
                ),
                new CollectAction(false),
                new PrepareToShootAction(Turret.MAX_ANGLE),
                new ShootAction(true)
            )
        );
    }
}
