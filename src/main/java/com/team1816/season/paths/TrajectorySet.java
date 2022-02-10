package com.team1816.season.paths;

import com.google.inject.Singleton;
import edu.wpi.first.math.trajectory.Trajectory;

@Singleton
public class TrajectorySet {

    public static Trajectory TUNE_DRIVETRAIN;
    public static Trajectory LIVING_ROOM;

    public TrajectorySet() {
        TUNE_DRIVETRAIN = new DriveStraight(155).generateTrajectory();
        LIVING_ROOM = new LivingRoomPath().generateTrajectory();
    }
}
