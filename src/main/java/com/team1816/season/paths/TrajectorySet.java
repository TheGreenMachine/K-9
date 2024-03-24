package com.team1816.season.paths;

import com.google.inject.Singleton;
import edu.wpi.first.math.trajectory.Trajectory;

@Singleton
public class TrajectorySet {

    public static Trajectory LIVING_ROOM;

    public TrajectorySet() {
        LIVING_ROOM = new LivingRoomPath().generateTrajectory();
    }
}
