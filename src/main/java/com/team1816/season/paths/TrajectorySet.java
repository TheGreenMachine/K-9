package com.team1816.season.paths;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import javax.inject.Singleton;

@Singleton
public class TrajectorySet {

    public static Trajectory<TimedState<Pose2dWithCurvature>> DRIVE_STRAIGHT;
    public static Trajectory<TimedState<Pose2dWithCurvature>> TUNE_DRIVETRAIN;
    public static Trajectory<TimedState<Pose2dWithCurvature>> LIVING_ROOM;

    public TrajectorySet() {
        DRIVE_STRAIGHT = new DriveStraight(12).generateTrajectory();
        TUNE_DRIVETRAIN = new DriveStraight(155).generateTrajectory();
        LIVING_ROOM = new LivingRoomPath().generateTrajectory();
    }
}
