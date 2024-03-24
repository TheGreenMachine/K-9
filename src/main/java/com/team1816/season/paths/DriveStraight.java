package com.team1816.season.paths;

import com.google.inject.Inject;
import com.team1816.lib.paths.PathContainer;
import com.team1816.season.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

public class DriveStraight implements PathContainer {

    private final int driveDistance;
    private final double maxVel;
    private final Pose2d start;

    @Inject
    protected static RobotState mRobotState;

    public DriveStraight(Pose2d start, int driveDistance, double maxVel) {
        this.driveDistance = driveDistance;
        this.maxVel = maxVel;
        this.start = start;
    }
    public DriveStraight(Pose2d start, int driveDistance) {
        this(start, driveDistance, PathContainer.kMaxVelocity);
    }

    public DriveStraight(int driveDistance) {
        this(mRobotState.getLatestFieldToVehicle(), driveDistance, PathContainer.kMaxVelocity);
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(start);
        waypoints.add(
            new Pose2d(
                start.getX() + Units.inchesToMeters(driveDistance),
                start.getY(),
                Rotation2d.fromDegrees(0)
            )
        );
        return waypoints;
    }

    @Override
    public boolean isReversed() {
        return driveDistance < 0;
    }
}
