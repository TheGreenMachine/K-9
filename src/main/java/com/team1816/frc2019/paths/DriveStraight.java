package com.team1816.frc2019.paths;

import com.team1816.frc2019.planners.DriveMotionPlanner;
import com.team1816.lib.paths.PathBuilder;
import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DriveStraight implements PathContainer {

  @Override
  public Path buildPath() {
    ArrayList<PathBuilder.Waypoint> waypoints = new ArrayList<>();
    waypoints.add(new PathBuilder.Waypoint(0, 0, 0, 0));
    waypoints.add(new PathBuilder.Waypoint(48, 0, 0, 30));
    return  PathBuilder.buildPathFromWaypoints(waypoints);
  }

    @Override
    public List<Pose2d> buildWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(-144, 0.0, Rotation2d.fromDegrees(0)));
        return waypoints;
    }

    @Override
  public boolean isReversed() {
    return false;
  }
}
