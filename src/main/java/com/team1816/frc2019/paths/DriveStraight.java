package com.team1816.frc2019.paths;

import com.team1816.lib.paths.PathBuilder;
import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;

import java.util.ArrayList;

public class DriveStraight implements PathContainer {

  @Override
  public Path buildPath() {
    ArrayList<PathBuilder.Waypoint> waypoints = new ArrayList<>();
    waypoints.add(new PathBuilder.Waypoint(0, 0, 0, 0));
    waypoints.add(new PathBuilder.Waypoint(48, 0, 0, 30));
    return  PathBuilder.buildPathFromWaypoints(waypoints);
  }

  @Override
  public boolean isReversed() {
    return false;
  }
}
