package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

public class LivingRoomPath implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        var x = Constants.StartingPose.getX();
        var y = Constants.StartingPose.getY();
        waypoints.add(Constants.StartingPose);
        waypoints.add(
            new Pose2d(
                Units.inchesToMeters(79.5) + x,
                Units.inchesToMeters(11.0) + y,
                Rotation2d.fromDegrees(45)
            )
        );
        waypoints.add(
            new Pose2d(
                Units.inchesToMeters(172) + x,
                Units.inchesToMeters(30) + y,
                Rotation2d.fromDegrees(0)
            )
        );
        return waypoints;
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
