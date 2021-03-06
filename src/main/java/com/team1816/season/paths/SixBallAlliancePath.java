package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import java.util.List;

public class SixBallAlliancePath implements PathContainer {

    @Override
    public Path buildPath() {
        return new Path();
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
            new Pose2d(70, 30, Rotation2d.fromDegrees(45)),
            new Pose2d(168, 78, Rotation2d.fromDegrees(0))
            //new Pose2d(124,0, Rotation2d.fromDegrees(180))
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
