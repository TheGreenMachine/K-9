package com.team1816.lib.paths;

import com.team1816.lib.hardware.RobotFactory;
import com.team254.lib.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.List;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public interface PathContainer {
    // velocities are in/sec
    double kMaxVelocity = Units.inches_to_meters(
        RobotFactory.getInstance().getConstant("maxVel")
    );
    double kMaxAccel = Units.inches_to_meters(
        RobotFactory.getInstance().getConstant("maxAccel")
    );

    List<Pose2d> buildWaypoints();

    default Trajectory generateTrajectory() {
        return generateBaseTrajectory(isReversed(), buildWaypoints());
    }

    private Trajectory generateBaseTrajectory(
        boolean isReversed,
        List<Pose2d> waypoints
    ) {
        TrajectoryConfig config = new TrajectoryConfig(kMaxVelocity, kMaxAccel);
        config.setReversed(isReversed);
        var baseTrajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
        return baseTrajectory;
    }

    boolean isReversed();
}
