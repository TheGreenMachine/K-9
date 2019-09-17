package com.team1816.frc2019.paths;

import com.team1816.frc2019.Robot;
import com.team1816.frc2019.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = DriveMotionPlanner.getInstance();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // -x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)

    // shop
    private static final Pose2d kShop1 = new Pose2d(-74,-36, Rotation2d.fromDegrees(180-45));
    private static final Pose2d kShop2 = new Pose2d(-114,-126, Rotation2d.fromDegrees(180-22));
    private static final Pose2d kVexBox = new Pose2d(-198,-150, Rotation2d.fromDegrees(180));

    private static final Pose2d kMiddleWalkway = new Pose2d(-79.5,11.0, Rotation2d.fromDegrees(180+45));
    private static final Pose2d kStairs = new Pose2d(-176,36, Rotation2d.fromDegrees(180));


    public class TrajectorySet {

        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        public final Trajectory<TimedState<Pose2dWithCurvature>> driveStraight;

        private TrajectorySet() {
            driveStraight = new DriveStraight().generateTrajectory();
        }
    }
}
