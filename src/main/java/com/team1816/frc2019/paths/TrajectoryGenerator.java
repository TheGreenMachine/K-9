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
    // velocities are in/sec
    private static final double kMaxVelocity = Robot.getFactory().getConstant("maxVel");
    private static final double kMaxAccel = Robot.getFactory().getConstant("maxAccel");
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
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

    // STARTING IN CENTER
    private static final Pose2d kCenterStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0));
    private static final Pose2d kCenterStraightPose = new Pose2d(-144, 0.0, Rotation2d.fromDegrees(180.0));


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

        public final Trajectory<TimedState<Pose2dWithCurvature>> centerStartToStairs;
        public final Trajectory<TimedState<Pose2dWithCurvature>> centerStartToVex;
        public final Trajectory<TimedState<Pose2dWithCurvature>> centerPID;
        public final Trajectory<TimedState<Pose2dWithCurvature>> driveStraight;

        private TrajectorySet() {
            centerStartToStairs = getCenterStartToStairs();
            centerStartToVex = getCenterStartToVex();
            centerPID = getPID();
            driveStraight = getDriveStraight();
            // System.out.println(centerPID.toString());
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPID() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCenterStartPose);
            waypoints.add(kCenterStraightPose);
            return mMotionPlanner.generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCenterStartToStairs() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCenterStartPose);
            waypoints.add(kMiddleWalkway);
            waypoints.add(kStairs);
            return mMotionPlanner.generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCenterStartToVex() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCenterStartPose);
            waypoints.add(kShop1);
            waypoints.add(kShop2);
            waypoints.add(kVexBox);
            return mMotionPlanner.generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getDriveStraight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCenterStartPose);
            waypoints.add(kCenterStraightPose);
            return mMotionPlanner.generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
    }
}
