package com.team1816.lib.control;

import com.team254.lib.control.AdaptivePurePursuitController;
import com.team254.lib.control.Lookahead;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.motion.MotionProfileConstraints;
import com.team254.lib.motion.MotionProfileGoal;
import com.team254.lib.motion.MotionProfileGoal.CompletionBehavior;
import com.team254.lib.motion.MotionState;
import com.team254.lib.motion.ProfileFollower;

/**
 * A PathFollower follows a predefined path using a combination of feedforward and feedback control. It uses an
 * AdaptivePurePursuitController to choose a reference pose and generate a steering command (curvature), and then a
 * ProfileFollower to generate a profile (displacement and velocity) command.
 */
public class PathFollower {
    private static final double kReallyBigNumber = 1E6;

    public static class Parameters {
        public final Lookahead lookahead;
        public final double inertia_gain;
        public final double profile_kp;
        public final double profile_ki;
        public final double profile_kv;
        public final double profile_kffv;
        public final double profile_kffa;
        public final double profile_ks;
        public final double profile_max_abs_vel;
        public final double profile_max_abs_acc;
        public final double goal_pos_tolerance;
        public final double goal_vel_tolerance;
        public final double stop_steering_distance;

        public Parameters(Lookahead lookahead, double inertia_gain, double profile_kp, double profile_ki,
                          double profile_kv, double profile_kffv, double profile_kffa, double profile_ks, double profile_max_abs_vel,
                          double profile_max_abs_acc, double goal_pos_tolerance, double goal_vel_tolerance,
                          double stop_steering_distance) {
            this.lookahead = lookahead;
            this.inertia_gain = inertia_gain;
            this.profile_kp = profile_kp;
            this.profile_ki = profile_ki;
            this.profile_kv = profile_kv;
            this.profile_kffv = profile_kffv;
            this.profile_kffa = profile_kffa;
            this.profile_ks = profile_ks;
            this.profile_max_abs_vel = profile_max_abs_vel;
            this.profile_max_abs_acc = profile_max_abs_acc;
            this.goal_pos_tolerance = goal_pos_tolerance;
            this.goal_vel_tolerance = goal_vel_tolerance;
            this.stop_steering_distance = stop_steering_distance;
        }
    }

    AdaptivePurePursuitController mSteeringController;
    Twist2d mLastSteeringDelta;
    ProfileFollower mVelocityController;
    final double mInertiaGain;
    boolean overrideFinished = false;
    boolean doneSteering = false;

    double mMaxProfileVel;
    double mMaxProfileAcc;
    final double mGoalPosTolerance;
    final double mGoalVelTolerance;
    final double mStopSteeringDistance;
    double mCrossTrackError = 0.0;
    double mAlongTrackError = 0.0;
    double mStartTime = 0.0;

    /**
     * Create a new PathFollower for a given path.
     */
    public PathFollower(Path path, boolean reversed, Parameters parameters) {
        mSteeringController = new AdaptivePurePursuitController(path, reversed, parameters.lookahead);
        mLastSteeringDelta = Twist2d.identity();
        mVelocityController = new ProfileFollower(parameters.profile_kp, parameters.profile_ki, parameters.profile_kv,
            parameters.profile_kffv, parameters.profile_kffa, parameters.profile_ks);
        mVelocityController.setConstraints(
            new MotionProfileConstraints(parameters.profile_max_abs_vel, parameters.profile_max_abs_acc));
        mMaxProfileVel = parameters.profile_max_abs_vel;
        mMaxProfileAcc = parameters.profile_max_abs_acc;
        mGoalPosTolerance = parameters.goal_pos_tolerance;
        mGoalVelTolerance = parameters.goal_vel_tolerance;
        mInertiaGain = parameters.inertia_gain;
        mStopSteeringDistance = parameters.stop_steering_distance;
        mStartTime = 0.0;
    }

    /**
     * Get new velocity commands to follow the path.
     *
     * @param t            The current timestamp
     * @param pose         The current robot pose
     * @param displacement The current robot displacement (total distance driven).
     * @param velocity     The current robot velocity.
     * @return The velocity command to apply
     */
    public synchronized Twist2d update(double t, Pose2d pose, double displacement, double velocity) {
        if (mStartTime <= 0.0) mStartTime = t;
        if (!mSteeringController.isFinished()) {
            final AdaptivePurePursuitController.Command steering_command = mSteeringController.update(pose);
            mCrossTrackError = steering_command.cross_track_error;
            mLastSteeringDelta = steering_command.delta;
            mVelocityController.setGoalAndConstraints(
                new MotionProfileGoal(displacement + steering_command.delta.dx,
                    Math.abs(steering_command.end_velocity), CompletionBehavior.VIOLATE_MAX_ACCEL,
                    mGoalPosTolerance, mGoalVelTolerance),
                new MotionProfileConstraints(Math.min(mMaxProfileVel, steering_command.max_velocity),
                    mMaxProfileAcc));

            if (steering_command.remaining_path_length < mStopSteeringDistance) {
                doneSteering = true;
            }
        }

        var pathTime = t - mStartTime;
        final double velocity_command = mVelocityController.update(new MotionState(pathTime, displacement, velocity, 0.0), pathTime);
        mAlongTrackError = mVelocityController.getPosError();
        final double curvature = mLastSteeringDelta.dtheta / mLastSteeringDelta.dx;
        double dtheta = mLastSteeringDelta.dtheta;
        if (!Double.isNaN(curvature) && Math.abs(curvature) < kReallyBigNumber) {
            // Regenerate angular velocity command from adjusted curvature.
            final double abs_velocity_setpoint = Math.abs(mVelocityController.getSetpoint().vel());
            dtheta = mLastSteeringDelta.dx * curvature * (1.0 + mInertiaGain * abs_velocity_setpoint);
        }
        final double scale = velocity_command / mLastSteeringDelta.dx;
        final Twist2d rv = new Twist2d(mLastSteeringDelta.dx * scale, 0.0, dtheta * scale);

        return rv;
    }

    public double getCrossTrackError() {
        return mCrossTrackError;
    }

    public double getAlongTrackError() {
        return mAlongTrackError;
    }

    public boolean isFinished() {
        return (mSteeringController.isFinished() && mVelocityController.isFinishedProfile()
            && mVelocityController.onTarget()) || overrideFinished;
    }

    public void forceFinish() {
        overrideFinished = true;
    }

    public boolean isForceFinished() {
        return overrideFinished;
    }

    public boolean hasPassedMarker(String marker) {
        return mSteeringController.hasPassedMarker(marker);
    }
}
