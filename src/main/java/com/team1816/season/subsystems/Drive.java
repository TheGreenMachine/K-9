package com.team1816.season.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team1816.lib.hardware.EnhancedMotorChecker;
import com.team1816.lib.hardware.PigeonChecker;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.subsystems.TrackableDrivetrain;
import com.team1816.season.AutoModeSelector;
import com.team1816.season.Constants;
import com.team1816.season.Kinematics;
import com.team1816.season.RobotState;
import com.team1816.season.planners.DriveMotionPlanner;
import com.team254.lib.control.Lookahead;
import com.team254.lib.control.Path;
import com.team254.lib.control.PathFollower;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem implements TrackableDrivetrain, PidProvider {

    private static Drive mInstance;
    private static final String NAME = "drivetrain";
    private static double DRIVE_ENCODER_PPR;

    // hardware
    private final IMotorControllerEnhanced mLeftMaster, mRightMaster;
    private final IMotorController mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;

    // Controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;

    // control states
    private DriveControlState mDriveControlState;
    private final PigeonIMU mPigeon;

    // hardware states
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private double openLoopRampRate;
    private BadLog mLogger;

    private PeriodicIO mPeriodicIO;
    private final DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;

    private boolean isSlowMode;

    private final RobotState mRobotState = RobotState.getInstance();

    // simulator
    private final Field2d fieldSim = new Field2d();
    private double leftEncoderSimPosition = 0, rightEncoderSimPosition = 0;
    private double gyroDrift;
    private final double tickRatioPerLoop = Constants.kLooperDt/.1d;
    private double robotWidthTicks;

    public static synchronized Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    private Drive() {
        super(NAME);
        DRIVE_ENCODER_PPR = factory.getConstant(NAME, "encPPR");
        robotWidthTicks = inchesPerSecondToTicksPer100ms(Constants.kDriveWheelTrackWidthInches) * Math.PI;
        mPeriodicIO = new PeriodicIO();

        // start all Talons in open loop mode
        mLeftMaster = factory.getMotor(NAME, "leftMain");
        mLeftSlaveA = factory.getMotor(NAME, "leftFollower", mLeftMaster);
        mLeftSlaveB = factory.getMotor(NAME, "leftFollowerTwo", mLeftMaster);
        mRightMaster = factory.getMotor(NAME, "rightMain");
        mRightSlaveA = factory.getMotor(NAME, "rightFollower", mRightMaster);
        mRightSlaveB = factory.getMotor(NAME, "rightFollowerTwo", mRightMaster);

        setOpenLoopRampRate(Constants.kOpenLoopRampRate);

        if (((int) factory.getConstant(NAME, "pigeonOnTalon")) == 1) {
            var pigeonId = ((int) factory.getConstant(NAME, "pigeonId"));
            System.out.println("Pigeon on Talon " + pigeonId);
            IMotorController master = null;
            if (pigeonId == mLeftSlaveA.getDeviceID()) {
                master = mLeftSlaveA;
            } else if (pigeonId == mLeftSlaveB.getDeviceID()) {
                master = mLeftSlaveB;
            } else if (pigeonId == mRightSlaveA.getDeviceID()) {
                master = mRightSlaveA;
            } else if (pigeonId == mRightSlaveB.getDeviceID()) {
                master = mRightSlaveB;
            }
            if (master != null) {
                mPigeon = new PigeonIMU((TalonSRX) master);
            } else {
                mPigeon =
                    new PigeonIMU(
                        new TalonSRX((int) factory.getConstant(NAME, "pigeonId"))
                    );
            }
        } else {
            mPigeon = new PigeonIMU((int) factory.getConstant(NAME, "pigeonId"));
        }
        mPigeon.configFactoryDefault();

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = false;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();

        SmartDashboard.putData("Field", fieldSim);
    }

    public double getHeadingDegrees() {
        return mPeriodicIO.gyro_heading.getDegrees();
    }

    public double getDesiredHeading() {
        return getDesiredRotation2d().getDegrees();
    }

    public Rotation2d getDesiredRotation2d() {
        if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            return mPeriodicIO.path_setpoint.state().getRotation();
        }
        return mPeriodicIO.desired_heading;
    }

    @Override
    public double getKP() {
        return factory.getConstant(NAME, "kP");
    }

    @Override
    public double getKI() {
        return factory.getConstant(NAME, "kI");
    }

    @Override
    public double getKD() {
        return factory.getConstant(NAME, "kD");
    }

    @Override
    public double getKF() {
        return factory.getConstant(NAME, "kF");
    }

    public static class PeriodicIO {

        // INPUTS
        public double timestamp;
        public double left_position_ticks;
        public double right_position_ticks;
        public double left_velocity_ticks_per_100ms;
        public double right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        // no_offset = Relative to initial position, unaffected by reset
        public Rotation2d gyro_heading_no_offset = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();
        double left_error;
        double right_error;

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public Rotation2d desired_heading = Rotation2d.identity();
        TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(
            Pose2dWithCurvature.identity()
        );
    }

    @Override
    public synchronized void readPeriodicInputs() {
        if(RobotBase.isSimulation()) {
            var driveTrainErrorPercent = .05;
            mPeriodicIO.left_error = mPeriodicIO.left_demand * driveTrainErrorPercent;
            leftEncoderSimPosition += (mPeriodicIO.left_demand - mPeriodicIO.left_error) * tickRatioPerLoop;
            rightEncoderSimPosition += mPeriodicIO.right_demand * tickRatioPerLoop;
            mPeriodicIO.left_position_ticks = leftEncoderSimPosition;
            mPeriodicIO.right_position_ticks = rightEncoderSimPosition;
            mPeriodicIO.left_velocity_ticks_per_100ms = mPeriodicIO.left_demand - mPeriodicIO.left_error;
            mPeriodicIO.right_velocity_ticks_per_100ms = mPeriodicIO.right_demand;
            // calculate rotation based on left/right vel differences
            gyroDrift -= (mPeriodicIO.left_velocity_ticks_per_100ms-mPeriodicIO.right_velocity_ticks_per_100ms)/robotWidthTicks;
            mPeriodicIO.gyro_heading_no_offset = getDesiredRotation2d().rotateBy(Rotation2d.fromDegrees(gyroDrift));
            var rot2d = new edu.wpi.first.wpilibj.geometry.Rotation2d(mPeriodicIO.gyro_heading_no_offset.getRadians());
            fieldSim.setRobotPose(Units.inches_to_meters(mRobotState.getEstimatedX()), Units.inches_to_meters(mRobotState.getEstimatedY())+3.5, rot2d);
        } else {
            mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
            mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
            mPeriodicIO.left_velocity_ticks_per_100ms =
                mLeftMaster.getSelectedSensorVelocity(0);
            mPeriodicIO.right_velocity_ticks_per_100ms =
                mRightMaster.getSelectedSensorVelocity(0);
            mPeriodicIO.gyro_heading_no_offset = Rotation2d.fromDegrees(mPigeon.getFusedHeading());
            if (mDriveControlState == DriveControlState.OPEN_LOOP) {
                mPeriodicIO.left_error = 0;
                mPeriodicIO.right_error = 0;
            } else {
                mPeriodicIO.left_error = mLeftMaster.getClosedLoopError(0);
                mPeriodicIO.right_error = mRightMaster.getClosedLoopError(0);
            }
        }
        mPeriodicIO.gyro_heading = mPeriodicIO.gyro_heading_no_offset.rotateBy(mGyroOffset);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            if (isSlowMode) {
                mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand * 0.5);
                mRightMaster.set(
                    ControlMode.PercentOutput,
                    mPeriodicIO.right_demand * 0.5
                );
            } else {
                mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
                mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
            }
        } else {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(
            new Loop() {
                @Override
                public void onStart(double timestamp) {
                    synchronized (Drive.this) {
                        stop();
                        setBrakeMode(false);
                    }
                }

                @Override
                public void onLoop(double timestamp) {
                    synchronized (Drive.this) {
                        switch (mDriveControlState) {
                            case OPEN_LOOP:
                                break;
                            case PATH_FOLLOWING:
                                if (mPathFollower != null) {
                                    if (Constants.kIsBadlogEnabled) {
                                        mLogger.updateTopics();
                                        mLogger.log();
                                    }
                                    updatePathFollower(timestamp);
                                }
                            case TRAJECTORY_FOLLOWING:
                                if (Constants.kIsBadlogEnabled) {
                                    mLogger.updateTopics();
                                    mLogger.log();
                                }
                                updatePathFollower(timestamp);
                                break;
                            default:
                                System.out.println(
                                    "unexpected drive control state: " +
                                    mDriveControlState
                                );
                                break;
                        }
                    }
                }

                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            }
        );
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToTicksPer100ms(double inches_per_second) {
        return inchesToRotations(inches_per_second) * DRIVE_ENCODER_PPR / 10.0;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR / 10.0;
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        setBrakeMode(signal.getBrakeMode());
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    public void setOpenLoopRampRate(double openLoopRampRate) {
        this.openLoopRampRate = openLoopRampRate;
        mLeftMaster.configOpenloopRamp(openLoopRampRate, Constants.kCANTimeoutMs);
        mRightMaster.configOpenloopRamp(openLoopRampRate, Constants.kCANTimeoutMs);
    }

    public double getOpenLoopRampRate() {
        return this.openLoopRampRate;
    }

    /**
     * Configure talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("Switching to Velocity");
            mLeftMaster.selectProfileSlot(0, 0);
            mRightMaster.selectProfileSlot(0, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public void setSlowMode(boolean slowMode) {
        isSlowMode = slowMode;
    }

    public void setLogger(BadLog logger) {
        mLogger = logger;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            System.out.println("setBrakeMode " + on);
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);
            mRightSlaveB.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);
            mLeftSlaveB.setNeutralMode(mode);
        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized Rotation2d getHeadingRelativeToInitial() {
        return mPeriodicIO.gyro_heading_no_offset;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset =
            heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.desired_heading = heading;
    }

    public synchronized void resetPigeon() {
        mPigeon.setYaw(0, Constants.kLongCANTimeoutMs);
        mPigeon.setFusedHeading(0, Constants.kLongCANTimeoutMs);
        mPigeon.setAccumZAngle(0, Constants.kLongCANTimeoutMs);
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
    }

    public DriveControlState getDriveControlState() {
        return mDriveControlState;
    }

    private double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    private double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    @Override
    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(
            getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR
        );
    }

    @Override
    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return (
            Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity()) / 2.0
        );
    }

    public double getAngularVelocity() {
        return (
            (getRightLinearVelocity() - getLeftLinearVelocity()) /
            Constants.kDriveWheelTrackWidthInches
        );
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     *
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (
            mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING
        ) {
            mRobotState.resetDistanceDriven();
            mPathFollower =
                new PathFollower(
                    path,
                    reversed,
                    new PathFollower.Parameters(
                        new Lookahead(
                            Constants.kMinLookAhead,
                            Constants.kMaxLookAhead,
                            Constants.kMinLookAheadSpeed,
                            Constants.kMaxLookAheadSpeed
                        ),
                        Constants.kInertiaSteeringGain,
                        Constants.kPathFollowingProfileKp,
                        Constants.kPathFollowingProfileKi,
                        Constants.kPathFollowingProfileKv,
                        Constants.kPathFollowingProfileKffv,
                        Constants.kPathFollowingProfileKffa,
                        Constants.kPathFollowingProfileKs,
                        Constants.kPathFollowingMaxVel,
                        Constants.kPathFollowingMaxAccel,
                        Constants.kPathFollowingGoalPosTolerance,
                        Constants.kPathFollowingGoalVelTolerance,
                        Constants.kPathStopSteeringDistance
                    )
                );
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }
    }

    public synchronized void setTrajectory(
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory
    ) {
        if (mMotionPlanner != null) {
            System.out.println("Now setting trajectory");
            setBrakeMode(true);
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (
            mMotionPlanner == null ||
            mDriveControlState != DriveControlState.TRAJECTORY_FOLLOWING
        ) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public synchronized boolean isDoneWithPath() {
        if (
            mDriveControlState == DriveControlState.PATH_FOLLOWING &&
            mPathFollower != null
        ) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (
            mDriveControlState == DriveControlState.PATH_FOLLOWING &&
            mPathFollower != null
        ) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    private void updatePathFollower(double timestamp) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            Pose2d field_to_vehicle = mRobotState.getLatestFieldToVehicle().getValue();
            Twist2d command = mPathFollower.update(
                timestamp,
                field_to_vehicle,
                mRobotState.getDistanceDriven(),
                mRobotState.getPredictedVelocity().dx
            );
            if (!mPathFollower.isFinished()) {
                DriveSignal setpoint = Kinematics.inverseKinematics(command);
                setVelocity(
                    new DriveSignal(
                        inchesPerSecondToTicksPer100ms(setpoint.getLeft()),
                        inchesPerSecondToTicksPer100ms(setpoint.getRight())
                    ),
                    new DriveSignal(0, 0)
                );
            } else {
                if (!mPathFollower.isForceFinished()) {
                    setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
                }
            }
        } else if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            DriveMotionPlanner.Output output = mMotionPlanner.update(
                timestamp,
                mRobotState.getFieldToVehicle(timestamp)
            );

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setVelocity(
                    new DriveSignal(
                        radiansPerSecondToTicksPer100ms(output.left_velocity),
                        radiansPerSecondToTicksPer100ms(output.right_velocity)
                    ),
                    new DriveSignal(
                        output.left_feedforward_voltage / 12.0,
                        output.right_feedforward_voltage / 12.0
                    )
                );

                mPeriodicIO.left_accel =
                    radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel =
                    radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (
            mDriveControlState == DriveControlState.PATH_FOLLOWING &&
            mPathFollower != null
        ) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
        TRAJECTORY_FOLLOWING,
    }

    @Override
    public void zeroSensors() {
        System.out.println("Zeroing drive sensors!");
        resetPigeon();
        setHeading(Rotation2d.identity());
        resetEncoders();
        if (mPigeon.getLastError() != ErrorCode.OK && RobotBase.isReal()) {
            System.out.println(
                "Error detected with Pigeon IMU - check if the sensor is present and plugged in!"
            );
            System.out.println("Defaulting to drive straight mode");
            AutoModeSelector.getInstance().setHardwareFailure(true);
        } else {
            AutoModeSelector.getInstance().setHardwareFailure(false);
        }
    }

    public boolean hasPigeonResetOccurred() {
        return mPigeon.hasResetOccurred();
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {
        setBrakeMode(false);

        boolean leftSide = EnhancedMotorChecker.checkMotors(
            this,
            getTalonCheckerConfig(mLeftMaster),
            new EnhancedMotorChecker.NamedMotor("left_master", mLeftMaster)
        );
        boolean rightSide = EnhancedMotorChecker.checkMotors(
            this,
            getTalonCheckerConfig(mRightMaster),
            new EnhancedMotorChecker.NamedMotor("right_master", mRightMaster)
        );

        boolean checkPigeon = PigeonChecker.checkPigeon(mPigeon);
        return leftSide && rightSide && checkPigeon;
    }

    private EnhancedMotorChecker.CheckerConfig getTalonCheckerConfig(
        IMotorControllerEnhanced talon
    ) {
        return EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, talon);
    }

    @Override
    public double getLeftVelocityDemand() {
        return mPeriodicIO.left_demand;
    }

    @Override
    public double getRightVelocityDemand() {
        return mPeriodicIO.right_demand;
    }

    @Override
    public double getLeftVelocityError() {
        return mPeriodicIO.left_error;
    }

    @Override
    public double getRightVelocityError() {
        return mPeriodicIO.right_error;
    }

    @Override
    public double getFieldXDistance() {
        return mRobotState.getEstimatedX();
    }

    @Override
    public double getFieldYDistance() {
        return mRobotState.getEstimatedY();
    }

    @Override
    public double getFieldDesiredXDistance() {
        return mPeriodicIO.path_setpoint.state().getTranslation().x();
    }

    @Override
    public double getFieldYDesiredYDistance() {
        return mPeriodicIO.path_setpoint.state().getTranslation().y();
    }

    public double getRightDriveTicks() {
        return mPeriodicIO.right_position_ticks;
    }

    public double getLeftDriveTicks() {
        return mPeriodicIO.left_position_ticks;
    }

    @Override
    public void initSendable(SendableBuilder builder) {

        builder.addDoubleProperty("Right Drive Ticks", this::getRightDriveTicks, null);

        builder.addDoubleProperty("Left Drive Ticks", this::getLeftDriveTicks, null);
        builder.addStringProperty(
            "Drive/ControlState",
            () -> this.getDriveControlState().toString(),
            null
        );
        builder.addBooleanProperty(
            "Drive/PigeonIMU State",
            () -> this.mPigeon.getLastError() == ErrorCode.OK,
            null
        );

        SmartDashboard.putNumber("Drive/OpenLoopRampRate", this.openLoopRampRate);
        SmartDashboard
            .getEntry("Drive/OpenLoopRampRate")
            .addListener(
                notification -> {
                    setOpenLoopRampRate(notification.value.getDouble());
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );

        SmartDashboard.putBoolean("Drive/Zero Sensors", false);
        SmartDashboard
            .getEntry("Drive/Zero Sensors")
            .addListener(
                entryNotification -> {
                    if (entryNotification.value.getBoolean()) {
                        zeroSensors();
                        entryNotification.getEntry().setBoolean(false);
                    }
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}
