package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.EnhancedMotorChecker;
import com.team1816.lib.subsystems.DifferentialDrivetrain;
import com.team1816.lib.subsystems.DrivetrainLogger;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.AutoModeSelector;
import com.team1816.season.Constants;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

@Singleton
public class TankDrive extends Drive implements DifferentialDrivetrain {

    private final CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();

    private static final String NAME = "drivetrain";

    @Inject
    private static AutoModeSelector autoModeSelector;

    // hardware
    private final IMotorControllerEnhanced mLeftMaster, mRightMaster;
    private final IMotorController mLeftSlaveA, mRightSlaveA;

    // hardware states

    private double leftEncoderSimPosition = 0, rightEncoderSimPosition = 0;
    private final double tickRatioPerLoop = Constants.kLooperDt / .1d;

    private DifferentialDriveOdometry odometry;

    @Override
    public void updateTrajectoryVelocities(Double leftVel, Double rightVel) {
        // Velocities are in m/sec comes from trajectory command
        var signal = new DriveSignal(
            metersPerSecondToTicksPer100ms(leftVel),
            metersPerSecondToTicksPer100ms(rightVel)
        );
        setVelocity(signal, DriveSignal.NEUTRAL);
    }

    @Override
    public Pose2d getPose() {
        return mRobotState.field_to_vehicle;
    }

    private void updateRobotPose() {
        mRobotState.field_to_vehicle = odometry.getPoseMeters();
    }

    @Override
    public void startTrajectory(Trajectory trajectory) {
        // first entry into auto reset pose
        if(mTrajectory == null) {
            var pose = trajectory.getInitialPose();
            odometry.resetPosition(pose.getRotation(),0,0,pose);
        }
        mTrajectory = trajectory;
        mTrajectoryStart = 0;
        updateRobotPose();
        mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        setBrakeMode(true);
        mOverrideTrajectory = false;
    }

    public TankDrive() {
        super();
        mPeriodicIO = new PeriodicIO();

        // start all Talons in open loop mode
        mLeftMaster = factory.getMotor(NAME, "leftMain");
        mLeftSlaveA = factory.getMotor(NAME, "leftFollower", mLeftMaster);
        mRightMaster = factory.getMotor(NAME, "rightMain");
        mRightSlaveA = factory.getMotor(NAME, "rightFollower", mRightMaster);

        if(Constants.kLoggingRobot){
            DrivetrainLogger.init(this);
        }

        var currentLimitConfig = new SupplyCurrentLimitConfiguration(
            true,
            factory.getConstant(NAME, "currentLimit", 40),
            0,
            0
        );
        mLeftMaster.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        ((IMotorControllerEnhanced) mLeftSlaveA).configSupplyCurrentLimit(
                currentLimitConfig,
                Constants.kLongCANTimeoutMs
            );
        mRightMaster.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        ((IMotorControllerEnhanced) mRightSlaveA).configSupplyCurrentLimit(
                currentLimitConfig,
                Constants.kLongCANTimeoutMs
            );
        setOpenLoopRampRate(Constants.kOpenLoopRampRate);

        mPigeon = factory.getPigeon();
        mPigeon.configFactoryDefaults();

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = false;
        setBrakeMode(mIsBrakeMode);
    }

    @Override
    public double getDesiredHeading() {
        return getDesiredRotation2d().getDegrees();
    }

    public Rotation2d getDesiredRotation2d() {
        if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            return mPeriodicIO.desired_pose.getRotation();
        }
        return mPeriodicIO.desired_heading;
    }

    @Override
    public synchronized void readFromHardware() {
        if (RobotBase.isSimulation()) {
            // read velocities from motor
            var leftVelocity = mLeftMaster.getSelectedSensorVelocity(0);
            var rightVelocity = mRightMaster.getSelectedSensorVelocity(0);
            // create some error in left drive to simulate problem to test ability to correct
            var driveTrainErrorPercent = .05;
            mPeriodicIO.left_error = leftVelocity * driveTrainErrorPercent;
            // calculate distance traveled
            leftEncoderSimPosition +=
                (leftVelocity - mPeriodicIO.left_error) * tickRatioPerLoop;
            rightEncoderSimPosition += rightVelocity * tickRatioPerLoop;
            mPeriodicIO.left_position_ticks = leftEncoderSimPosition;
            mPeriodicIO.right_position_ticks = rightEncoderSimPosition;
            //update velocities for periodic IO
            mPeriodicIO.left_velocity_ticks_per_100ms =
                leftVelocity - mPeriodicIO.left_error;
            mPeriodicIO.right_velocity_ticks_per_100ms = rightVelocity;
            // calculate rotation based on left/right vel differences
            additionalRotation -= (leftVelocity - rightVelocity) / robotWidthTicks;
            mPeriodicIO.gyro_heading_no_offset =
                getDesiredRotation2d()
                    .rotateBy(Rotation2d.fromDegrees(additionalRotation));
        } else {
            mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
            mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
            mPeriodicIO.left_velocity_ticks_per_100ms =
                mLeftMaster.getSelectedSensorVelocity(0);
            mPeriodicIO.right_velocity_ticks_per_100ms =
                mRightMaster.getSelectedSensorVelocity(0);
            mPeriodicIO.gyro_heading_no_offset =
                Rotation2d.fromDegrees(mPigeon.getYawValue());
        }
        mPeriodicIO.gyro_heading =
            mPeriodicIO.gyro_heading_no_offset.rotateBy(mGyroOffset);
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mPeriodicIO.left_error = 0;
            mPeriodicIO.right_error = 0;
        } else {
            mPeriodicIO.left_error = mLeftMaster.getClosedLoopError(0);
            mPeriodicIO.right_error = mRightMaster.getClosedLoopError(0);
        }
        odometry.update(
            mPeriodicIO.gyro_heading,
            Units.inchesToMeters(getLeftEncoderDistance()),
            Units.inchesToMeters(getRightEncoderDistance())
        );
        updateRobotPose();
    }

    @Override
    public synchronized void writeToHardware() {
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
    protected void updateOpenLoopPeriodic() {
        // no openLoop update needed
    }

    @Override
    protected void updateTrajectoryPeriodic(double timestamp) {
        if (mTrajectoryStart == 0) mTrajectoryStart = timestamp;
        // update desired pose from trajectory
        mPeriodicIO.desired_pose =
            mTrajectory.sample(timestamp - mTrajectoryStart).poseMeters;
    }

    /**
     * Configure talons for open loop control
     */

    @Override
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            GreenLogger.log("switching to open loop");
            GreenLogger.log(signal.toString());
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        setBrakeMode(signal.getBrakeMode());
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    @Override
    public void setOpenLoopRampRate(double openLoopRampRate) {
        super.setOpenLoopRampRate(openLoopRampRate);
        mLeftMaster.configOpenloopRamp(openLoopRampRate, Constants.kCANTimeoutMs);
        mRightMaster.configOpenloopRamp(openLoopRampRate, Constants.kCANTimeoutMs);
    }

    @Override
    public void setTeleopInputs(
        double forward,
        double strafe,
        double rotation,
        boolean low_power,
        boolean use_heading_controller
    ) {
        DriveSignal driveSignal = cheesyDriveHelper.cheesyDrive(forward, rotation, false); // quick turn temporarily eliminated
        // }

        if (mDriveControlState == Drive.DriveControlState.TRAJECTORY_FOLLOWING) {
            if (driveSignal.getLeft() != 0 || driveSignal.getRight() != 0) {
                setOpenLoop(driveSignal);
            }
        } else {
            setOpenLoop(driveSignal);
        }

        if (mDriveControlState != Drive.DriveControlState.OPEN_LOOP) {
            mDriveControlState = Drive.DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = driveSignal.getLeft();
        mPeriodicIO.right_demand = driveSignal.getRight();
    }

    /**
     * Configure talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            GreenLogger.log("Switching to Velocity");
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

    @Override
    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            GreenLogger.log("setBrakeMode " + on);
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);
        }
    }

    @Override
    public synchronized void setHeading(Rotation2d heading) {
        GreenLogger.log("set heading: " + heading.getDegrees());

        mGyroOffset =
            heading.rotateBy(
                Rotation2d.fromDegrees(mPigeon.getYawValue()).unaryMinus()
            );
        GreenLogger.log("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.desired_heading = heading;
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
        leftEncoderSimPosition = 0;
        rightEncoderSimPosition = 0;
        mRobotState.field.setRobotPose(Constants.StartingPose);
        odometry =
            new DifferentialDriveOdometry(
                Constants.StartingPose.getRotation(),
                0,0,
                Constants.StartingPose
            );
    }

    public double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    @Override
    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    @Override
    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    @Override
    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    @Override
    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    @Override
    public void zeroSensors() {
        GreenLogger.log("Zeroing drive sensors!");
        resetPigeon();
        setHeading(Constants.EmptyRotation);
        resetEncoders();
        if (!mPigeon.isReady()) {
            // BadLog.createValue("PigeonErrorDetected", "true");
            GreenLogger.log(
                "Error detected with Pigeon IMU - check if the sensor is present and plugged in!"
            );
            GreenLogger.log("Defaulting to drive straight mode");
            autoModeSelector.setHardwareFailure(true);
        } else {
            autoModeSelector.setHardwareFailure(false);
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
        // need for force one last write while enabled for updates to take
        writeToHardware();
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

        boolean checkPigeon = mPigeon == null;

        GreenLogger.log(leftSide && rightSide && checkPigeon);
        if (leftSide && rightSide && checkPigeon) {
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
        } else {
            ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
        }
        return leftSide && rightSide;
    }

    private EnhancedMotorChecker.CheckerConfig getTalonCheckerConfig(
        IMotorControllerEnhanced talon
    ) {
        return EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, talon);
    }

    @Override
    public double getLeftVelocityDemand() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            var ticks = mPeriodicIO.left_demand * maxVelTicksPer100ms;
            return ticks;
        }
        return mPeriodicIO.left_demand;

    }

    @Override
    public double getRightVelocityDemand() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            return mPeriodicIO.right_demand * maxVelTicksPer100ms;
        }
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
}
