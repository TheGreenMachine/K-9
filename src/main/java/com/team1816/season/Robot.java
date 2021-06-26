package com.team1816.season;

import static com.team1816.season.controlboard.ControlUtils.*;

import badlog.lib.BadLog;
import com.team1816.lib.auto.AutoModeExecutor;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.DrivetrainLogger;
import com.team1816.lib.subsystems.Infrastructure;
import com.team1816.lib.subsystems.RobotStateEstimator;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team1816.season.controlboard.ActionManager;
import com.team1816.season.controlboard.ControlBoard;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.LatchedBoolean;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Optional;

public class Robot extends TimedRobot {

    private BadLog logger;

    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    // subsystems
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final LedManager ledManager = LedManager.getInstance();
    private final Turret turret = Turret.getInstance();
    private final Camera camera = Camera.getInstance();

    // button placed on the robot to allow the drive team to zero the robot right
    // before the start of a match
    DigitalInput resetRobotButton = new DigitalInput(Constants.kResetButtonChannel);

    private boolean mHasBeenEnabled = false;

    private LatchedBoolean mWantsAutoExecution = new LatchedBoolean();
    private LatchedBoolean mWantsAutoInterrupt = new LatchedBoolean();

    private AutoModeSelector mAutoModeSelector = AutoModeSelector.getInstance();
    private AutoModeExecutor mAutoModeExecutor;

    private boolean mDriveByCameraInAuto = false;
    private double loopStart;
    private boolean faulted;

    private ActionManager actionManager;
    private CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();

    private PowerDistributionPanel pdp = new PowerDistributionPanel();
    private Turret.ControlMode prevTurretControlMode = Turret.ControlMode.FIELD_FOLLOWING;

    public Robot() {
        super();
    }

    private Double getLastLoop() {
        return (Timer.getFPGATimestamp() - loopStart) * 1000;
    }

    @Override
    public void robotInit() {
        try {
            DriverStation.getInstance().silenceJoystickConnectionWarning(true);
            if (Constants.kIsBadlogEnabled) {
                var logFile = new SimpleDateFormat("MMdd_HH-mm").format(new Date());
                var robotName = System.getenv("ROBOT_NAME");
                if (robotName == null) robotName = "default";
                var filePath = " /home/lvuser/" + robotName + "_" + logFile + ".bag";
                // if there is a usb drive use it
                if (Files.exists(Path.of("/media/sda1"))) {
                    filePath = "/media/sda1/" + robotName + "_" + logFile + ".bag";
                }
                if (System.getProperty("os.name").toLowerCase().contains("win")) {
                    filePath =
                        System.getenv("temp") + "\\" + robotName + "_" + logFile + ".bag";
                }
                logger = BadLog.init(filePath);

                BadLog.createTopic(
                    "Timings/Looper",
                    "ms",
                    mEnabledLooper::getLastLoop,
                    "hide",
                    "join:Timings"
                );
                BadLog.createTopic(
                    "Timings/RobotLoop",
                    "ms",
                    this::getLastLoop,
                    "hide",
                    "join:Timings"
                );
                BadLog.createTopic(
                    "Timings/Timestamp",
                    "s",
                    Timer::getFPGATimestamp,
                    "xaxis",
                    "hide"
                );

                if (RobotBase.isReal()) {
                    BadLog.createTopic("PDP/Current", "Amps", pdp::getTotalCurrent);
                }

                DrivetrainLogger.init(mDrive);
                mDrive.CreateBadLogValue("Drivetrain PID", mDrive.pidToString());

                if (RobotBase.isReal()) {
                    turret.CreateBadLogValue("Turret PID", turret.pidToString());
                    turret.CreateBadLogTopic(
                        "Vision/DeltaXAngle",
                        "Degrees",
                        camera::getDeltaXAngle
                    );
                    turret.CreateBadLogTopic(
                        "Vision/Distance",
                        "inches",
                        camera::getDistance
                    );
                    turret.CreateBadLogTopic(
                        "Vision/CenterX",
                        "pixels",
                        camera::getRawCenterX
                    );
                    turret.CreateBadLogTopic(
                        "Turret/ActPos",
                        "NativeUnits",
                        () -> (double) turret.getActualTurretPositionTicks(),
                        "hide",
                        "join:Turret/Positions"
                    );
                    turret.CreateBadLogTopic(
                        "Turret/TargetPos",
                        "NativeUnits",
                        turret::getTargetPosition,
                        "hide",
                        "join:Turret/Positions"
                    );
                    turret.CreateBadLogTopic(
                        "Turret/ErrorPos",
                        "NativeUnits",
                        turret::getPositionError
                    );
                }

                turret.CreateBadLogTopic(
                    "Turret/FieldToTurret",
                    "Degrees",
                    mRobotState::getLatestFieldToTurret,
                    "hide",
                    "join:Tracking/Angles"
                );
                turret.CreateBadLogTopic(
                    "Drive/HeadingRelativeToInitial",
                    "Degrees",
                    () -> mDrive.getHeadingRelativeToInitial().getDegrees(),
                    "hide",
                    "join:Tracking/Angles"
                );
                turret.CreateBadLogTopic(
                    "Turret/TurretAngle",
                    "Degrees",
                    turret::getActualTurretPositionDegrees,
                    "hide",
                    "join:Tracking/Angles"
                );

                mDrive.setLogger(logger);

                logger.finishInitialization();
            }

            mSubsystemManager.setSubsystems(
                mRobotStateEstimator,
                mDrive,
                mSuperstructure,
                mInfrastructure,
                turret
            );

            mDrive.zeroSensors();
            turret.zeroSensors();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            ledManager.registerEnabledLoops(mEnabledLooper);
            ledManager.registerEnabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset(
                Timer.getFPGATimestamp(),
                Pose2d.identity(),
                Rotation2d.identity()
            );
            mDrive.setHeading(Rotation2d.identity());

            TrajectorySet.getInstance();

            mAutoModeSelector.updateModeCreator();

            actionManager =
                new ActionManager(
                    // Driver Gamepad
                    createAction(
                        mControlBoard::getTrenchToFeederSpline,
                        () -> {
                            System.out.println("STARTING TRENCH TO FEEDER");
                            SmartDashboard.putString("Teleop Spline", "TRENCH TO FEEDER");
                            var trajectory = new DriveTrajectory(
                                TrajectorySet.getInstance().TRENCH_TO_FEEDER,
                                true
                            );
                            trajectory.start();
                        }
                    ),
                    createAction(
                        mControlBoard::getFeederToTrenchSpline,
                        () -> {
                            System.out.println("STARTING FEEDER TO TRENCH");
                            SmartDashboard.putString("Teleop Spline", "FEEDER TO TRENCH");
                            turret.setTurretAngle(Turret.CARDINAL_SOUTH);
                            var trajectory = new DriveTrajectory(
                                TrajectorySet.getInstance().FEEDER_TO_TRENCH,
                                true
                            );
                            trajectory.start();
                        }
                    ),
                    createHoldAction(mControlBoard::getSlowMode, mDrive::setSlowMode),
                    // Operator Gamepad
                    createAction(
                        mControlBoard::getFieldFollowing,
                        () -> turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING)
                    ),
                    createHoldAction(
                        mControlBoard::getTurretJogLeft,
                        moving ->
                            turret.setTurretSpeed(moving ? -Turret.TURRET_JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getTurretJogRight,
                        moving ->
                            turret.setTurretSpeed(moving ? Turret.TURRET_JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getAutoAim,
                        pressed -> {
                            if (pressed) {
                                prevTurretControlMode = turret.getControlMode();
                                turret.setControlMode(
                                    Turret.ControlMode.CAMERA_FOLLOWING
                                );
                            } else {
                                turret.setControlMode(prevTurretControlMode);
                            }
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getShoot,
                        shooting -> {
                            // shooter.setVelocity(shooting ? Shooter.MID_VELOCITY : 0);
                            if (shooting) {
                                mDrive.setOpenLoop(DriveSignal.BRAKE);
                                turret.lockTurret();
                            } else {
                                turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
                            }
                        }
                    )
                );
        } catch (Throwable t) {
            faulted = true;
            DriverStation.reportError(t.getMessage(), true);
        }
    }

    @Override
    public void disabledInit() {
        try {
            mEnabledLooper.stop();

            ledManager.setDefaultStatus(LedManager.RobotStatus.DISABLED);

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mInfrastructure.setIsManualControl(false);

            mDisabledLooper.start();

            mDrive.setBrakeMode(false);
        } catch (Throwable t) {
            faulted = true;
            DriverStation.reportError(t.getMessage(), true);
        }
    }

    @Override
    public void autonomousInit() {
        try {
            mDisabledLooper.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.AUTONOMOUS);

            // Robot starts forwards.
            mRobotState.reset(
                Timer.getFPGATimestamp(),
                Pose2d.identity(),
                Rotation2d.identity()
            );
            mDrive.setHeading(Rotation2d.identity());

            mHasBeenEnabled = true;

            mInfrastructure.setIsManualControl(true); // turn on compressor when superstructure is not moving

            mDrive.zeroSensors();
            turret.zeroSensors();

            System.out.println("Auto init - " + mDriveByCameraInAuto);
            if (!mDriveByCameraInAuto) {
                mAutoModeExecutor.start();
            }

            mEnabledLooper.start();
        } catch (Throwable t) {
            faulted = true;
            DriverStation.reportError(t.getMessage(), true);
        }
    }

    @Override
    public void teleopInit() {
        try {
            mDisabledLooper.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.ENABLED);

            turret.zeroSensors();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mHasBeenEnabled = true;

            mEnabledLooper.start();

            turret.setTurretAngle(Turret.CARDINAL_NORTH);
            turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);

            mInfrastructure.setIsManualControl(true);
            mControlBoard.reset();
        } catch (Throwable t) {
            faulted = true;
            DriverStation.reportError(t.getMessage(), true);
        }
    }

    @Override
    public void testInit() {
        try {
            double initTime = System.currentTimeMillis();

            ledManager.blinkStatus(LedManager.RobotStatus.DRIVETRAIN_FLIPPED);
            // Warning - blocks thread - intended behavior?
            while (System.currentTimeMillis() - initTime <= 3000) {
                ledManager.writePeriodicOutputs();
            }

            mEnabledLooper.stop();
            mDisabledLooper.start();

            ledManager.blinkStatus(LedManager.RobotStatus.DISABLED);

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
            } else {
                System.err.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
                ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
            }
        } catch (Throwable t) {
            faulted = true;
            DriverStation.reportError(t.getMessage(), true);
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
            mRobotStateEstimator.outputToSmartDashboard();
        } catch (Throwable t) {
            faulted = true;
            DriverStation.reportError(t.getMessage(), true);
        }
    }

    @Override
    public void disabledPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        try {
            if (RobotController.getUserButton() && !mHasBeenEnabled) {
                System.out.println("Zeroing Robot!");
                mDrive.zeroSensors();
                turret.zeroSensors();
                mRobotState.reset(
                    Timer.getFPGATimestamp(),
                    Pose2d.identity(),
                    Rotation2d.identity()
                );
                mDrive.setHeading(Rotation2d.identity());
                ledManager.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
            } else {
                if (faulted) {
                    ledManager.blinkStatus(LedManager.RobotStatus.ERROR);
                } else {
                    ledManager.indicateStatus(LedManager.RobotStatus.DISABLED);
                }
            }

            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            mDriveByCameraInAuto = mAutoModeSelector.isDriveByCamera();
            if (
                autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()
            ) {
                System.out.println(
                    "Set auto mode to: " + autoMode.get().getClass().toString()
                );
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
        } catch (Throwable t) {
            faulted = true;
            DriverStation.reportError(t.getMessage(), true);
        }
    }

    @Override
    public void autonomousPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        boolean signalToResume = !mControlBoard.getDrivetrainFlipped(); // TODO: select auto interrupt button
        boolean signalToStop = mControlBoard.getDrivetrainFlipped();
        // Resume if switch flipped up
        if (mWantsAutoExecution.update(signalToResume)) {
            mAutoModeExecutor.resume();
        }

        // Interrupt if switch flipped down
        if (mWantsAutoInterrupt.update(signalToStop)) {
            System.out.println("Auto mode interrupted ");
            mAutoModeExecutor.interrupt();
        }

        if (mDriveByCameraInAuto || mAutoModeExecutor.isInterrupted()) {
            manualControl();
        }

        if (Constants.kIsLoggingAutonomous) {
            logger.updateTopics();
            logger.log();
        }
    }

    @Override
    public void teleopPeriodic() {
        loopStart = Timer.getFPGATimestamp();

        try {
            manualControl();
        } catch (Throwable t) {
            faulted = true;
            DriverStation.reportError(t.getMessage(), true);
        }

        if (Constants.kIsLoggingTeleOp) {
            logger.updateTopics();
            logger.log();
        }
    }

    public void manualControl() {
        // boolean arcadeDrive = false;
        actionManager.update();

        double throttle = mControlBoard.getThrottle();
        double turn = mControlBoard.getTurn();

        DriveSignal driveSignal;

        // if (arcadeDrive) {
        //            var filteredThrottle = Math.signum(throttle) * (throttle * throttle);
        //            double left = Util.limit(filteredThrottle + (turn * 0.55), 1);
        //            double right = Util.limit(filteredThrottle - (turn * 0.55), 1);
        //            driveSignal = new DriveSignal(left, right);
        // } else {
        driveSignal = cheesyDriveHelper.cheesyDrive(throttle, turn, false); // quick turn temporarily eliminated
        // }
        if (
            mDrive.getDriveControlState() == Drive.DriveControlState.TRAJECTORY_FOLLOWING
        ) {
            if (
                driveSignal.getLeft() != 0 ||
                    driveSignal.getRight() != 0 ||
                    mDrive.isDoneWithTrajectory()
            ) {
                mDrive.setOpenLoop(driveSignal);
            }
        } else {
            mDrive.setOpenLoop(DriveSignal.BRAKE);
        }
    }

    @Override
    public void testPeriodic() {
    }
}
