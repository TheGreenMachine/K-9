package com.team1816.season;

import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.LibModule;
import com.team1816.lib.auto.AutoModeExecutor;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.Infrastructure;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team1816.lib.util.GreenDriveHelper;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.controlboard.ActionManager;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.*;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.LatchedBoolean;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.*;

import java.util.Optional;

import static com.team1816.season.controlboard.ControlUtils.createAction;
import static com.team1816.season.controlboard.ControlUtils.createHoldAction;

public class Robot extends TimedRobot {

    private final Injector injector;

    private final Looper mEnabledLooper = new Looper(this);
    private final Looper mDisabledLooper = new Looper(this);

    private IControlBoard mControlBoard;

    private final SubsystemManager mSubsystemManager;

    // subsystems
    private final Superstructure mSuperstructure;
    private final Infrastructure mInfrastructure;
    private final RobotState mRobotState;
    private final Drive mDrive;
    private final LedManager ledManager;
    private final Turret turret;
    private final Camera camera;

    private boolean mHasBeenEnabled = false;

    private final LatchedBoolean mWantsAutoExecution = new LatchedBoolean();
    private final LatchedBoolean mWantsAutoInterrupt = new LatchedBoolean();

    private final AutoModeSelector mAutoModeSelector;
    private AutoModeExecutor mAutoModeExecutor;

    private double loopStart;
    private boolean faulted;

    private ActionManager actionManager;
    private final GreenDriveHelper greenDriveHelper = new GreenDriveHelper();

    private final PowerDistribution pdp = new PowerDistribution(1, PowerDistribution.ModuleType.kCTRE);
    private Turret.ControlMode prevTurretControlMode = Turret.ControlMode.FIELD_FOLLOWING;

    public Robot() {
        super();
        // initialize injector
        injector = Guice.createInjector(new LibModule(), new SeasonModule());
        mDrive = (injector.getInstance(Drive.Factory.class)).getInstance();
        turret = injector.getInstance(Turret.class);
        mRobotState = injector.getInstance(RobotState.class);
        mSuperstructure = injector.getInstance(Superstructure.class);
        mInfrastructure = injector.getInstance(Infrastructure.class);
        ledManager = injector.getInstance(LedManager.class);
        camera = injector.getInstance(Camera.class);
        mSubsystemManager = injector.getInstance(SubsystemManager.class);
        mAutoModeSelector = injector.getInstance(AutoModeSelector.class);
        // we need to get an instance to initialize trajectories
        injector.getInstance(TrajectorySet.class);
    }

    public static RobotFactory getFactory() {
        return RobotFactory.getInstance();
    }

    private Double getLastLoop() {
        var dur = (Timer.getFPGATimestamp() - loopStart) * 1000;
        return dur;
    }

    @Override
    public void robotInit() {
        try {
            DriverStation.silenceJoystickConnectionWarning(true);
            mControlBoard = injector.getInstance(IControlBoard.class);

            mSubsystemManager.setSubsystems(
                mDrive,
                mSuperstructure,
                mInfrastructure,
                turret,
                ledManager
            );

            mDrive.zeroSensors();
            turret.zeroSensors();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            ledManager.registerEnabledLoops(mEnabledLooper);
            ledManager.registerEnabledLoops(mDisabledLooper);

            DataLog log = DataLogManager.getLog();
            GreenLogger.AddPeriodicLog(new DoubleLogEntry(log, "Timings/Looper"),mEnabledLooper::getLastLoop);
            GreenLogger.AddPeriodicLog(new DoubleLogEntry(log, "Timings/RobotLoop"),this::getLastLoop);
            if (RobotBase.isReal()) {
                GreenLogger.AddPeriodicLog(new DoubleLogEntry(log,"PDP/Current"), pdp::getTotalCurrent);
            }

            // Robot starts forwards.
            mRobotState.reset();
            mDrive.setHeading(Constants.EmptyRotation);

            actionManager =
                new ActionManager(
                    // Driver Gamepad
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
            t.printStackTrace();
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
            t.printStackTrace();
        }
    }

    @Override
    public void autonomousInit() {
        try {
            mDisabledLooper.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.AUTONOMOUS);

            // Robot starts forwards.
            mRobotState.reset();
            mDrive.setHeading(Constants.EmptyRotation);

            mHasBeenEnabled = true;

            mInfrastructure.setIsManualControl(true); // turn on compressor when superstructure is not moving

            mDrive.zeroSensors();
            turret.zeroSensors();
            turret.setTurretAngle(Turret.CARDINAL_SOUTH);
            turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
            mAutoModeExecutor.start();
            mEnabledLooper.start();
        } catch (Throwable t) {
            faulted = true;
            t.printStackTrace();
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

            turret.setTurretAngle(Turret.CARDINAL_WEST);
            turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
            mInfrastructure.setIsManualControl(true);
            mControlBoard.reset();
        } catch (Throwable t) {
            faulted = true;
            t.printStackTrace();
        }
    }

    @Override
    public void testInit() {
        try {
            double initTime = System.currentTimeMillis();

            ledManager.blinkStatus(LedManager.RobotStatus.DRIVETRAIN_FLIPPED);
            // Warning - blocks thread - intended behavior?
            while (System.currentTimeMillis() - initTime <= 3000) {
                ledManager.writeToHardware();
            }

            mEnabledLooper.stop();
            mDisabledLooper.start();

            ledManager.blinkStatus(LedManager.RobotStatus.DISABLED);

            if (mSubsystemManager.checkSubsystems()) {
                GreenLogger.log("ALL SYSTEMS PASSED");
                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
            } else {
                System.err.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
                ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
            }
        } catch (Throwable t) {
            faulted = true;
            t.printStackTrace();
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mRobotState.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
        } catch (Throwable t) {
            faulted = true;
            t.printStackTrace();
        }
    }

    @Override
    public void disabledPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        try {
            if (RobotController.getUserButton() && !mHasBeenEnabled) {
                GreenLogger.log("Zeroing Robot!");
                mDrive.zeroSensors();
                turret.zeroSensors();
                mRobotState.reset();
                mDrive.setHeading(Constants.EmptyRotation);
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

            if (
                autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()
            ) {
                var auto = autoMode.get();
                GreenLogger.log("Set auto mode to: " + auto.getClass());
                mAutoModeExecutor.setAutoMode(auto);
            }
        } catch (Throwable t) {
            faulted = true;
            DriverStation.reportError(t.getMessage(), t.getStackTrace());
        }
    }

    @Override
    public void autonomousPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        boolean signalToResume = !mControlBoard.getDrivetrainFlipped();
        boolean signalToStop = mControlBoard.getDrivetrainFlipped();
        // Resume if switch flipped up
        if (mWantsAutoExecution.update(signalToResume)) {
            mAutoModeExecutor.resume();
        }

        // Interrupt if switch flipped down
        if (mWantsAutoInterrupt.update(signalToStop)) {
            GreenLogger.log("Auto mode interrupted ");
            mAutoModeExecutor.interrupt();
        }

        if (mAutoModeExecutor.isInterrupted()) {
            manualControl();
        }
        GreenLogger.UpdatePeriodicLogs();
    }

    @Override
    public void teleopPeriodic() {
        loopStart = Timer.getFPGATimestamp();

        try {
            manualControl();
            GreenLogger.UpdatePeriodicLogs();
        } catch (Throwable t) {
            faulted = true;
            t.printStackTrace();
            t.printStackTrace();
        }
    }

    public void manualControl() {
        // boolean arcadeDrive = false;
        actionManager.update();

        double throttle = mControlBoard.getThrottle();
        double turn = mControlBoard.getTurn();
        DriveSignal driveSignal = greenDriveHelper.cheesyDrive(
            throttle,
            turn,
            false,
            false
        );
        if (
            mDrive.getDriveControlState() == Drive.DriveControlState.TRAJECTORY_FOLLOWING
        ) {
            if (driveSignal.getLeft() != 0 || driveSignal.getRight() != 0) {
                mDrive.setOpenLoop(driveSignal);
            }
        } else {
            mDrive.setOpenLoop(driveSignal);
        }
    }

    @Override
    public void testPeriodic() {}
}
