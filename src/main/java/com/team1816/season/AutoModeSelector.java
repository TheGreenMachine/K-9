package com.team1816.season;

import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.auto.modes.DoNothingMode;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.modes.LivingRoomMode;
import com.team1816.season.auto.modes.TuneDrivetrainMode;
import com.team1816.season.auto.modes.TurretTestMode;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.inject.Singleton;
import java.util.Optional;

@Singleton
public class AutoModeSelector {

    private boolean hardwareFailure = false;

    enum StartingPosition {
        LEFT_HAB_2,
        CENTER_HAB_1,
        LEFT_HAB_1,
        RIGHT_HAB_1,
    }

    enum DesiredMode {
        DO_NOTHING,
        TUNE_DRIVETRAIN,
        TURRET_TEST,
        LIVING_ROOM,
        DRIVE_STRAIGHT,
    }

    private DesiredMode mCachedDesiredMode = null;

    private final SendableChooser<DesiredMode> mModeChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();

        mModeChooser.setDefaultOption("Tune Drivetrain", DesiredMode.TUNE_DRIVETRAIN);
        mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);

        // CheezeCurd
        mModeChooser.addOption("Living Room", DesiredMode.LIVING_ROOM);
        mModeChooser.addOption("Turret Tuning", DesiredMode.TURRET_TEST);

        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void setHardwareFailure(boolean hasFailed) {
        if (RobotBase.isReal()) {
            hardwareFailure = hasFailed;
        }
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (mCachedDesiredMode != desiredMode) {
            GreenLogger.log(
                "Auto selection changed, updating creator: desiredMode->" +
                desiredMode.name()
            );
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private boolean startingLeft(StartingPosition position) {
        return (
            position == StartingPosition.LEFT_HAB_1 ||
            position == StartingPosition.LEFT_HAB_2
        );
    }

    private boolean startingHab1(StartingPosition position) {
        return (
            position == StartingPosition.LEFT_HAB_1 ||
            position == StartingPosition.RIGHT_HAB_1
        );
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        if (hardwareFailure) {
            return Optional.of(new DoNothingMode());
        }
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case TUNE_DRIVETRAIN:
                return Optional.of(new TuneDrivetrainMode());
            case TURRET_TEST:
                return Optional.of(new TurretTestMode());
            case LIVING_ROOM:
                return (Optional.of(new LivingRoomMode()));
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        if (mCachedDesiredMode == null) return;
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }
}
