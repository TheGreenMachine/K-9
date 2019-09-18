package com.team1816.frc2019;

import com.team1816.frc2019.auto.modes.DriveByCameraMode;
import com.team1816.frc2019.auto.modes.FrontThenSideCargoShipMode;
import com.team1816.frc2019.auto.modes.TuneDrivetrainMode;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.auto.modes.DoNothingMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum StartingPosition {
        LEFT_HAB_2, RIGHT_HAB_2, CENTER_HAB_1, LEFT_HAB_1, RIGHT_HAB_1,
        LEFT, CENTER, RIGHT,
    }

    enum DesiredMode {
        DRIVE_BY_CAMERA,
        DO_NOTHING,
        FRONT_THEN_SIDE_CARGO_SHIP,
        TUNE_DRIVETRAIN,
        CROSS_AUTO_LINE,
        LIVING_ROOM,
        SHOP,
        PID,
        DRIVE_STRAIGHT
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Left HAB 2", StartingPosition.LEFT_HAB_2);
        mStartPositionChooser.addOption("Right HAB 2", StartingPosition.RIGHT_HAB_2);
        mStartPositionChooser.addOption("Right HAB 1", StartingPosition.RIGHT_HAB_1);
        mStartPositionChooser.addOption("Left HAB 1", StartingPosition.LEFT_HAB_1);
        mStartPositionChooser.addOption("Center HAB 1", StartingPosition.CENTER_HAB_1);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Drive By Camera", DesiredMode.DRIVE_BY_CAMERA);
        mModeChooser.addOption("Tune Drivetrain", DesiredMode.TUNE_DRIVETRAIN);
        mModeChooser.addOption("Front Then Side Cargo Ship", DesiredMode.FRONT_THEN_SIDE_CARGO_SHIP);
        mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
        SmartDashboard.putData("Auto mode", mModeChooser);

        // CheezeCurd
//        mModeChooser.addOption("Living Room",DesiredMode.LIVING_ROOM);
//        mModeChooser.addOption("Shop", DesiredMode.SHOP);
//        mModeChooser.addOption("PID", DesiredMode.PID);
        mModeChooser.addOption("Drive Straight", DesiredMode.DRIVE_STRAIGHT);

        mStartPositionChooser.addOption("Center", StartingPosition.CENTER);
        SmartDashboard.putData("Starting Position", mStartPositionChooser);

//        mSwitchScalePositionChooser = new SendableChooser<>();
//        mSwitchScalePositionChooser.setDefaultOption("Use FMS Data", SwitchScalePosition.USE_FMS_DATA);
//        mSwitchScalePositionChooser.addOption("Left Switch Left Scale", SwitchScalePosition.LEFT_SWITCH_LEFT_SCALE);
//        mSwitchScalePositionChooser.addOption("Left Switch Right Scale", SwitchScalePosition.LEFT_SWITCH_RIGHT_SCALE);
//        mSwitchScalePositionChooser.addOption("Right Switch Left Scale", SwitchScalePosition.RIGHT_SWITCH_LEFT_SCALE);
//        mSwitchScalePositionChooser.addOption("Right Switch Right Scale", SwitchScalePosition.RIGHT_SWITCH_RIGHT_SCALE);
//        SmartDashboard.putData("Switch and Scale Position", mSwitchScalePositionChooser);

    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition staringPosition = mStartPositionChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || staringPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + ", starting position->" + staringPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, staringPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = staringPosition;
    }

    private boolean startingLeft(StartingPosition position) {
        return position == StartingPosition.LEFT_HAB_1 || position == StartingPosition.LEFT_HAB_2;
    }

    private boolean startingHab1(StartingPosition position) {
        return position == StartingPosition.LEFT_HAB_1 || position == StartingPosition.RIGHT_HAB_1;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case DRIVE_BY_CAMERA:
                return Optional.of(new DriveByCameraMode());
            case FRONT_THEN_SIDE_CARGO_SHIP:
                return Optional.of(new FrontThenSideCargoShipMode(
                    startingLeft(position), startingHab1(position)));
            case TUNE_DRIVETRAIN:
                return Optional.of(new TuneDrivetrainMode());
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
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DRIVE_BY_CAMERA;
    }
}
