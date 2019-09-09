package com.team1816.frc2019.controlboard;

import com.team1816.frc2019.Constants;
import com.team1816.lib.controlboard.IDriveControlBoard;
import edu.wpi.first.wpilibj.Joystick;

public class MainDriveControlBoard implements IDriveControlBoard {
    private static MainDriveControlBoard mInstance = null;

    public static MainDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainDriveControlBoard();
        }

        return mInstance;
    }

    private final Joystick mThrottleStick;
    private final Joystick mTurnStick;
    private final int mThrottleAxisId;

    private MainDriveControlBoard() {
        mThrottleStick = new Joystick(Constants.kMainThrottleJoystickPort);
        mTurnStick = new Joystick(Constants.kMainTurnJoystickPort);
        boolean mSteelSeries = mThrottleStick.getName().contains("SteelSeries");
        mThrottleAxisId = mSteelSeries ? 3 : 1;
    }

    @Override
    public double getThrottle() {
        return mThrottleStick.getRawAxis(mThrottleAxisId);
    }

    @Override
    public double getTurn() {
        return -mTurnStick.getRawAxis(0);
    }

    @Override
    public boolean getQuickTurn() {
        return mTurnStick.getRawButton(1);
    }

    @Override
    public boolean getShoot() {
        if (mTurnStick.getButtonCount() > 2) return mTurnStick.getRawButton(2);
        return false;
    }

    @Override
    public boolean getWantsLowGear() {
        if (mThrottleStick.getButtonCount() > 2) return mThrottleStick.getRawButton(2);
        return false;
    }

    public boolean getThrust() {
        return mThrottleStick.getRawButton(1);
    }
}
