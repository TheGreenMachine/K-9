package com.team1816.frc2019.controlboard;

import com.team1816.frc2019.Constants;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.lib.controlboard.LogitechController;

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard mInstance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    private final Controller mController;

    private GamepadDriveControlBoard() {
        mController = new LogitechController(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return mController.getJoystick(LogitechController.Side.LEFT, LogitechController.Axis.Y);
    }

    @Override
    public double getTurn() {
        return mController.getJoystick(LogitechController.Side.RIGHT, LogitechController.Axis.X);
    }

    @Override
    public boolean getWantsLowGear() {
        return false;
    }

    @Override
    public boolean getThrust() {
        return false;
    }

    @Override
    public boolean getQuickTurn() {
        return mController.getTrigger(LogitechController.Side.LEFT);
    }

    @Override
    public boolean getShoot() {
        return false;
    }
}
