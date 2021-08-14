package com.team1816.season.controlboard;

import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.lib.controlboard.LogitechController;
import com.team1816.season.Constants;

import static com.team1816.season.controlboard.ControlUtils.PressAction.getControllerInstance;

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
        mController = getControllerInstance(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return mController.getJoystick(
            Controller.Side.LEFT,
            Controller.Axis.Y
        );
    }

    @Override
    public double getTurn() {
        return mController.getJoystick(
            Controller.Side.RIGHT,
            Controller.Axis.X
        );
    }

    @Override
    public boolean getSlowMode() {
        return mController.getTrigger(Controller.Side.RIGHT);
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return mController.getButton(Controller.Button.Y);
    }

    @Override
    public boolean getQuickTurn() {
        return mController.getButton(Controller.Button.R_JOYSTICK);
    }

    @Override
    public boolean getCollectorToggle() {
        return mController.getButton(Controller.Button.LB);
    }

    @Override
    public boolean getCollectorUp() {
        return mController.getButton(Controller.Button.RB);
    }

    @Override
    public boolean getFeederToTrenchSpline() {
        return mController.getButton(Controller.Button.X);
    }

    @Override
    public boolean getTrenchToFeederSpline() {
        return mController.getButton(Controller.Button.B);
    }

    @Override
    public boolean getBrakeMode() {
        return mController.getButton(Controller.Button.A);
    }

    @Override
    public int getDriverClimber() {
        switch (mController.getDPad()) {
            case 0:
            case 45:
            case 315:
                return 1;
            case 180:
            case 135:
            case 225:
                return -1;
            default:
                return 0;
        }
    }
}
