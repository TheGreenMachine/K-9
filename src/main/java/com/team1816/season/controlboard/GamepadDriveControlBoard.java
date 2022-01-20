package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.Controller.Axis;
import com.team1816.lib.controlboard.Controller.Button;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.season.Constants;

@Singleton
public class GamepadDriveControlBoard implements IDriveControlBoard {

    private final Controller mController;

    @Inject
    private GamepadDriveControlBoard(Controller.Factory controller) {
        mController = controller.getControllerInstance(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return mController.getJoystick(Axis.LEFT_Y);
    }

    @Override
    public double getTurn() {
        return mController.getJoystick(Axis.RIGHT_X);
    }

    @Override
    public double getStrafe() {
        return 0;
    }

    @Override
    public boolean getSlowMode() {
        return mController.getTrigger(Axis.RIGHT_TRIGGER);
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return mController.getButton(Button.Y);
    }

    @Override
    public boolean getQuickTurn() {
        return mController.getButton(Button.R_JOYSTICK);
    }

    @Override
    public boolean getCollectorToggle() {
        return mController.getButton(Button.LEFT_BUMPER);
    }

    @Override
    public boolean getCollectorUp() {
        return mController.getButton(Button.RIGHT_BUMPER);
    }

    @Override
    public boolean getFeederToTrenchSpline() {
        return mController.getButton(Button.X);
    }

    @Override
    public boolean getTrenchToFeederSpline() {
        return mController.getButton(Button.B);
    }

    @Override
    public boolean getBrakeMode() {
        return mController.getButton(Button.A);
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
