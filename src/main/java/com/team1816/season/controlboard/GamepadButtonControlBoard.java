package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.Controller.Axis;
import com.team1816.lib.controlboard.Controller.Button;
import com.team1816.lib.controlboard.IButtonControlBoard;
import com.team1816.season.Constants;
import com.team254.lib.util.DelayedBoolean;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class GamepadButtonControlBoard implements IButtonControlBoard {

    private final double kDeadband = 0.15;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;

    private final Controller mController;

    @Inject
    private GamepadButtonControlBoard(Controller.Factory controller) {
        mController = controller.getControllerInstance(Constants.kButtonGamepadPort);
        reset();
    }

    @Override
    public void setRumble(boolean on) {
        mController.setRumble(on);
    }

    @Override
    public void reset() {
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }

    // Spinner
    @Override
    public boolean getSpinnerReset() {
        return mController.getButton(Button.START);
    }

    @Override
    public boolean getSpinnerColor() {
        return mController.getButton(Button.X);
    }

    @Override
    public boolean getSpinnerThreeTimes() {
        return mController.getButton(Button.B);
    }

    // Turret
    @Override
    public boolean getTurretJogLeft() {
        return mController.getDPad() == 270;
    }

    @Override
    public boolean getTurretJogRight() {
        return mController.getDPad() == 90;
    }

    @Override
    public boolean getFieldFollowing() {
        return mController.getDPad() == 180;
    }

    // Feeder Flap
    @Override
    public boolean getFeederFlapOut() {
        return mController.getButton(Button.Y);
    }

    @Override
    public boolean getFeederFlapIn() {
        return mController.getButton(Button.A);
    }

    @Override
    public double getClimber() {
        return -mController.getJoystick(Axis.LEFT_X);
    }

    @Override
    public boolean getShoot() {
        return mController.getTrigger(Axis.RIGHT_TRIGGER);
    }

    @Override
    public boolean getAutoAim() {
        return mController.getButton(Button.LEFT_BUMPER);
    }

    @Override
    public boolean getCollectorBackSpin() {
        return mController.getButton(Button.RIGHT_BUMPER);
    }

    @Override
    public boolean getClimberDeploy() {
        return mController.getDPad() == 0;
    }
}
