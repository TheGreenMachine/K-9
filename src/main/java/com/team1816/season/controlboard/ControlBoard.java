package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.team1816.lib.controlboard.IButtonControlBoard;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.IDriveControlBoard;

public class ControlBoard implements IControlBoard {

    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    @Inject
    private ControlBoard(
        IDriveControlBoard driveControlBoard,
        IButtonControlBoard buttonControlBoard
    ) {
        mDriveControlBoard = driveControlBoard;
        mButtonControlBoard = buttonControlBoard;
    }

    @Override
    public void reset() {}

    // Drive Control Board
    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getSlowMode() {
        return mDriveControlBoard.getSlowMode();
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return mDriveControlBoard.getDrivetrainFlipped();
    }

    @Override
    public boolean getCollectorToggle() {
        return mDriveControlBoard.getCollectorToggle();
    }

    @Override
    public boolean getCollectorUp() {
        return mDriveControlBoard.getCollectorUp();
    }

    @Override
    public boolean getFeederToTrenchSpline() {
        return mDriveControlBoard.getFeederToTrenchSpline();
    }

    @Override
    public boolean getTrenchToFeederSpline() {
        return mDriveControlBoard.getTrenchToFeederSpline();
    }

    @Override
    public boolean getBrakeMode() {
        return false;
    }

    @Override
    public int getDriverClimber() {
        return mDriveControlBoard.getDriverClimber();
    }

    // Button Control Board
    @Override
    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    @Override
    public boolean getSpinnerThreeTimes() {
        return mButtonControlBoard.getSpinnerThreeTimes();
    }

    @Override
    public boolean getTurretJogLeft() {
        return mButtonControlBoard.getTurretJogLeft();
    }

    @Override
    public boolean getTurretJogRight() {
        return mButtonControlBoard.getTurretJogRight();
    }

    @Override
    public boolean getFieldFollowing() {
        return mButtonControlBoard.getFieldFollowing();
    }

    @Override
    public boolean getAutoAim() {
        return mButtonControlBoard.getAutoAim();
    }

    @Override
    public boolean getShoot() {
        return mButtonControlBoard.getShoot();
    }

    @Override
    public boolean getSpinnerColor() {
        return mButtonControlBoard.getSpinnerColor();
    }

    @Override
    public boolean getSpinnerReset() {
        return mButtonControlBoard.getSpinnerReset();
    }

    @Override
    public boolean getFeederFlapOut() {
        return mButtonControlBoard.getFeederFlapOut();
    }

    @Override
    public boolean getFeederFlapIn() {
        return mButtonControlBoard.getFeederFlapIn();
    }

    @Override
    public double getClimber() {
        return mButtonControlBoard.getClimber();
    }

    @Override
    public boolean getCollectorBackSpin() {
        return mButtonControlBoard.getCollectorBackSpin();
    }

    @Override
    public boolean getClimberDeploy() {
        return mButtonControlBoard.getClimberDeploy();
    }
}
