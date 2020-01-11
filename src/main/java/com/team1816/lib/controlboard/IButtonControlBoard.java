package com.team1816.lib.controlboard;

public interface IButtonControlBoard {
    void reset();

    void setRumble(boolean on);

    // Climbing
    boolean getToggleHangMode();
    boolean getToggleHangModeLow();

    boolean getBeakOpen();
    boolean getBeakClose();

    double getClimberThrottle();

    boolean getShooterOut();
    boolean getShooterIn();

    boolean getShooterPositionUp();
    boolean getShooterPositionRocket();
    boolean getCollectingMode();

}
