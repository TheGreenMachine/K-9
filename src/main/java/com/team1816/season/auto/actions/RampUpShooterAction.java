package com.team1816.season.auto.actions;

import com.team1816.season.subsystems.Shooter;
import com.team1816.lib.auto.actions.Action;

public class RampUpShooterAction implements Action {
    @Override
    public void start() {
        Shooter.getInstance().startShooter();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
