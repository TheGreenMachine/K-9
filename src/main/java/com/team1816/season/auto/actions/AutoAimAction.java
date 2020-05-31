package com.team1816.season.auto.actions;

import com.team1816.season.subsystems.Camera;
import com.team1816.season.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;

public class AutoAimAction implements Action {
    private Turret turret;
    private Camera camera;

    public AutoAimAction() {
        turret = Turret.getInstance();
        camera = Camera.getInstance();
    }

    @Override
    public void start() {
        turret.setControlMode(Turret.ControlMode.CAMERA_FOLLOWING);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return camera.getDeltaXAngle() < Camera.ALLOWABLE_AIM_ERROR;
    }

    @Override
    public void done() {

    }
}
