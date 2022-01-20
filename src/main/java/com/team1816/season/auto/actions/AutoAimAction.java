package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.subsystems.Camera;
import com.team1816.season.subsystems.Turret;

public class AutoAimAction implements Action {

    @Inject
    private static Turret turret;

    @Inject
    private static Camera camera;

    public AutoAimAction() {}

    @Override
    public void start() {
        turret.setControlMode(Turret.ControlMode.CAMERA_FOLLOWING);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return camera.getDeltaXAngle() < Camera.ALLOWABLE_AIM_ERROR;
    }

    @Override
    public void done() {}
}
