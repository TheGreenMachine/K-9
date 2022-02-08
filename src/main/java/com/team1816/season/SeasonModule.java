package com.team1816.season;

import com.google.inject.AbstractModule;
import com.team1816.lib.controlboard.IButtonControlBoard;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.season.auto.actions.AutoAimAction;
import com.team1816.season.auto.actions.TurretAction;
import com.team1816.season.controlboard.ControlBoard;
import com.team1816.season.controlboard.GamepadButtonControlBoard;
import com.team1816.season.controlboard.GamepadDriveControlBoard;
import com.team1816.season.subsystems.Camera;
import com.team1816.season.subsystems.Drive;
import com.team1816.season.subsystems.TankDrive;
import com.team1816.season.subsystems.Turret;

public class SeasonModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Drive.Factory.class).to(SeasonFactory.class);
        bind(IControlBoard.class).to(ControlBoard.class);
        bind(IDriveControlBoard.class).to(GamepadDriveControlBoard.class);
        bind(IButtonControlBoard.class).to(GamepadButtonControlBoard.class);
        requestStaticInjection(Drive.class);
        requestStaticInjection(TankDrive.class);
        requestStaticInjection(Camera.class);
        requestStaticInjection(Turret.class);
        requestStaticInjection(TurretAction.class);
        requestStaticInjection(AutoAimAction.class);
    }
}
