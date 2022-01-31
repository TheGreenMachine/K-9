package com.team1816.lib;

import com.google.inject.AbstractModule;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitUntilInsideRegion;
import com.team1816.lib.controlboard.Controller;
import com.team1816.season.controlboard.ControlUtils;

public class LibModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Controller.Factory.class).to(ControlUtils.class);
        requestStaticInjection(TrajectoryAction.class);
        requestStaticInjection(WaitUntilInsideRegion.class);
    }
}
