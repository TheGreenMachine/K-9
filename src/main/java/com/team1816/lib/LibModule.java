package com.team1816.lib;

import com.google.inject.AbstractModule;
import com.team1816.lib.controlboard.*;
import com.team1816.season.controlboard.ControlUtils;

public class LibModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Controller.Factory.class).to(ControlUtils.class);
    }
}
