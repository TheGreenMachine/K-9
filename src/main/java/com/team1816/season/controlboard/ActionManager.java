package com.team1816.season.controlboard;

import java.util.Arrays;
import java.util.List;

public class ActionManager {

    private final List<ControlUtils.ButtonAction> actions;

    public ActionManager(ControlUtils.ButtonAction... actions) {
        this.actions = Arrays.asList(actions);
    }

    public void update() {
        actions.forEach(ControlUtils.ButtonAction::update);
    }
}
