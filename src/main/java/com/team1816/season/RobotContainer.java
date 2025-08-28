package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.BaseRobotContainer;
import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.Turret;

public class RobotContainer extends BaseRobotContainer {
    public RobotContainer() {
        Singleton.CreateSubSystem(Turret.class);
        NamedCommands.registerCommand("InTheZone", new InTheZoneCommand());
        // call the base to initialize library objects
        // i.e. subsystems that always exist like the drivetrain and path planner
        InitializeLibSubSystems();
    }
}
