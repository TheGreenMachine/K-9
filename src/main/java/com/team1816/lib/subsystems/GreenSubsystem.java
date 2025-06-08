package com.team1816.lib.subsystems;

import com.team1816.lib.Singleton;
import com.team1816.lib.events.PubSubHandler;
import com.team1816.lib.hardware.factory.RobotFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class GreenSubsystem extends SubsystemBase {


    public static RobotFactory factory = Singleton.get(RobotFactory.class);
    public PubSubHandler pubsub = Singleton.get(PubSubHandler.class);

    public abstract Command TestSubsystem();

}
