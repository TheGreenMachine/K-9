package com.team1816.lib.commands;

import com.team1816.lib.Singleton;
import com.team1816.lib.events.PubSubHandler;
import edu.wpi.first.wpilibj2.command.Command;

public class GreenCommand extends Command {
    protected PubSubHandler pubsub = Singleton.get(PubSubHandler.class);
}
