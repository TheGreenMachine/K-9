package com.team1816.lib.commands;

import com.team1816.lib.Singleton;
import com.team1816.lib.events.PubSubConsumer;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class SubsystemTestCommand extends GreenCommand {

    private final LedManager.RobotLEDStatusEvent robotStatusEvent = pubsub.GetEvent(LedManager.RobotLEDStatusEvent.class);
    private final LedManager.RobotLEDStateEvent robotStateEvent = pubsub.GetEvent(LedManager.RobotLEDStateEvent.class);
    private final SequentialCommandGroup group = new SequentialCommandGroup();
    public static class TestResult extends PubSubConsumer<Boolean> {}
    private boolean passed = true;

    @Override
    public void initialize() {
        GreenLogger.log("Subsystem Tests starting");
        pubsub.GetEvent(TestResult.class).Subscribe(this::TestResult);
        // blink leds to warn humans of movement
        robotStateEvent.Publish(LedManager.LEDControlState.BLINK);
        group.addCommands(new WaitCommand(3));
        group.addCommands(runOnce(()->{
            robotStateEvent.Publish(LedManager.LEDControlState.SOLID);
        }));
        // add all the substem tests to a sequential group
        for (ITestableSubsystem sys : Singleton.getSubSystems()) {
            var cmd = sys.TestSubsystem();
            // if a subsystem has a command add it
            if(cmd != null) group.addCommands(cmd);
        }
        group.addCommands(runOnce(()->{
            if (passed) {
                GreenLogger.log("ALL SYSTEMS PASSED");
                robotStatusEvent.Publish(LedManager.RobotLEDStatus.ENABLED);
            } else {
                GreenLogger.log("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
                robotStateEvent.Publish(LedManager.LEDControlState.BLINK);
                robotStatusEvent.Publish(LedManager.RobotLEDStatus.ERROR);
            }
        }));
        group.schedule();
    }

    private void TestResult(Boolean testPassed) {
        passed = passed && testPassed;
    }
}
