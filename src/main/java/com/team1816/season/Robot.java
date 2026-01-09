package com.team1816.season;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1816.lib.Singleton;
import com.team1816.lib.commands.SubsystemTestCommand;
import com.team1816.lib.events.PubSubHandler;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.util.Elastic;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private RobotContainer robotContainer;

    public Robot() {
    }

    double periodicLoopTime;
    private Command autonomousCommand;
    private final PubSubHandler pubsub = Singleton.get(PubSubHandler.class);
    private final LedManager.RobotLEDStatusEvent robotStatusEvent = pubsub.GetEvent(LedManager.RobotLEDStatusEvent.class);
    private final LedManager.RobotLEDStateEvent robotStateEvent = pubsub.GetEvent(LedManager.RobotLEDStateEvent.class);

    @Override
    public void robotInit() {
        try {
            GreenLogger.SilenceLoopOverrun(this);
            // used to serve elastic dashboards must be port 5800
            WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
            GreenLogger.periodicLog("timings/RobotLoop (ms)", () -> periodicLoopTime);
            robotContainer = new RobotContainer();
        } catch (Throwable t) {
            robotStatusEvent.Publish(LedManager.RobotLEDStatus.ERROR);
            GreenLogger.log(t);
        }
    }

    @Override
    public void disabledInit() {
        try {
            robotStatusEvent.Publish(LedManager.RobotLEDStatus.DISABLED);
            Elastic.selectTab("Autonomous");
        } catch (Throwable t) {
            robotStatusEvent.Publish(LedManager.RobotLEDStatus.ERROR);
            GreenLogger.log(t);
        }
    }

    @Override
    public void disabledPeriodic() {
        if(robotContainer == null) return;
        robotContainer.updateInitialPose();
    }

    @Override
    public void autonomousInit() {
        try {
            autonomousCommand = robotContainer.autoChooser.getSelected();
            // schedule the autonomous command
            if (autonomousCommand != null) {
                CommandScheduler.getInstance().schedule(autonomousCommand);
            }
            robotStatusEvent.Publish(LedManager.RobotLEDStatus.AUTONOMOUS);
        } catch (Throwable t) {
            robotStatusEvent.Publish(LedManager.RobotLEDStatus.ERROR);
            GreenLogger.log(t);
        }
    }

    @Override
    public void teleopInit() {
        try {
            if (autonomousCommand != null) {
                autonomousCommand.cancel();
            }
            robotStatusEvent.Publish(LedManager.RobotLEDStatus.ENABLED);
            Elastic.selectTab("Teleoperated");
        } catch (Throwable t) {
            robotStatusEvent.Publish(LedManager.RobotLEDStatus.ERROR);
            GreenLogger.log(t);
        }
    }

    @Override
    public void testInit() {
        try {
            CommandScheduler.getInstance().schedule(new SubsystemTestCommand());
        } catch (Throwable t) {
            robotStatusEvent.Publish(LedManager.RobotLEDStatus.ERROR);
            GreenLogger.log(t);
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            Threads.setCurrentThreadPriority(true, 99);
            double start = HALUtil.getFPGATime();
            CommandScheduler.getInstance().run();
            // update logs
            GreenLogger.updatePeriodic();
            double end = HALUtil.getFPGATime();
            periodicLoopTime = (end - start) / 1000;
        } catch (Throwable t) {
            robotStatusEvent.Publish(LedManager.RobotLEDStatus.ERROR);
            GreenLogger.log(t);
        }
    }
}
