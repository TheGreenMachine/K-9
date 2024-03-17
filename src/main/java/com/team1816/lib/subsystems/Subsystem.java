package com.team1816.lib.subsystems;

import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.Robot;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.function.Supplier;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * initializing all member components at the start of the match.
 */
public abstract class Subsystem implements Sendable {

    private final String name;
    public static RobotFactory factory = Robot.getFactory();
    private final DataLog log = DataLogManager.getLog();

    protected Subsystem(String name) {
        this.name = name;
        SendableRegistry.addLW(this, name, name);
    }

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readFromHardware() {}

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writeToHardware() {}

    public void registerEnabledLoops(ILooper mEnabledLooper) {}

    public void zeroSensors() {}

    public abstract void stop();

    public abstract boolean checkSystem();

    public void CreateSubSystemLog(
        String topicName,
        Supplier<?> supplier
    ) {
        if (factory.getSubsystem(name).implemented) {
            var logName = name.substring(0, 1).toUpperCase() + name.substring(1) + "/" + topicName;
            var cls = supplier.get().getClass();
            if( cls == Double.class) {
                GreenLogger.AddPeriodicLog(new DoubleLogEntry(log, logName), supplier);
            } else if (cls == Integer.class ) {
                GreenLogger.AddPeriodicLog(new IntegerLogEntry(log, logName), supplier);
            } else if (cls == String.class) {
                GreenLogger.AddPeriodicLog(new StringLogEntry(log, logName), supplier);
            } else if (cls == Boolean.class) {
                GreenLogger.AddPeriodicLog(new BooleanLogEntry(log, logName), supplier);
            }
        }
    }

    @Deprecated
    public void outputTelemetry() {}

    @Override
    public void initSendable(SendableBuilder builder) {}

    public String getSubsystemName() {
        return name;
    }

    public boolean isImplemented() {
        return factory.getSubsystem(name).implemented;
    }
}
