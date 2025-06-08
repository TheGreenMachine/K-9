package com.team1816.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.HashMap;
import java.util.function.Supplier;

public class GreenLogger {

    @SuppressWarnings("rawtypes")
    private static final HashMap<LogTopic, Supplier> periodicLogs = new HashMap<>();
    // using an empty string here to make the logs and live views consistent
    private static final NetworkTable netTable;
    private static final StringPublisher msg;

    static {
        DriverStation.silenceJoystickConnectionWarning(true);
        // this will log the robot modes i.e., auto enabled estop
        DriverStation.startDataLog(DataLogManager.getLog(), false);
        // Log network tables then we can use advantage scope on a live robot
        // and use the same layout for the logs
        DataLogManager.logNetworkTables(true);
        // don't log console since we output to network tables
        DataLogManager.logConsoleOutput(false);
        netTable = NetworkTableInstance.getDefault().getTable("");
        msg = netTable.getStringTopic("messages").publish();
    }

    // adds a new periodic log
    public static <T> void periodicLog(String name, Supplier<T> supplier) {
        T result = supplier.get();
        Publisher pub;
        if (result instanceof Double) {
            pub = netTable.getDoubleTopic(name).publish();
        } else if (result instanceof Integer) {
            pub = netTable.getIntegerTopic(name).publish();
        } else if (result instanceof Boolean) {
            pub = netTable.getBooleanTopic(name).publish();
        } else if (result instanceof Pose2d) {
            pub = netTable.getStructTopic(name, Pose2d.struct).publish();
        } else {
            pub = netTable.getStringTopic(name).publish();
        }
        periodicLogs.put(new LogTopic(pub), supplier);
    }

    public static void log(Object s) {
        if (s instanceof Throwable throwable) {
            // print only the cause of the error
            while (throwable.getCause() != null) {
                throwable = throwable.getCause();
            }
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            throwable.printStackTrace(printWriter);
            var value = stringWriter.toString().replace("\r", "");
            msg.set(value);
            DataLogManager.log(value);
            Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, throwable.getClass().getName(), throwable.getMessage(), 15000, 450, -1));
        } else {
            var value = String.valueOf(s);
            msg.set(value);
            DataLogManager.log(value);
        }
    }

    // Will update all registered periodic loggers
    @SuppressWarnings({"unchecked", "rawtypes"})
    public static void updatePeriodic() {
        for (LogTopic entry : periodicLogs.keySet()) {
            var supplier = periodicLogs.get(entry);
            if (entry.Publisher instanceof DoublePublisher) {
                var value = (Double) supplier.get();
                ((DoublePublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof IntegerPublisher) {
                var value = (Integer) supplier.get();
                ((IntegerPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof BooleanPublisher) {
                var value = (Boolean) supplier.get();
                ((BooleanPublisher) entry.Publisher).set(value);
            } else if (entry.Publisher instanceof StructPublisher) {
                var value = supplier.get();
                ((StructPublisher) entry.Publisher).set(value);
            } else {
                var value = String.valueOf(supplier.get());
                ((StringPublisher) entry.Publisher).set(value);
            }
        }
    }
}
