package com.team1816.lib.util.logUtil;

import com.team1816.season.Constants;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

/**
 * The universal project-wide message logging wrapper.
 * This class utilizes WPI's DataLogManager and is a wrapper for the standard GreenLogger.log()
 *
 * @see DataLogManager
 */
public class GreenLogger {

    public  GreenLogger(){
    }

    private static final HashMap<String, DataLogEntry> dynamicLogs = new HashMap<>();

    private static final List<PeriodicLog> periodicLogs = new ArrayList<>();

    public static void log(double d) {
        if (Constants.kLoggingRobot) {
            DataLogManager.log(String.valueOf(d));
        }
    }
    /**
     * Logs a string message
     *
     * @param s message
     */
    public static void log(String s) {
        if (Constants.kLoggingRobot) {
            DataLogManager.log(s);
        }
    }

    /**
     * Logs a boolean message
     *
     * @param b message
     */
    public static void log(boolean b) {
        if (Constants.kLoggingRobot) {
            DataLogManager.log(String.valueOf(b));
        }
    }

    /**
     * Logs an exception message
     *
     * @param e exception
     */
    public static void log(Exception e) {
        GreenLogger.log(e.getMessage());
    }

    /**
     * Dynamic Log Appending
     */

    public static void appendQuickLog(String logName, double value) {
        DataLogEntry entry = dynamicLogs.get(logName);
        dynamicLogs.putIfAbsent(logName, new DoubleLogEntry(DataLogManager.getLog(), "GreenLogs/" + logName));
        if (entry instanceof DoubleLogEntry) ((DoubleLogEntry) entry).append(value);
    }

    public static void appendQuickLog(String logName, double... value) {
        DataLogEntry entry = dynamicLogs.get(logName);
        dynamicLogs.putIfAbsent(logName, new DoubleArrayLogEntry(DataLogManager.getLog(), "GreenLogs/" + logName));
        if (entry instanceof DoubleArrayLogEntry) ((DoubleArrayLogEntry) entry).append(value);
    }

    public static void appendQuickLog(String logName, boolean value) {
        DataLogEntry entry = dynamicLogs.get(logName);
        dynamicLogs.putIfAbsent(logName, new BooleanLogEntry(DataLogManager.getLog(), "GreenLogs/" + logName));
        if (entry instanceof BooleanLogEntry) ((BooleanLogEntry) entry).append(value);

    }

    /**
     * Adds logEntries that will be updated every robot loop
     * @param logEntry  the WPI logger to be used for creating log
     * @param supplier the method that provides the current value
     */
    public static void AddPeriodicLog(DataLogEntry logEntry, Supplier<?> supplier){
        if(!Constants.kLoggingRobot) return;
        periodicLogs.add(new PeriodicLog(logEntry,supplier) );
    }
    /**
     * Updates the values of registered periodic logs
     */
    public static void UpdatePeriodicLogs(){
        for (PeriodicLog log : periodicLogs){
            log.UpdateLog();
        }
    }

}
