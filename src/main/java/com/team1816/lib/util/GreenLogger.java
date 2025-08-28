package com.team1816.lib.util;

import com.pathplanner.lib.config.RobotConfig;
import com.team1816.lib.subsystems.IDrivetrain;
import com.team1816.season.Robot;
import com.team1816.season.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.function.Supplier;

public class GreenLogger {

    @SuppressWarnings("rawtypes")
    private static final HashMap<LogTopic, Supplier> periodicLogs = new HashMap<>();
    // using an empty string here to make the logs and live views consistent
    private static final NetworkTable netTable;
    private static final StringPublisher msg;

    /* Robot swerve drive state */
    private static StructPublisher<Pose2d> drivePose;
    private static StructPublisher<ChassisSpeeds> driveSpeeds;
    private static StructArrayPublisher<SwerveModuleState> driveModuleStates;
    private static StructArrayPublisher<SwerveModuleState> driveModuleTargets;
    private static StructArrayPublisher<SwerveModulePosition> driveModulePositions;
    private static DoublePublisher driveTimestamp;
    private static DoublePublisher driveOdometryFrequency;
    private static DoubleArrayPublisher fieldPub;
    private static StringPublisher fieldTypePub;
    private static final double[] poseArray = new double[3];

    static {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        // this will log the robot modes i.e., auto enabled estop
        DriverStation.startDataLog(DataLogManager.getLog(), false);
        // Log network tables then we can use advantage scope on a live robot
        // and use the same layout for the logs
        DataLogManager.logNetworkTables(true);
        // don't log console since we output to network tables
        DataLogManager.logConsoleOutput(false);
        netTable = NetworkTableInstance.getDefault().getTable("");
        msg = netTable.getStringTopic("messages").publish();
        driveSpeeds = netTable.getStructTopic(IDrivetrain.NAME + "/Speeds", ChassisSpeeds.struct).publish();
        driveModuleStates = netTable.getStructArrayTopic(IDrivetrain.NAME + "/ModuleStates", SwerveModuleState.struct).publish();
        driveModuleTargets = netTable.getStructArrayTopic(IDrivetrain.NAME + "/ModuleTargets", SwerveModuleState.struct).publish();
        driveModulePositions = netTable.getStructArrayTopic(IDrivetrain.NAME + "/ModulePositions", SwerveModulePosition.struct).publish();
        driveTimestamp = netTable.getDoubleTopic(IDrivetrain.NAME + "/Timestamp").publish();
        driveOdometryFrequency = netTable.getDoubleTopic(IDrivetrain.NAME + "/OdometryFrequency").publish();
        // name must be Robot for elastic to show as robot in UI
        fieldPub = netTable.getDoubleArrayTopic("Field/Robot").publish();
        fieldTypePub = netTable.getStringTopic("Field/.type").publish();
    }

    public static void SilenceLoopOverrun(Robot robot) {
        // Adjust loop overrun warning timeout
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(robot);
            watchdog.setTimeout(.2);
        } catch (Exception e) {
            log("Failed to disable loop overrun warnings.");
        }
        CommandScheduler.getInstance().setPeriod(.2);
    }

    public static void processPose2d(Pose2d pose) {
        // update Field NOTE: this format is deprecated for advantage scope but no support in simulator yet
        poseArray[0] = pose.getX();
        poseArray[1] = pose.getY();
        poseArray[2] = pose.getRotation().getDegrees();
        fieldTypePub.set("Field2d");
        fieldPub.set(poseArray);
    }

    public static void processSwerveState(IDrivetrain drivetrain) {
        if(drivetrain == null) return;
        var state = drivetrain.getState();
        driveSpeeds.set(state.Speeds);
        if(state.ModuleStates != null) {
            driveModuleStates.set(state.ModuleStates);
            driveModuleTargets.set(state.ModuleTargets);
            driveModulePositions.set(state.ModulePositions);
        }
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);
        processPose2d(state.Pose);
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
        } else if (result instanceof NTSendable) {
            pub = null;
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
        } else  if (s instanceof RobotConfig) {
            log("Pathplanner config:");
            var config = (RobotConfig) s;
            log(String.format("  Holonomic: %b",config.isHolonomic));
            log(String.format("  MOI: %f",config.MOI));
            log(String.format("  massKg: %f",config.massKG));
            log(String.format("  maxTorqueFriction: %f",config.maxTorqueFriction));
            log(String.format("  wheelFrictionForce: %f",config.wheelFrictionForce));
            log(String.format("  maxDriveVelocityMPS: %f",config.moduleConfig.maxDriveVelocityMPS));
            log(String.format("  driveCurrentLimit: %f",config.moduleConfig.driveCurrentLimit));
            log(String.format("  wheelRadiusMeters: %f",config.moduleConfig.wheelRadiusMeters));
            log(String.format("  wheelCOF: %f",config.moduleConfig.wheelCOF));
            log(String.format("  freeSpeedRadPerSec: %f",config.moduleConfig.driveMotor.freeSpeedRadPerSec));
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
            } else if (entry.Publisher == null) {
                var value = (NTSendable) supplier.get();
                SmartDashboard.putData(value);
            } else {
                var value = String.valueOf(supplier.get());
                ((StringPublisher) entry.Publisher).set(value);
            }
        }
        // log the current drive state
        processSwerveState(RobotContainer.drivetrain);
    }
}
