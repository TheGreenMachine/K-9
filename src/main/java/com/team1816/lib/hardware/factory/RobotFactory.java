package com.team1816.lib.hardware.factory;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.team1816.lib.hardware.*;
import com.team1816.lib.hardware.components.ICTREDevice;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.lib.hardware.components.gyro.Pigeon2Impl;
import com.team1816.lib.hardware.components.led.CANdleImpl;
import com.team1816.lib.hardware.components.led.CANifierImpl;
import com.team1816.lib.hardware.components.motor.TalonFXImpl;
import com.team1816.lib.hardware.components.motor.TalonFXSImpl;
import com.team1816.lib.hardware.components.sensor.CANCoderImpl;
import com.team1816.lib.hardware.components.sensor.CanRangeImpl;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.Map;

import static com.team1816.lib.Singleton.factory;
import static com.team1816.lib.hardware.components.motor.WpiMotorUtil.getModuleName;
import static com.team1816.lib.util.FormatUtils.GetDisplay;


public class RobotFactory {

    private RobotConfiguration config;
    public boolean RobotIsReal; // Use to detect real or simulation public to override for tests
    public final static int StartingGhostId = 50;
    private int lastGhostId = StartingGhostId;

    public RobotFactory() {
        RobotIsReal = RobotBase.isReal();
        var robotName = System.getenv("ROBOT_NAME");
        if (robotName == null) {
            GreenLogger.log("ROBOT_NAME environment variable not defined using test yaml");
            robotName = "test";
        }
        robotName = robotName.toLowerCase();
        GreenLogger.log("Loading Config for " + robotName);
        try {
            config = YamlConfig.loadFrom(this.getClass().getClassLoader().getResourceAsStream("yaml/" + robotName + ".yml")
            );
        } catch (Exception e) {
            GreenLogger.log("Yaml Config error!" + e.getMessage());
        }
    }

    public Map<String, Double> getConstants() {
        return config.constants;
    }

    public SubsystemConfig getSubsystemConfig(String subsystemName) {
        var subsystem = config.subsystems.get(subsystemName);
        if (subsystem == null) {
            subsystem = new SubsystemConfig();
            subsystem.implemented = false;
        }
        subsystem.name = subsystemName;
        return subsystem;
    }

    public double getConstant(String name, double defaultVal) {
        if (getConstants() == null || !getConstants().containsKey(name)) {
            DriverStation.reportWarning("Yaml constants:" + name + " missing", true);
            return defaultVal;
        }
        return getConstants().get(name);
    }

    public double getConstant(String subsystemName, String name, double defaultVal) {
        return getConstant(subsystemName, name, defaultVal, defaultVal == 0);
    }

    public double getConstant(
        String subsystemName,
        String name,
        double defaultVal,
        boolean showWarning
    ) {
        if (config == null || !getSubsystemConfig(subsystemName).implemented) {
            return defaultVal;
        }
        if (
            getSubsystemConfig(subsystemName).constants == null ||
                !getSubsystemConfig(subsystemName).constants.containsKey(name)
        ) {
            if (showWarning) {
                DriverStation.reportWarning(
                    "Yaml: subsystem \"" +
                        subsystemName +
                        "\" constant \"" +
                        name +
                        "\" missing",
                    defaultVal == 0
                );
            }
            return defaultVal;
        }
        return getSubsystemConfig(subsystemName).constants.get(name);
    }

    public String getDefaultAuto() {
        return config.defaultAuto;
    }

    public IPhoenix6 getDevice(String subsystemName, String deviceName) {
        if (config == null) {
            throw new NullPointerException("config is null");
        }
        var subsystem = getSubsystemConfig(subsystemName);

        // if subsystem is not implemented we don't need anything from it
        if (!subsystem.implemented) return null;

        // we should have devices defined verify that
        if (subsystem.devices == null) {
            throw new IllegalArgumentException("Devices not defined for " + subsystemName);
        }
        // ensure device is defined in yaml
        var deviceConfig = subsystem.devices.get(deviceName);
        if (deviceConfig == null) {
            throw new IllegalArgumentException("Device " + deviceName + " not found");
        }
        deviceConfig.name = deviceName;
        // ensure device type is set
        if (deviceConfig.deviceType == null) {
            throw new IllegalArgumentException("deviceType missing in yml");
        }
        return getDevInst(deviceConfig, subsystem, true);
    }

    // Used to get devices by there id  Used by CTRE swerve
    public IPhoenix6 getDeviceById(String subsystemName, int id) {
        var config = factory.getSubsystemConfig(subsystemName);
        for (var key : config.devices.keySet()) {
            var device = config.devices.get(key);
            if (device.id == id) {
                // ensure the name is set
                if (device.name == null) device.name = key;
                return getDevInst(device, config, true);
            }
        }
        return null;
    }

    // Instantiates device based on the type and applies the configuration
    private IPhoenix6 getDevInst(DeviceConfiguration deviceConfig, SubsystemConfig subsystemConfig, boolean logDetails) {
        IPhoenix6 devInst = null;
        var bus = subsystemConfig.canBusName;
        if (!subsystemConfig.implemented || deviceConfig.id > StartingGhostId) {
            GreenLogger.log("Device " +  deviceConfig.name + " not implemented ghosting");
            bus = "ghost";
        } else {
            GreenLogger.log("Creating " + deviceConfig.name);
        }
        GreenLogger.log("  id: " + deviceConfig.id);
        GreenLogger.log("  deviceType: " + deviceConfig.deviceType);

        switch (deviceConfig.deviceType) {
            case TalonFX -> {
                if (devInst == null) devInst = new TalonFXImpl(deviceConfig.id, subsystemConfig.canBusName);
            }
            case TalonFXS -> {
                devInst = new TalonFXSImpl(deviceConfig.id, bus);
            }
            case Pigeon2 -> {
                if (devInst == null) devInst = new Pigeon2Impl(deviceConfig.id, subsystemConfig.canBusName);
            }
            case CANdle -> {
                if (devInst == null) devInst = new CANdleImpl(deviceConfig.id, subsystemConfig.canBusName);
            }
            case CANrange -> {
                if (devInst == null) devInst = new CanRangeImpl(deviceConfig.id, subsystemConfig.canBusName);
            }
            case CANifier -> {
                if (devInst == null) devInst = new CANifierImpl(deviceConfig.id);
            }
            case CANcoder -> {
                if (devInst == null) devInst = new CANCoderImpl(deviceConfig.id, subsystemConfig.canBusName);
            }
            default -> {
                GreenLogger.log("Device type " + deviceConfig.deviceType + " not implemented");
            }
        }
        var parentConfig = getCTREConfig(subsystemConfig, deviceConfig);
        if (parentConfig != null) {
            var logPath = subsystemConfig.name + "/" + deviceConfig.name + "/";
            // apply the configuration
            devInst.applyConfiguration(parentConfig, logPath, logDetails);
        }
        return devInst;
    }

    // Takes YAML Device configuration and creates a CTRE configuration object
    private ParentConfiguration getCTREConfig(SubsystemConfig subsystemConfig, DeviceConfiguration deviceConfig) {
        ParentConfiguration parentConfig = null;
        switch (deviceConfig.deviceType) {
            case CANifier -> {
                // this is phoenix 5 there is no config
            }
            case TalonFX -> {
                parentConfig = new TalonFXConfiguration();
            }
            case TalonFXS -> {
                parentConfig = new TalonFXSConfiguration();
            }
            case Pigeon2 -> {
                parentConfig = new Pigeon2Configuration();
            }
            case CANdle -> {
                parentConfig = new CANdleConfiguration();
            }
            case CANrange -> {
                parentConfig = new CANrangeConfiguration();
            }
            case CANcoder -> {
                parentConfig = new CANcoderConfiguration();
            }
            default -> {
                GreenLogger.log("Unknown CTRE configuration for deviceType: " + deviceConfig.deviceType);
            }
        }
        if (parentConfig != null) {
            // Update defaults with yaml values
            ApplyYamlConfigs(subsystemConfig, deviceConfig, parentConfig);
        }
        return parentConfig;
    }

    private void ApplyYamlConfigs(SubsystemConfig subsystemConfig, DeviceConfiguration deviceConfig, ParentConfiguration parentConfig) {
        switch (deviceConfig.deviceType) {
            case TalonFX -> {
                var clazz = (TalonFXConfiguration) parentConfig;
                clazz.MotorOutput = GetMotorOutputConfigs(deviceConfig);
                clazz.Slot0 = Slot0Configs.from(GetSlotConfigs(subsystemConfig.pidConfig, 0));
                clazz.Slot1 = Slot1Configs.from(GetSlotConfigs(subsystemConfig.pidConfig, 1));
                clazz.Slot2 = Slot2Configs.from(GetSlotConfigs(subsystemConfig.pidConfig, 2));
                clazz.CurrentLimits = GetCurrentConfigs(deviceConfig);
                clazz.SoftwareLimitSwitch = GetSoftLimitConfigs(deviceConfig);
                clazz.Feedback = GetFeedbackConfigs(deviceConfig);
            }
            case TalonFXS -> {
                var clazz = (TalonFXSConfiguration) parentConfig;
                clazz.MotorOutput = GetMotorOutputConfigs(deviceConfig);
                clazz.Commutation = GetCommunicationConfigs(deviceConfig);
                clazz.Slot0 = Slot0Configs.from(GetSlotConfigs(subsystemConfig.pidConfig, 0));
                clazz.Slot1 = Slot1Configs.from(GetSlotConfigs(subsystemConfig.pidConfig, 1));
                clazz.Slot2 = Slot2Configs.from(GetSlotConfigs(subsystemConfig.pidConfig, 2));
                clazz.CurrentLimits = GetCurrentConfigs(deviceConfig);
                clazz.SoftwareLimitSwitch = GetSoftLimitConfigs(deviceConfig);
                clazz.ExternalFeedback = GetExternalFeedbackConfigs(deviceConfig);
            }
            case Pigeon2 -> {
            }
            case CANdle -> {
                var clazz = (CANdleConfiguration) parentConfig;
                clazz.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
                clazz.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
                clazz.LED.StripType = StripTypeValue.BRG;
            }
            case CANifier -> {
            }
            case CANrange -> {
                var clazz = (CANrangeConfiguration) parentConfig;
            }
        }
    }

    private FeedbackConfigs GetFeedbackConfigs(DeviceConfiguration deviceConfig) {
        var config = new FeedbackConfigs();
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.remoteSensor != null) {
            switch (deviceConfig.remoteSensor) {
                case RemoteCANcoder ->  config.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            }
        }
        GreenLogger.log("  remoteSensor: " + config.FeedbackSensorSource);

        if (deviceConfig.remoteSensorId != null) {
            config.FeedbackRemoteSensorID = deviceConfig.remoteSensorId;
            GreenLogger.log("  remoteSensorId: " + config.FeedbackRemoteSensorID);
        }

        if (deviceConfig.remoteOffest != null) {
            config.FeedbackRotorOffset = deviceConfig.remoteOffest;
            GreenLogger.log("  remoteOffest: " + config.FeedbackRotorOffset);
        }
        return config;
    }

    private ExternalFeedbackConfigs GetExternalFeedbackConfigs(DeviceConfiguration deviceConfig) {
        var config = new ExternalFeedbackConfigs();
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.remoteSensor != null) {
            config.ExternalFeedbackSensorSource = deviceConfig.remoteSensor;
        }
        GreenLogger.log("  remoteSensor: " + config.ExternalFeedbackSensorSource);

        if (deviceConfig.remoteSensorId != null) {
            config.FeedbackRemoteSensorID = deviceConfig.remoteSensorId;
            GreenLogger.log("  remoteSensorId: " + config.FeedbackRemoteSensorID);
        }

        if (deviceConfig.remoteOffest != null) {
            config.AbsoluteSensorOffset = deviceConfig.remoteOffest;
            GreenLogger.log("  remoteOffest: " + config.AbsoluteSensorOffset);
        }

        return config;
    }

    private SoftwareLimitSwitchConfigs GetSoftLimitConfigs(DeviceConfiguration deviceConfig) {
        var config = new SoftwareLimitSwitchConfigs();
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.forwardSoftLimit != null) {
            config.ForwardSoftLimitThreshold = deviceConfig.forwardSoftLimit;
            config.ForwardSoftLimitEnable = true;
        }
        GreenLogger.log("  forwardSoftLimit: " + config.ForwardSoftLimitThreshold);
        if (deviceConfig.reverseSoftLimit != null) {
            config.ReverseSoftLimitThreshold = deviceConfig.reverseSoftLimit;
            config.ReverseSoftLimitEnable = true;
        }
        GreenLogger.log("  reverseSoftLimit: " + config.ReverseSoftLimitThreshold);

        return config;
    }

    public CurrentLimitsConfigs GetCurrentConfigs(DeviceConfiguration deviceConfig) {
        var config = new CurrentLimitsConfigs();

        // the defaults for current limits are on from CTRE we want to enforce this.
        // turning off current limits only breaks things
        config.StatorCurrentLimitEnable = true;
        config.SupplyCurrentLimitEnable = true;

        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.statorCurrentLimit != null) {
            config.StatorCurrentLimit = deviceConfig.statorCurrentLimit;
        }
        GreenLogger.log("  statorCurrentLimit: " + config.StatorCurrentLimit);

        if (deviceConfig.supplyCurrentLimit != null) {
            config.SupplyCurrentLimit = deviceConfig.supplyCurrentLimit;
        }
        GreenLogger.log("  supplyCurrentLimit: " + config.SupplyCurrentLimit);

        if (deviceConfig.supplyCurrentLowerLimit != null) {
            config.SupplyCurrentLowerLimit = deviceConfig.supplyCurrentLowerLimit;
        }
        GreenLogger.log("  supplyCurrentLowerLimit: " + config.SupplyCurrentLowerLimit);

        if (deviceConfig.supplyCurrentLowerTime != null) {
            config.SupplyCurrentLowerTime = deviceConfig.supplyCurrentLowerTime;
        }
        GreenLogger.log("  supplyCurrentLowerTime: " + config.SupplyCurrentLowerTime);

        return config;
    }

    // Slot configs are the PID values
    private SlotConfigs GetSlotConfigs(Map<String, PIDSlotConfiguration> pidConfig, int slot) {
        var config = new SlotConfigs();
        config.SlotNumber = slot;
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        var key = "slot" + slot;
        if (pidConfig != null && pidConfig.containsKey(key)) {
            var pid = pidConfig.get(key);
            if (pid.kA != null) config.kA = pid.kA;
            if (pid.kD != null) config.kD = pid.kD;
            if (pid.kG != null) config.kG = pid.kG;
            if (pid.kP != null) config.kP = pid.kP;
            if (pid.kS != null) config.kS = pid.kS;
            if (pid.kV != null) config.kV = pid.kV;
            if (pid.kI != null) config.kI = pid.kI;
            if (pid.gravityType != null) config.GravityType = pid.gravityType;
            GreenLogger.log("  " + key +
                " - kP:" + GetDisplay(config.kP) +
                " kI:" + GetDisplay(config.kI) +
                " kD:" + GetDisplay(config.kD) +
                " kV:" + GetDisplay(config.kV) +
                " kS:" + GetDisplay(config.kS) +
                " kA:" + GetDisplay(config.kA) +
                " kG:" + GetDisplay(config.kG) +
                " gravityType:" + config.GravityType);
        }
        return config;
    }

    // The Communication configs are used to configure motor type and connections
    private CommutationConfigs GetCommunicationConfigs(DeviceConfiguration deviceConfig) {
        var config = new CommutationConfigs();
        String info = "";
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.motorType != null) {
            config.MotorArrangement = deviceConfig.motorType;
            if (deviceConfig.motorType == MotorArrangementValue.Brushed_DC) {
                config.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_C;
                info = " using Leads_A_and_C";
            }
        } else {
            throw new IllegalArgumentException("motorType config is missing in yml for " + deviceConfig.name);
        }
        GreenLogger.log("  motorType: " + config.MotorArrangement + info);
        return config;
    }

    // These configurations control the motor inversions and neutral behaviour
    private MotorOutputConfigs GetMotorOutputConfigs(DeviceConfiguration deviceConfig) {
        var config = new MotorOutputConfigs();
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if (deviceConfig.motorRotation != null) {
            config.Inverted = deviceConfig.motorRotation;
        }
        GreenLogger.log("  motorRotation: " + config.Inverted);
        if (deviceConfig.neutralMode != null) {
            config.NeutralMode = deviceConfig.neutralMode;
        }
        GreenLogger.log("  neutralMode: " + config.NeutralMode);
        return config;
    }

    /**
     * Gets CTRE Swerve Modules
     * See <a href="https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/mechanisms/swerve/swerve-builder-api.html">Swerve Builder</a>
     */
    public SwerveDrivetrainConstants getSwerveDrivetrainConstant(String subsystemName) {
        var config = getSubsystemConfig(subsystemName);
        var constants = new SwerveDrivetrainConstants();
        constants.CANBusName = config.canBusName;
        constants.Pigeon2Id = config.devices.get("gyro").id;
        return constants;
    }

    private int getNextGhostId() {
        lastGhostId++;
        return lastGhostId;
    }

    public SwerveModuleConstants[] getSwerveModuleConstants(String subsystemName, double maxSpd) {
        var config = getSubsystemConfig(subsystemName);
        var kinematics = config.kinematics;
        verifyKinematics(subsystemName, kinematics);
        var constants = new SwerveModuleConstants[config.modules.size()];
        var x = config.kinematics.wheelbaseWidth / 2;
        var y = config.kinematics.wheelbaseLength / 2;
        int i = 0;
        ParentConfiguration driveConf = null;
        ParentConfiguration steerConf = null;
        Slot0Configs driveSlot = null;
        Slot0Configs steerSlot = null;

        // use ctre swerve factory to create the constants
        var factory = new SwerveModuleConstantsFactory();

        // setup kinematics
        factory.WheelRadius = kinematics.wheelRadius;
        factory.DriveMotorGearRatio = kinematics.driveGearing;
        factory.SteerMotorGearRatio = kinematics.steerGearing;
        factory.SpeedAt12Volts = maxSpd;

        for (var module : config.modules.values()) {
            var drive = config.devices.get(module.drive);
            var azm = config.devices.get(module.azimuth);
            // pull module configs from the first module
            if (driveConf == null) {
                GreenLogger.log("Module Drive Configuration");
                driveConf = getCTREConfig(config, drive);
                driveSlot = Slot0Configs.from(GetSlotConfigs(config.drivePID, 0));
                GreenLogger.log("Module Azimuth Configuration");
                steerConf = getCTREConfig(config, azm);
                steerSlot = Slot0Configs.from(GetSlotConfigs(config.azimuthPID, 0));
                GreenLogger.log("Creating " + config.modules.size() + " Modules");
                GreenLogger.log(" WheelRadius: " + factory.WheelRadius);
                GreenLogger.log(" DriveMotorGearRatio: " + factory.DriveMotorGearRatio);
                GreenLogger.log(" SteerMotorGearRatio: " + factory.SteerMotorGearRatio);
                GreenLogger.log(" SpeedAt12Volts: " + factory.SpeedAt12Volts);
            }

            // CTRE overwrites some of the defaults see their docs
            factory.DriveMotorInitialConfigs = driveConf;
            factory.DriveMotorGains = driveSlot;
            factory.DriveMotorClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;

            factory.SteerMotorClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;
            factory.SteerMotorGains = steerSlot;
            factory.SteerMotorInitialConfigs = steerConf;
            factory.SteerMotorType = getSteerArrangement(azm.motorType);

            var constant = factory.createModuleConstants(
                azm.id,
                drive.id,
                azm.remoteSensorId == null ? azm.id : azm.remoteSensorId,
                azm.remoteOffest,
                i == 0 || i == 1 ? x : -x,
                i == 1 || i == 3 ? -y : y,
                drive.motorRotation == InvertedValue.Clockwise_Positive,
                azm.motorRotation == InvertedValue.Clockwise_Positive,
                false
            );
            GreenLogger.log("  " + getModuleName(i) + ": x:" + constant.LocationX + " y:" + constant.LocationY + " remoteOffest:" + azm.remoteOffest);
            constants[i] = constant;
            i++;
        }
        return constants;
    }

    // Logs missing values to kinematics yaml configuration
    private void verifyKinematics(String subsystemName, KinematicsConfig kinematics) {
        if (kinematics == null) {
            var message = subsystemName + " requires kinematics in yaml";
            throw new NullPointerException(message);
        }
        String property = null;
        if (kinematics.driveGearing == 0) {
            property = "driveGearing";
        }
        if (kinematics.steerGearing == 0) {
            property = "steerGearing";
        }
        if (kinematics.maxDriveRPS == 0) {
            property = "maxDriveRPS";
        }
        if (kinematics.wheelbaseLength == 0) {
            property = "wheelbaseLength";
        }
        if (kinematics.wheelbaseWidth == 0) {
            property = "wheelbaseWidth";
        }
        if (kinematics.robotMass == 0) {
            property = "robotMass";
        }
        if (kinematics.wheelCOF == 0) {
            property = "wheelCOF";
        }
        if (kinematics.wheelRadius == 0) {
            property = "wheelRadius";
        }
        if (kinematics.maxAngularRate == null || kinematics.maxAngularRate == 0) {
            property = "maxAngularRate";
        }
        if (property != null) {
            var message = subsystemName + " kinematics " + property + " can't be null or 0";
            throw new IllegalArgumentException(message);
        }
    }

    private SwerveModuleConstants.SteerMotorArrangement getSteerArrangement(MotorArrangementValue value) {
        var steer = SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated;
        if (value == MotorArrangementValue.Brushed_DC) {
            return SwerveModuleConstants.SteerMotorArrangement.TalonFXS_Brushed_AC;
        } else if (value == MotorArrangementValue.Minion_JST) {
            return SwerveModuleConstants.SteerMotorArrangement.TalonFXS_Minion_JST;
        }
        return steer;
    }
}
