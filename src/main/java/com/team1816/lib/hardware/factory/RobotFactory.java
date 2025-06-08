package com.team1816.lib.hardware.factory;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.team1816.lib.hardware.DeviceConfiguration;
import com.team1816.lib.hardware.RobotConfiguration;
import com.team1816.lib.hardware.SubsystemConfig;
import com.team1816.lib.hardware.components.GhostDevice;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.lib.hardware.components.gyro.Pigeon2Impl;
import com.team1816.lib.hardware.components.led.CANdleImpl;
import com.team1816.lib.hardware.components.led.CANifierImpl;
import com.team1816.lib.hardware.components.motor.TalonFXImpl;
import com.team1816.lib.hardware.components.motor.TalonFXSImpl;
import com.team1816.lib.hardware.components.sensor.CanRangeImpl;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.Map;
import java.util.function.Supplier;

import static com.team1816.lib.util.FormatUtils.GetDisplay;


public class RobotFactory {

    private RobotConfiguration config;
    public boolean RobotIsReal; // Use to detect real or simulation public to override for tests

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
        if(subsystem == null) {
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

    public double getConstant(String subsystemName, String name) {
        return getConstant(subsystemName, name, 0, true);
    }

    public double getConstant(String subsystemName, String name, double defaultVal) {
        return getConstant(subsystemName, name, defaultVal, true);
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

    public IPhoenix6 getDevice(String subsystemName, String deviceName) {
        if(config == null) {
            throw new NullPointerException("config is null");
        }
        var subsystem = getSubsystemConfig(subsystemName);
        // we should have devices defined verify that
        if(subsystem.devices == null){
            throw new IllegalArgumentException("Devices not defined for " + subsystemName);
        }
        // ensure device is defined in yaml
        var deviceConfig = subsystem.devices.get(deviceName);
        if (deviceConfig == null) {
            throw new IllegalArgumentException("Device " + deviceName + " not found");
        }
        deviceConfig.name = deviceName;
        // ensure device type is set
        if(deviceConfig.deviceType == null) {
            throw new IllegalArgumentException("deviceType missing in yml");
        }
        return getDevInst(deviceConfig, subsystem);
    }


    // Instantiates device based on the type and applies the configuration
    private IPhoenix6 getDevInst(DeviceConfiguration deviceConfig, SubsystemConfig subsystemConfig) {
        IPhoenix6 devInst = null;
        ParentConfiguration parentConfig = null;
        if(!(subsystemConfig.implemented && RobotIsReal)) {
            if(RobotIsReal)
                GreenLogger.log("Subsystem " + subsystemConfig.name + " not implemented ghosting " + deviceConfig.name);
            else
                GreenLogger.log("Robot is not real ghosting " + deviceConfig.name);
            devInst = new GhostDevice(deviceConfig.id, subsystemConfig.canBusName);
        } else {
            GreenLogger.log("Creating " + deviceConfig.name);
        }
        GreenLogger.log("  id: " + deviceConfig.id);
        GreenLogger.log("  deviceType: " + deviceConfig.deviceType);
        switch (deviceConfig.deviceType) {
            case TalonFX -> {
                if(devInst == null) devInst = new TalonFXImpl(deviceConfig.id, subsystemConfig.canBusName);
                parentConfig = new TalonFXConfiguration();
            }
            case TalonFXS -> {
                if(devInst == null) devInst = new TalonFXSImpl(deviceConfig.id, subsystemConfig.canBusName);
                parentConfig = new TalonFXSConfiguration();
            }
            case Pigeon2 -> {
                if(devInst == null) devInst = new Pigeon2Impl(deviceConfig.id, subsystemConfig.canBusName);
                parentConfig = new Pigeon2Configuration();
            }
            case Candle -> {
                if(devInst == null)devInst = new CANdleImpl(deviceConfig.id, subsystemConfig.canBusName);
                parentConfig = new CANdleConfiguration();
            }
            case CanRange -> {
                if(devInst == null)  devInst = new CanRangeImpl(deviceConfig.id, subsystemConfig.canBusName);
                parentConfig = new CANrangeConfiguration();
            }
            case Canifier -> {
                if(devInst == null) devInst = new CANifierImpl(deviceConfig.id);
            }
        }

        if(parentConfig != null) {
            var logPath = subsystemConfig.name + "/" + deviceConfig.name + "/";
            // Update defaults with yaml values
            ApplyYamlConfigs(subsystemConfig,deviceConfig,parentConfig);
            // apply the configuration
            devInst.applyConfiguration(parentConfig, logPath);
        }
        return devInst;
    }

    private void ApplyYamlConfigs(SubsystemConfig subsystemConfig, DeviceConfiguration deviceConfig, ParentConfiguration parentConfig) {
        switch (deviceConfig.deviceType) {
            case TalonFX -> {
                var clazz = (TalonFXConfiguration)parentConfig;
                clazz.MotorOutput = GetMotorOutputConfigs(deviceConfig);
                clazz.Slot0 = Slot0Configs.from(GetSlotConfigs(subsystemConfig, 0));
                clazz.Slot1 = Slot1Configs.from(GetSlotConfigs(subsystemConfig, 1));
                clazz.Slot2 = Slot2Configs.from(GetSlotConfigs(subsystemConfig, 2));
                clazz.CurrentLimits = GetCurrentConfigs(deviceConfig);
                clazz.SoftwareLimitSwitch = GetSoftLimitConfigs(deviceConfig);
            }
            case TalonFXS -> {
                var clazz = (TalonFXSConfiguration)parentConfig;
                clazz.MotorOutput = GetMotorOutputConfigs(deviceConfig);
                clazz.Commutation = GetCommunicationConfigs(deviceConfig);
                clazz.Slot0 = Slot0Configs.from(GetSlotConfigs(subsystemConfig, 0));
                clazz.Slot1 = Slot1Configs.from(GetSlotConfigs(subsystemConfig, 1));
                clazz.Slot2 = Slot2Configs.from(GetSlotConfigs(subsystemConfig, 2));
                clazz.CurrentLimits = GetCurrentConfigs(deviceConfig);
                clazz.SoftwareLimitSwitch = GetSoftLimitConfigs(deviceConfig);
                clazz.ExternalFeedback = GetExternalFeedbackConfigs(deviceConfig);
            }
            case Pigeon2 -> {
            }
            case Candle -> {
                var clazz = (CANdleConfiguration)parentConfig;
                clazz.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
                clazz.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
                clazz.LED.StripType = StripTypeValue.BRG;
            }
            case Canifier -> {
            }
            case CanRange -> {
            }
            default -> {}
        }
    }

    private ExternalFeedbackConfigs GetExternalFeedbackConfigs(DeviceConfiguration deviceConfig) {
        var config = new ExternalFeedbackConfigs();
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if(deviceConfig.remoteSensor != null) {
            config.ExternalFeedbackSensorSource = deviceConfig.remoteSensor;
        }
        GreenLogger.log("  remoteSensor: " + config.ExternalFeedbackSensorSource);

        if(deviceConfig.remoteSensorId != null) {
            config.FeedbackRemoteSensorID = deviceConfig.remoteSensorId;
            GreenLogger.log("  remoteSensorId: " + config.FeedbackRemoteSensorID);
        }

        if(deviceConfig.remoteOffest != null) {
            config.AbsoluteSensorOffset = deviceConfig.remoteOffest;
            GreenLogger.log("  remoteOffest: " + config.AbsoluteSensorOffset);
        }

        return config;
    }

    private SoftwareLimitSwitchConfigs GetSoftLimitConfigs(DeviceConfiguration deviceConfig) {
        var config = new SoftwareLimitSwitchConfigs();
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if(deviceConfig.forwardSoftLimit != null) {
            config.ForwardSoftLimitThreshold = deviceConfig.forwardSoftLimit;
            config.ForwardSoftLimitEnable = true;
        }
        GreenLogger.log("  forwardSoftLimit: " + config.ForwardSoftLimitThreshold);
        if(deviceConfig.reverseSoftLimit != null) {
            config.ReverseSoftLimitThreshold = deviceConfig.reverseSoftLimit;
            config.ReverseSoftLimitEnable = true;
        }
        GreenLogger.log("  reverseSoftLimit: " + config.ReverseSoftLimitThreshold);

        return config;
    }

    private CurrentLimitsConfigs GetCurrentConfigs(DeviceConfiguration deviceConfig) {
        var config = new CurrentLimitsConfigs();

        // the defaults for current limits are on from CTRE we want to enforce this.
        // turning off current limits only breaks things
        config.StatorCurrentLimitEnable = true;
        config.SupplyCurrentLimitEnable = true;

        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        if(deviceConfig.statorCurrentLimit != null) {
            config.StatorCurrentLimit = deviceConfig.statorCurrentLimit;
        }
        GreenLogger.log("  statorCurrentLimit: " + config.StatorCurrentLimit);

        if(deviceConfig.supplyCurrentLimit != null) {
            config.SupplyCurrentLimit = deviceConfig.supplyCurrentLimit;
        }
        GreenLogger.log("  supplyCurrentLimit: " + config.SupplyCurrentLimit);

        if(deviceConfig.supplyCurrentLowerLimit != null) {
            config.SupplyCurrentLowerLimit = deviceConfig.supplyCurrentLowerLimit;
        }
        GreenLogger.log("  supplyCurrentLowerLimit: " + config.SupplyCurrentLowerLimit);

        if(deviceConfig.supplyCurrentLowerTime != null) {
            config.SupplyCurrentLowerTime = deviceConfig.supplyCurrentLowerTime;
        }
        GreenLogger.log("  supplyCurrentLowerTime: " + config.SupplyCurrentLowerTime);

        return config;
    }

    // Slot configs are the PID values
    private SlotConfigs GetSlotConfigs(SubsystemConfig subsystemConfig, int slot) {
        var config = new SlotConfigs();
        config.SlotNumber = slot;
        // if we have settings defined in YAML use them otherwise use the CTRE defaults
        var key =  "slot" + slot;
        if(subsystemConfig.pidConfig != null && subsystemConfig.pidConfig.containsKey(key)) {
            var pid = subsystemConfig.pidConfig.get(key);
            if(pid.kA != null) config.kA = pid.kA;
            if(pid.kD != null) config.kD = pid.kD;
            if(pid.kG != null) config.kG = pid.kG;
            if(pid.kP != null) config.kP = pid.kP;
            if(pid.kS != null) config.kS = pid.kS;
            if(pid.kV != null) config.kV = pid.kV;
            if(pid.kI != null) config.kI = pid.kI;
            if(pid.gravityType != null) config.GravityType = pid.gravityType;
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
        if(deviceConfig.motorType != null) {
            config.MotorArrangement = deviceConfig.motorType;
            if(deviceConfig.motorType == MotorArrangementValue.Brushed_DC){
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
        if(deviceConfig.motorRotation != null) {
            config.Inverted = deviceConfig.motorRotation;
        }
        GreenLogger.log("  motorRotation: " + config.Inverted);
        if(deviceConfig.neutralMode != null) {
            config.NeutralMode = deviceConfig.neutralMode;
        }
        GreenLogger.log("  neutralMode: " + config.NeutralMode);
        return config;
    }

}
