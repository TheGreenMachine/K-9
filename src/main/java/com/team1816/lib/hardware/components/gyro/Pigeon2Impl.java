package com.team1816.lib.hardware.components.gyro;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.lib.util.GreenLogger;
import com.team1816.season.Robot;

/**
 * A class that interfaces with the Pigeon2
 */
public class Pigeon2Impl extends Pigeon2 implements IPhoenix6, IGyro {

    private Pigeon2SimState simState;

    public Pigeon2Impl(int deviceId, String canbus) {
        super(deviceId, canbus);
        if(Robot.isSimulation()){
            simState = getSimState();
        }
    }

    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logging) {
        GreenLogger.periodicLog(logPath + "Connected", this::isConnected);
        GreenLogger.periodicLog(logPath + "Yaw", this::getGyroYaw);
        return this.getConfigurator().apply((Pigeon2Configuration) config);
    }

    @Override
    public double getGyroYaw() {
        return getYaw().getValueAsDouble();
    }

    @Override
    public StatusCode updateYaw(double deg) {
        if(Robot.isSimulation()){
            return simState.setRawYaw(deg);
        }
        return setYaw(deg);
    }

    @Override
    public StatusCode setSimSupplyVoltage(double volts) {
        return simState.setSupplyVoltage(volts);
    }
}
