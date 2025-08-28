package com.team1816.lib.hardware.components.sensor;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.season.Robot;

public class CANCoderImpl extends CANcoder implements IPhoenix6 {

    private CANcoderSimState simState;

    public CANCoderImpl(int deviceId, String canbus) {

        super(deviceId, canbus);
        if(Robot.isSimulation()){
            simState = getSimState();
        }
    }

    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logDetails) {
        return this.getConfigurator().apply((CANcoderConfiguration) config);
    }

    @Override
    public StatusCode setSimSupplyVoltage(double volts) {
        return simState.setSupplyVoltage(volts);
    }
}
