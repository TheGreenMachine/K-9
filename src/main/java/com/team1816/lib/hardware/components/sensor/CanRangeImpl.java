package com.team1816.lib.hardware.components.sensor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.sim.CANrangeSimState;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.season.Robot;

public class CanRangeImpl extends CANrange implements IPhoenix6 {

    private CANrangeSimState simState;

    public CanRangeImpl(int deviceId, CANBus canbus) {
        super(deviceId, canbus);
        if(Robot.isSimulation()){
            simState = getSimState();
        }
    }

    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logging) {
        return this.getConfigurator().apply((CANrangeConfiguration) config);
    }

    @Override
    public StatusCode setSimSupplyVoltage(double volts) {
        return simState.setSupplyVoltage(volts);
    }
}
