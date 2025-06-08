package com.team1816.lib.hardware.components.sensor;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.team1816.lib.hardware.DeviceConfiguration;
import com.team1816.lib.hardware.components.IPhoenix6;

public class CanRangeImpl extends CANrange implements IPhoenix6 {

    public CanRangeImpl(int deviceId, String canbus) {
        super(deviceId, canbus);
    }

    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath) {
        return this.getConfigurator().apply((CANrangeConfiguration) config);
    }
}
