package com.team1816.lib.hardware.components.led;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.team1816.lib.hardware.components.IPhoenix6;

public class CANifierImpl extends CANifier implements IPhoenix6 {
    public CANifierImpl(int deviceId) {
        super(deviceId);
    }

    @Override
    public StatusCode setControl(ControlRequest request) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logging) {
        return StatusCode.OK;
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public StatusCode setSimSupplyVoltage(double volts) {
        // does not support sim
        return StatusCode.OK;
    }

}
