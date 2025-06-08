package com.team1816.lib.hardware.components.led;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.lib.util.GreenLogger;

public class CANdleImpl extends CANdle implements IPhoenix6 {

    public CANdleImpl(Integer canID, String canBusName) {
        super(canID, canBusName);
    }

    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath) {
        GreenLogger.periodicLog(logPath + "Connected", this::isConnected);
        return this.getConfigurator().apply((CANdleConfiguration) config);
    }

}
