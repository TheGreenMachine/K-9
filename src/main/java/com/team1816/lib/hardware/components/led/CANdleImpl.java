package com.team1816.lib.hardware.components.led;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.sim.CANdleSimState;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.lib.util.GreenLogger;
import com.team1816.season.Robot;
import edu.wpi.first.wpilibj.RobotController;

public class CANdleImpl extends CANdle implements IPhoenix6 {

    private CANdleSimState simState;

    public CANdleImpl(Integer canID, String canBusName) {

        super(canID, canBusName);
        if(Robot.isSimulation()){
            simState = getSimState();
        }
    }

    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logging) {
        GreenLogger.periodicLog(logPath + "Connected", this::isConnected);
        return this.getConfigurator().apply((CANdleConfiguration) config);
    }

    @Override
    public StatusCode setSimSupplyVoltage(double volts) {
        return simState.setSupplyVoltage(volts);
    }

}
