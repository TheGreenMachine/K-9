package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.team1816.lib.hardware.components.ICTREDevice;
import com.team1816.lib.util.GreenLogger;
import com.team1816.season.Robot;

public class TalonFXSImpl extends TalonFXS implements ICTREDevice, IMotor {

    private TalonFXSSimState simState;
    private TalonFXSConfiguration config;

    public TalonFXSImpl(int deviceId, String canbus) {
        super(deviceId, canbus);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logDetails) {
        this.config = (TalonFXSConfiguration) config;
        // add default logging for target and actual position/velocities
        if (logDetails) {
            GreenLogger.periodicLog(logPath + "Actual Position (rotations)", this::getMotorPosition);
            GreenLogger.periodicLog(logPath + "Actual Velocity (rps)", this::getMotorVelocity);
        }
        GreenLogger.periodicLog(logPath + "Reference", this::getDeviceReference);
        GreenLogger.periodicLog(logPath + "Connected", this::isConnected);
        GreenLogger.periodicLog(logPath + "Stator", ()-> getStatorCurrent().getValueAsDouble());
        if(Robot.isSimulation()){
            simState = getSimState();
            var clockwise = ((TalonFXSConfiguration) config).MotorOutput.Inverted == InvertedValue.Clockwise_Positive;
            simState.MotorOrientation = clockwise ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        }
        return this.getConfigurator().apply(this.config);
    }

    @Override
    public StatusCode setSimSupplyVoltage(double volts) {
        return simState.setSupplyVoltage(volts);
    }

    @Override
    public double getDeviceError() {
        return getClosedLoopError().getValueAsDouble();
    }

    @Override
    public double getDeviceReference() {
        return getClosedLoopReference().getValueAsDouble();
    }

    @Override
    public double getMotorVelocity() {
        return getVelocity().getValueAsDouble();
    }

    @Override
    public double getMotorPosition() {
        return getPosition().getValueAsDouble();
    }

    @Override
    public void zeroMotorPosition() {
        setPosition(0);
    }

    @Override
    public StatusCode setSimRotorVelocity(double rps) {
        return simState.setRotorVelocity(rps);
    }

    @Override
    public double getSimMotorVoltage() {
        return simState.getMotorVoltage();
    }

    @Override
    public StatusCode setSimRotorPosition(double rotations) {
        return simState.setRawRotorPosition(rotations);
    }
}

