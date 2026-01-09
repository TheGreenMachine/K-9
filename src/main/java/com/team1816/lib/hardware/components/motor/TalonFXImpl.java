package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team1816.lib.hardware.components.ICTREDevice;
import com.team1816.lib.util.GreenLogger;
import com.team1816.season.Robot;

public class TalonFXImpl extends TalonFX implements ICTREDevice, IMotor {

    private TalonFXConfiguration config;
    private TalonFXSimState simState;

    public TalonFXImpl(int deviceId, CANBus canbus) {
        super(deviceId, canbus);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logDetails) {
        this.config = (TalonFXConfiguration) config;
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
            var clockwise = ((TalonFXConfiguration) config).MotorOutput.Inverted == InvertedValue.Clockwise_Positive;
            simState.Orientation = clockwise ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        }
        return this.getConfigurator().apply(this.config);
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
    public StatusCode setSimRotorPosition(double rotations) {
        return simState.setRawRotorPosition(rotations);
    }

    @Override
    public double getSimMotorVoltage() {
        return simState.getMotorVoltage();
    }

    @Override
    public StatusCode setSimSupplyVoltage(double volts) {
        return simState.setSupplyVoltage(volts);
    }
}

