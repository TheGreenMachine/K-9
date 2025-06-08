package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team1816.lib.hardware.DeviceConfiguration;
import com.team1816.lib.hardware.components.ICTREDevice;
import com.team1816.lib.util.GreenLogger;

public class TalonFXImpl extends TalonFX implements ICTREDevice, IMotor {

    private double lastVelocityCommand;
    private double lastPositionCommand;

    public TalonFXImpl(int deviceId, String canbus) {
        super(deviceId, canbus);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public StatusCode setControl(ControlRequest request) {
        var name = request.getName();
        if(name.contains("Velocity")) {
            lastVelocityCommand = 0;
        } else if(name.contains("Position")) {
            lastPositionCommand = 0;
        } else if(name.contains("Brake")) {
            lastVelocityCommand = 0;
        }
        return super.setControl(request);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath) {
        // add default logging for target and actual position/velocities
        GreenLogger.periodicLog(logPath + "Target Position (rotations)", this::getDeviceTargetPosition);
        GreenLogger.periodicLog(logPath + "Actual Position (rotations)", this::getMotorPosition);
        GreenLogger.periodicLog(logPath + "Target Velocity (rps)", this::getDeviceTargetVelocity);
        GreenLogger.periodicLog(logPath + "Actual Velocity (rps)", this::getMotorVelocity);
        GreenLogger.periodicLog(logPath + "Connected", this::isConnected);
        return this.getConfigurator().apply((TalonFXConfiguration) config);
    }

    @Override
    public double getDeviceTargetVelocity() {
        return lastVelocityCommand;
    }

    @Override
    public double getDeviceTargetPosition() {
        return lastPositionCommand;
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
}

