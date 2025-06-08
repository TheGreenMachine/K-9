package com.team1816.lib.hardware.components;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.team1816.lib.hardware.components.gyro.IGyro;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.util.GreenLogger;

// for devices to be ghosted, this class needs to implement their interfaces
public class GhostDevice implements ICTREDevice, IMotor, IGyro {

    private final String canBusName;
    private final int canId;
    private double lastTargetVelocity;
    private double lastTargetPosition;
    private double simTarget; // this will be the value when commanded and simulation will iterate over every call
    private final int simVelChange = 20;

    public GhostDevice(Integer canID, String canBusName) {
        this.canId = canID;
        this.canBusName = canBusName;
    }

    @Override
    public StatusCode setControl(ControlRequest request) {
        var name = request.getName();
        if (name.contains("Velocity")) {
            var info = request.getControlInfo().get("Velocity");
            lastTargetVelocity = Double.parseDouble(info);
        } else if (name.contains("Position")) {
            var info = request.getControlInfo().get("Position");
            lastTargetPosition = Double.parseDouble(info);
        } else if (name.contains("Brake")) {
            lastTargetVelocity = 0;
        } else if (name.contains("DutyCycleOut")) {
            var info = request.getControlInfo().get("Output");
            lastTargetVelocity = 600 * Double.parseDouble(info);
        }
        return StatusCode.OK;
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath) {
        // add default logging for target and actual position/velocities
        if (config.getClass().getName().contains("Talon")) {
            GreenLogger.periodicLog(logPath + "Target Position (rotations)", this::getDeviceTargetPosition);
            GreenLogger.periodicLog(logPath + "Actual Position (rotations)", this::getMotorPosition);
            GreenLogger.periodicLog(logPath + "Target Velocity (rps)", this::getDeviceTargetVelocity);
            GreenLogger.periodicLog(logPath + "Actual Velocity (rps)", this::getMotorVelocity);
        }
        GreenLogger.periodicLog(logPath + "Connected", this::isConnected);
        return StatusCode.OK;
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public double getDeviceTargetVelocity() {
        return lastTargetVelocity;
    }

    @Override
    public double getDeviceTargetPosition() {
        return lastTargetPosition;
    }

    @Override
    public double getMotorVelocity() {
        if(simTarget != lastTargetVelocity) {
            var prevSim = simTarget;
            simTarget += simTarget < lastTargetVelocity ? simVelChange : -simVelChange;
            if(lastTargetVelocity < 0 && simTarget < lastTargetVelocity){
                simTarget = lastTargetVelocity;
            } else if(lastTargetVelocity > 0 && simTarget > lastTargetVelocity) {
                simTarget = lastTargetVelocity;
            } else if(lastTargetVelocity == 0 && Math.abs(simTarget) <= simVelChange) {
                simTarget = 0;
            }
        }
        return simTarget;
    }

    @Override
    public double getMotorPosition() {
        return lastTargetPosition;
    }

    @Override
    public void zeroMotorPosition() {
        lastTargetPosition = 0;
    }

    @Override
    public double getGyroYaw() {
        return 0;
    }
}
