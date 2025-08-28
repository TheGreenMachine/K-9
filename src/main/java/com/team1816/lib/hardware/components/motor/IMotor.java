package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix6.StatusCode;
import com.team1816.lib.hardware.components.IPhoenix6;

public interface IMotor extends IPhoenix6 {
    // method to get the current velocity of rotor in rps
    double getMotorVelocity();
    // method to get the angle of rotor in rotations
    double getMotorPosition();
    // used to zero the motor position
    void zeroMotorPosition();
    StatusCode setSimRotorVelocity(double rps);
    StatusCode setSimRotorPosition(double rotations);
    double getSimMotorVoltage();
}
