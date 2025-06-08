package com.team1816.lib.hardware.components.motor;

import com.team1816.lib.hardware.components.IPhoenix6;

public interface IMotor extends IPhoenix6 {
    // method to get the current velocity of rotor in rps
    double getMotorVelocity();
    // method to get the angle of rotor in rotations
    double getMotorPosition();
    // used to zero the motor position
    void zeroMotorPosition();
}
