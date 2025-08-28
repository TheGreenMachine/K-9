package com.team1816.lib.hardware.components;

public interface ICTREDevice extends IPhoenix6 {
    enum DeviceType {
        TalonFX, //Falcons and Krakens
        TalonFXS, //cims, bags, etc
        Pigeon2,
        CANdle,
        CANifier,
        CANrange,
        CANcoder
    }
    double getDeviceReference();
    double getDeviceError();
}
