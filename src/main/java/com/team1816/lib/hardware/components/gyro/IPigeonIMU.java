package com.team1816.lib.hardware.components.gyro;

import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

/**
 * The root interface for the CTRE Pigeon component
 */
public interface IPigeonIMU {
    /**
     * Returns gyroscopic yaw / transverse planar angle
     *
     * @return yaw (degrees)
     */
    double getYawValue();

    /**
     * Returns gyroscopic pitch / transverse lateral angle
     *
     * @return pitch (degrees)
     */
    double getPitchValue();

    /**
     * Returns gyroscopic roll / transverse frontal angle
     *
     * @return roll (degrees)
     */
    double getRollValue();

    /**
     * Returns x, y, and z acceleration in a casted fixed point double array
     *
     * @return acceleration
     */
    double[] getAcceleration();

    /**
     * Sets the gyroscopic yaw to a specific angle
     *
     * @param angle (degrees)
     */
    void set_Yaw(double angle);

    /**
     * Returns true if a pigeon reset has occurred
     *
     * @return hasResetOccurred
     */
    boolean hasResetOccurred();

    /**
     * Configures factory defaults
     *
     */
    void configFactoryDefaults();

    /**
     * Sets the synchronized status frame period of the pigeon and is directly related to CAN-bus utilization
     *
     * @param statusFrame
     * @param periodMs
     * @return
     */
    void set_StatusFramePeriod(PigeonIMU_StatusFrame statusFrame, int periodMs);

    boolean isReady();
}
