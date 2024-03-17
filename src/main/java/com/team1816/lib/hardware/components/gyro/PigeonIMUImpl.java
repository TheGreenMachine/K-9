package com.team1816.lib.hardware.components.gyro;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

/**
 * A class that interfaces with the PigeonIMU
 */
public class PigeonIMUImpl extends PigeonIMU implements IPigeonIMU {

    /**
     * State
     */
    private long m_handle = 0l;

    /**
     * Instantiates a PigeonIMUImpl
     *
     * @param id (CAN-Bus id)
     */
    public PigeonIMUImpl(int id) {
        super(id);
        m_handle = super.getHandle();
    }

    /**
     * Returns gyroscopic yaw / transverse planar angle
     *
     * @return yaw (degrees)
     * @see IPigeonIMU#getYawValue()
     */
    @Override
    public double getYawValue() {
        return super.getYaw();
    }

    /**
     * Returns gyroscopic pitch / transverse lateral angle
     *
     * @return pitch (degrees)
     * @see IPigeonIMU#getPitchValue()
     */
    @Override
    public double getPitchValue() {
        return super.getPitch();
    }

    /**
     * Returns gyroscopic roll / transverse frontal angle
     *
     * @return roll (degrees)
     * @see IPigeonIMU#getRollValue()
     */
    @Override
    public double getRollValue() {
        return super.getRoll();
    }

    /**
     * Returns x, y, and z acceleration in a casted fixed point double array
     *
     * @return acceleration
     * @see IPigeonIMU#getAcceleration()
     */
    @Override
    public double[] getAcceleration() {
        short[] accel = new short[3];
        getBiasedAccelerometer(accel);
        return new double[]{accel[0], accel[1], accel[2]};
    }

    /**
     * Sets the gyroscopic yaw to a specific angle
     *
     * @param angle (degrees)
     * @return ErrorCode / void
     * @see IPigeonIMU#getAcceleration()
     */
    @Override
    public void set_Yaw(double angle) {
        super.setYaw(angle);
    }

    /**
     * Returns true if a pigeon reset has occurred
     *
     * @return boolean hasResetOccurred
     * @see IPigeonIMU#hasResetOccurred()
     */
    @Override
    public boolean hasResetOccurred() {
        return super.hasResetOccurred();
    }

    /**
     * Configures factory defaults
     *
     * @return ErrorCode / void
     * @see IPigeonIMU#configFactoryDefaults()
     */
    @Override
    public void configFactoryDefaults() {
        super.configFactoryDefault();
    }

    @Override
    public void set_StatusFramePeriod(PigeonIMU_StatusFrame statusFrame, int periodMs) {
        super.setStatusFramePeriod(statusFrame, periodMs);
    }

    @Override
    public boolean isReady() {
        return super.getState() == PigeonState.Ready;
    }
}
