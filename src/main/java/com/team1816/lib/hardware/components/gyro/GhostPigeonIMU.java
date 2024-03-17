package com.team1816.lib.hardware.components.gyro;

import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team1816.lib.hardware.components.motor.IGreenMotor;

/**
 * This class emulates the behaviour of a Pigeon that is not physically implemented on a robot
 *
 * @see IPigeonIMU
 */
public class GhostPigeonIMU implements IPigeonIMU {

    /**
     * State
     */
    double simulatedYaw; // simulated yaw
    double simulatedPitch;
    double simulatedRoll;

    /**
     * Instantiates a free ghost pigeon
     */
    public GhostPigeonIMU(int id) {
        simulatedYaw = 0;
        simulatedPitch = 0;
        simulatedRoll = 0;
    }

    /**
     * Alternately instantiates a ghost pigeon attached to a motor
     */
    public GhostPigeonIMU(IGreenMotor motor) {
    }

    /**
     * Returns the simulatedYaw
     *
     * @return simulatedYaw
     * @see IPigeonIMU#getYawValue()
     */
    @Override
    public double getYawValue() {
        return simulatedYaw;
    }

    /**
     * Returns the simulatedPitch
     *
     * @return simulatedPitch
     * @see IPigeonIMU#getPitchValue()
     */
    @Override
    public double getPitchValue() {
        return simulatedPitch;
    }

    /**
     * Returns the simulatedRoll
     *
     * @return simulatedRoll
     * @see IPigeonIMU#getRollValue()
     */
    @Override
    public double getRollValue() {
        return simulatedRoll;
    }

    /**
     * Returns constant simulated acceleration, can be modified for other purposes
     *
     * @return simulatedAcceleration
     * @see IPigeonIMU#getAcceleration()
     */
    @Override
    public double[] getAcceleration() {
        double[] accel = new double[]{0d, 0d, 9.8d};
        return accel;
    }

    /**
     * Sets the simulated yaw to a specified value
     *
     * @param angle (degrees)
     * @return ErrorCode / void
     * @see IPigeonIMU#set_Yaw(double)
     */
    @Override
    public void set_Yaw(double angle) {
        simulatedYaw = angle;
    }

    /**
     * Returns if a reset has occurred
     *
     * @return boolean hasResetOccurred
     * @see IPigeonIMU#hasResetOccurred()
     */
    @Override
    public boolean hasResetOccurred() {
        return false;
    }

    /**
     * Functionality: non-existent
     *
     * @return ErrorCode / void
     * @see IPigeonIMU#configFactoryDefaults()
     */
    @Override
    public void configFactoryDefaults() {}

    /**
     * Functionality: non-existent
     *
     * @return ErrorCode / void
     * @see IPigeonIMU#set_StatusFramePeriod(PigeonIMU_StatusFrame, int)
     */
    @Override
    public void set_StatusFramePeriod(
        PigeonIMU_StatusFrame statusFrame,
        int periodMs
    ) {}

    @Override
    public boolean isReady() {
        return true;
    }
}
