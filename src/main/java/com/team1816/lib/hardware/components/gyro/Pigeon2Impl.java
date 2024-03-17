package com.team1816.lib.hardware.components.gyro;

import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team1816.lib.util.Util;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A class that interfaces with the Pigeon2
 */
public class Pigeon2Impl extends Pigeon2 implements IPigeonIMU, IPigeon2 {

    /**
     * Instantiates a Pigeon2Impl
     *
     * @param id     CAN-bus ID
     * @param canBus CAN-bus name (if multiple)
     */
    public Pigeon2Impl(int id, String canBus) {
        super(id, canBus);
    }

    /**
     * Returns gyroscopic yaw / transverse planar angle
     *
     * @return yaw (degrees)
     * @see IPigeonIMU#getYawValue()
     */
    @Override
    public double getYawValue() {
        return super.getYaw().getValueAsDouble();
    }

    /**
     * Returns gyroscopic pitch
     *
     * @return yaw (degrees)
     */
    @Override
    public double getPitchValue() {
        return super.getPitch().getValueAsDouble();
    }

    /**
     * Returns gyroscopic pitch
     *
     * @return yaw (degrees)
     */
    @Override
    public double getRollValue() {
        return super.getRoll().getValueAsDouble();
    }


    /**
     * Returns x, y, and z acceleration in a fixed point double array
     *
     * @return acceleration
     * @see IPigeonIMU#getAcceleration()
     */
    @Override
    public double[] getAcceleration() {
        return new double[]{
                super.getAccelerationX().getValueAsDouble(),
                super.getAccelerationY().getValueAsDouble(),
                super.getAccelerationZ().getValueAsDouble()
        };
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
        super.getConfigurator().apply(new Pigeon2Configuration());
    }

    @Override
    public void set_StatusFramePeriod(PigeonIMU_StatusFrame statusFrame, int periodMs) {
        BaseStatusSignal.setUpdateFrequencyForAll(
                Util.msToHz(periodMs),
                super.getAccelerationX(), //Don't you just love varargs? i don't
                super.getAccelerationY(),
                super.getAccelerationZ(),
                super.getRoll(),
                super.getPitch(),
                super.getYaw(),
                super.getAngularVelocityZWorld() //Used internally in Pigeon2
        );
    }

    @Override
    public boolean isReady() {
        //TODO:this needs to be real issue in API?
        return true;
    }

    @Override
    public void configMountPose(Rotation2d angle) {
        super.getConfigurator().apply(new MountPoseConfigs()
                .withMountPoseYaw(angle.getDegrees())
                .withMountPosePitch(0)
                .withMountPoseRoll(0)
        );
    }
}
