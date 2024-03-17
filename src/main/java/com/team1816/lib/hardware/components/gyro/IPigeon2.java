package com.team1816.lib.hardware.components.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * An interface for Pigeon 2 Gyroscopes
 * @see com.ctre.phoenix6.hardware.Pigeon2
 */
public interface IPigeon2 {
    /**
     * Configures the gyro to be relative to the passed in angle
     * @param angle The angle for the robot to be relative to
     */
    void configMountPose(Rotation2d angle);
}
