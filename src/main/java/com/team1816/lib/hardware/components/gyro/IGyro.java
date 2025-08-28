package com.team1816.lib.hardware.components.gyro;

import com.ctre.phoenix6.StatusCode;
import com.team1816.lib.hardware.components.IPhoenix6;
import edu.wpi.first.math.geometry.Rotation2d;

public interface IGyro extends IPhoenix6 {
    double getGyroYaw();
    Rotation2d getRotation2d();
    StatusCode updateYaw(double deg);
}
