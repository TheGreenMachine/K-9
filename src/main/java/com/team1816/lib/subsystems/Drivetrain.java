package com.team1816.lib.subsystems;

import com.team1816.lib.hardware.components.gyro.IGyro;
import com.team1816.lib.hardware.components.motor.IMotor;
import edu.wpi.first.wpilibj2.command.Command;

public class Drivetrain extends GreenSubsystem {

    public static final String NAME = "drivetrain";
    private final IMotor leftMain = (IMotor)factory.getDevice(NAME, "leftMain");
    private final IMotor rightMain = (IMotor)factory.getDevice(NAME, "rightMain");
    private final IGyro gyro = (IGyro) factory.getDevice(NAME, "gyro");

    @Override
    public Command TestSubsystem() {
        return null;
    }

    @Override
    public void periodic() {
    }

}
