package com.team1816.lib.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.team1816.lib.commands.SubsystemTestCommand;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Turret extends GreenSubsystem {

    public static final String NAME = "turret";
    private final IMotor turret = (IMotor)factory.getDevice(NAME, "spiny");
    private double desiredPosition = 0;

    @Override
    public void periodic() {
        if (turret.getMotorPosition() != desiredPosition) {
            turret.setControl(new PositionVoltage(desiredPosition));
        }
    }

    @Override
    public Command TestSubsystem() {
        var ref = new Object() {
            boolean passed = true;
        };
        var group = new SequentialCommandGroup();
        group.addCommands(runOnce(()->{
            GreenLogger.log("Testing " + NAME);
            ref.passed = ref.passed && turret.setControl(new DutyCycleOut(.42)) == StatusCode.OK;
        }));
        group.addCommands(new WaitCommand(2));
        group.addCommands(runOnce(()->{
            ref.passed = ref.passed && turret.setControl(new StaticBrake()) == StatusCode.OK;
            pubsub.GetEvent(SubsystemTestCommand.TestResult.class).Publish(ref.passed);
            GreenLogger.log("Testing " + NAME + " passed: " + ref.passed);
        }));
        return group;
    }
}
