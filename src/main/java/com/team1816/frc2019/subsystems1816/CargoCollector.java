package com.team1816.frc2019.subsystems1816;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.team1816.frc2019.Robot;
import com.team1816.lib.checker.CheckFailException;
import com.team1816.lib.checker.Checkable;
import com.team1816.lib.checker.RunTest;
import com.team1816.lib.hardware.RobotFactory;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;


@RunTest
public class CargoCollector extends Subsystem implements Checkable {
    public static final String NAME = "cargocollector";

    private Solenoid armPiston;
    private IMotorController intake;

    private double intakePow;

    private boolean armDown;
    private boolean outputsChanged = false;

    public CargoCollector() {
        super(NAME);
        RobotFactory factory = Robot.getFactory();

        this.intake = factory.getMotor(NAME, "intake");
        this.armPiston = factory.getSolenoid(NAME, "arm");
    }

    public void setArm(boolean down) {
        this.armDown = down;
        outputsChanged = true;
        periodic();
    }

    public void setIntake(double intakePower) {
        this.intakePow = intakePower;
        outputsChanged = true;
    }

    public boolean isArmDown() {
        return armDown;
    }

    public double getIntakePow() {
        return this.intakePow;
    }

    @Override
    public void periodic() {
        if (outputsChanged) {
            this.intake.set(ControlMode.PercentOutput, intakePow);
            this.armPiston.set(armDown);
            outputsChanged = false;
        }
    }

    public void initDefaultCommand() {
    }

    @Override
    public boolean check() throws CheckFailException {
        System.out.println("Warning: mechanisms will move!");
        Timer.delay(3);

        setIntake(1.0);
        Timer.delay(0.5);
        setIntake(0.0);

        return true;
    }
}
