package com.team1816.frc2019.subsystems1816;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.team1816.frc2019.Robot;
import com.team1816.lib.checker.RunTest;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;


@RunTest
public class CargoCollector extends Subsystem {
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
        writePeriodicOutputs();
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
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            this.intake.set(ControlMode.PercentOutput, intakePow);
            this.armPiston.set(armDown);
            outputsChanged = false;
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        System.out.println("Warning: mechanisms will move!");
        Timer.delay(3);

        setIntake(1.0);
        Timer.delay(0.5);
        setIntake(0.0);

        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }

}
