package com.team1816.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.team1816.frc2019.Robot;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;


public class CargoCollector extends Subsystem {
    public static final String NAME = "cargocollector";

    private Solenoid armPiston;
    private IMotorController intake;

    private double intakePow;
    private double delay;

    private boolean armDown;
    private boolean outputsChanged = false;

    private static CargoCollector INSTANCE;

    private CargoCollector() {
        super(NAME);
        RobotFactory factory = Robot.getFactory();

        this.intake = factory.getMotor(NAME, "intake");
        this.armPiston = factory.getSolenoid(NAME, "arm");
    }

    public static CargoCollector getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CargoCollector();
        }
        return INSTANCE;
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

    public boolean isArmDown(double startTime) {
        delay = Timer.getFPGATimestamp() - startTime;
        System.out.println("Time delay passed:" + delay);
        return armDown && delay > 0.25;
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
