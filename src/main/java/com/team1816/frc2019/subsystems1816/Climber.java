package com.team1816.frc2019.subsystems1816;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frc2019.Robot;
import com.team1816.lib.checker.RunTest;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

@RunTest
public class Climber extends Subsystem {
    public static final String NAME = "climber";

    private IMotorControllerEnhanced climbMaster;
    private IMotorControllerEnhanced climbSlave;

    private DoubleSolenoid habPiston;
    private Value habPistonState = Value.kOff;

    private double motorPower;

    private boolean outputsChanged = false;

    private static final int kTimeoutMs = 100;

    public Climber() {
        RobotFactory factory = Robot.getFactory();

        this.climbMaster = (IMotorControllerEnhanced) factory.getMotor(NAME, "climbMaster");
        this.climbSlave =
                (IMotorControllerEnhanced) factory.getMotor(NAME, "climbSlave", climbMaster);
        this.habPiston = factory.getDoubleSolenoid(NAME, "habPiston");

        this.climbSlave.setInverted(true);

        this.climbMaster.enableCurrentLimit(true);
        this.climbMaster.configContinuousCurrentLimit(30, kTimeoutMs);
        this.climbMaster.configPeakCurrentLimit(35, kTimeoutMs);
        this.climbMaster.configPeakCurrentDuration(500, kTimeoutMs);

        this.climbMaster.set(ControlMode.PercentOutput, 0.0);
    }

    public void setClimberPower(double motorPow) {
        this.motorPower = motorPow;
        outputsChanged = true;
    }

    public void setHabPiston(Value state) {
        this.habPistonState = state;
    }

    public Value getHabPistonState() {
        return habPistonState;
    }

    public double getMotorPower() {
        return motorPower;
    }

    public void toggleClimberPiston() {
        if (habPistonState == Value.kReverse) {
            setHabPiston(Value.kOff);
            setHabPiston(Value.kForward);
            System.out.println("Set Climber Piston: k_fwd");
        } else {
            setHabPiston(Value.kOff);
            setHabPiston(Value.kReverse);
            System.out.println("Set Climber Piston: k_rev");
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            climbMaster.set(ControlMode.PercentOutput, motorPower);
            habPiston.set(habPistonState);
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
        setClimberPower(0.5);
        Timer.delay(3);
        setClimberPower(0);
        Timer.delay(0.5);
        setClimberPower(-0.5);
        Timer.delay(3);
        setClimberPower(0);
        Timer.delay(3);
        setHabPiston(Value.kForward);
        Timer.delay(3);
        setHabPiston(Value.kReverse);
        Timer.delay(3);
        setHabPiston(Value.kOff);
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }
}
