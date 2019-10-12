package com.team1816.frc2019.subsystems1816;


import com.team1816.frc2019.Robot;
import com.team1816.lib.checker.CheckFailException;
import com.team1816.lib.checker.Checkable;
import com.team1816.lib.checker.RunTest;
import com.team1816.lib.hardware.RobotFactory;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;


@RunTest
public class Birdbeak extends Subsystem implements Checkable {
    public static final String NAME = "birdbeak";

    private Solenoid beak;
    private Solenoid hatchPuncher;

    private boolean beakNotGripped;
    private boolean puncherOut;
    private boolean outputsChanged = false;

    public Birdbeak() {
        super(NAME);
        RobotFactory factory = Robot.getFactory();

        this.beak = factory.getSolenoid(NAME, "beak");
        this.hatchPuncher = factory.getSolenoid(NAME, "puncher");
    }

    public void setBeak(boolean notGripped) {
        beakNotGripped = notGripped;
        outputsChanged = true;
    }

    public void setPuncher(boolean out) {
        puncherOut = out;
        outputsChanged = true;
    }

    public boolean getBeakState() {
        return beak.get();
    }

    public boolean getPuncherState() {
        return hatchPuncher.get();
    }

    @Override
    protected void initDefaultCommand() {
    }

    @Override
    public void periodic() {
        if (outputsChanged) {
            beak.set(beakNotGripped);
            hatchPuncher.set(puncherOut);

            outputsChanged = false;
        }
    }

    @Override
    public boolean check() throws CheckFailException {
        System.out.println("Warning: mechanisms will move!");
        Timer.delay(3);

        setBeak(true);
        setPuncher(true);
        Timer.delay(0.5);
        setBeak(false);
        setPuncher(false);

        return true;
    }
}
