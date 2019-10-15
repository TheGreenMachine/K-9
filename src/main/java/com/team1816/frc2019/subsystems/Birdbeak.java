package com.team1816.frc2019.subsystems;


import com.team1816.frc2019.Robot;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;


public class Birdbeak extends Subsystem {
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
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            beak.set(beakNotGripped);
            hatchPuncher.set(puncherOut);

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

        setBeak(true);
        setPuncher(true);
        Timer.delay(0.5);
        setBeak(false);
        setPuncher(false);

        return true;
    }
}
