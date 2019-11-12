package com.team1816.frc2019.subsystems;


import com.team1816.frc2019.Robot;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;


public class Birdbeak extends Subsystem {
    public static final String NAME = "birdbeak";
    private static Birdbeak INSTANCE;

    private Solenoid beak;
    private Solenoid hatchPuncher;

    private boolean beakNotGripped;
    private boolean puncherOut;
    private boolean outputsChanged = false;
    private boolean enableTimeout = false;
    private long timeoutStartTime = 0;

    private Birdbeak() {
        super(NAME);
        RobotFactory factory = Robot.getFactory();

        this.beak = factory.getSolenoid(NAME, "beak");
        this.hatchPuncher = factory.getSolenoid(NAME, "puncher");
    }

    public static Birdbeak getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Birdbeak();
        }
        return INSTANCE;
    }

    public void setBeak(boolean notGripped) {
        beakNotGripped = notGripped;
        outputsChanged = true;
    }

    public void setPuncher(boolean out) {
        puncherOut = out;
        outputsChanged = true;
    }

    public void setEject(boolean eject) {
        setPuncher(eject);
        enableTimeout = eject;
        setBeak(eject);
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
            hatchPuncher.set(puncherOut);
            if (enableTimeout) {
                if (timeoutStartTime == 0) {
                    timeoutStartTime = System.currentTimeMillis();
                }
                if (timeoutStartTime + 100 < System.currentTimeMillis()) {
                    beak.set(beakNotGripped);
                    enableTimeout = false;
                    timeoutStartTime = 0;
                    outputsChanged = false;
                }
            } else {
                beak.set(beakNotGripped);
                outputsChanged = false;
            }
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
