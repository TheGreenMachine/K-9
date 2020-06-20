package com.team1816.lib.hardware;

import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidWrapper implements ISolenoid{

    private final Solenoid solenoid;

    public SolenoidWrapper(Integer pcm, Integer solenoidId) {
        solenoid = new Solenoid(pcm,solenoidId);
    }

    public void set(boolean on) {
        solenoid.set(on);
    }
}
