package com.team1816.frc2019.subsystems;


import com.team1816.frc2019.Robot;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class CameraMount extends Subsystem {
    public static final String NAME = "cameramount";

    private DoubleSolenoid camRetractor;

    private boolean outputsChanged = false;
    private Value camState = Value.kOff;

    public CameraMount() {
        super(NAME);
        RobotFactory factory = Robot.getFactory();

        this.camRetractor = factory.getDoubleSolenoid(NAME, "shifter");
    }

    public void setCameraPistonState(Value pistonState) {
        camState = pistonState;
        outputsChanged = true;
    }

    public Value getCameraPistonState() {
        return camState;
    }

    public void toggleCameraShifter() {
        if (camState == Value.kReverse) {
            setCameraPistonState(Value.kOff);
            setCameraPistonState(Value.kForward);
            System.out.println("set camera piston: k_fwd");
        } else {
            setCameraPistonState(Value.kOff);
            setCameraPistonState(Value.kReverse);
            System.out.println("set camera piston: k_rev");
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            camRetractor.set(camState);
            outputsChanged = false;
        }
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}
