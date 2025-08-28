package com.team1816.lib.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import com.team1816.lib.commands.SubsystemTestCommand;
import com.team1816.lib.events.PubSubConsumer;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.lib.hardware.components.led.CANifierImpl;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static com.team1816.lib.Singleton.factory;
import static com.team1816.lib.Singleton.pubsub;

public class LedManager extends SubsystemBase implements ITestableSubsystem {

    public static class RobotLEDStatusEvent extends PubSubConsumer<RobotLEDStatus>{}
    public static class RobotLEDStateEvent extends PubSubConsumer<LEDControlState>{}

    public static final String NAME = "ledManager";

    private static final int MAX = (int) factory.getConstant(NAME, "maxLevel", 255);
    private static final int LED_COUNT = (int) factory.getConstant(NAME, "ledCount", 8);
    private static final SolidColor SOLID_COLOR = new SolidColor(0, LED_COUNT + 8);

    private boolean blinkLedOn = false;
    private boolean outputsChanged = false;

    private int ledR;
    private int ledG;
    private int ledB;

    private final int period = 500; // ms
    private long lastWriteTime = System.currentTimeMillis();
    private LEDControlState controlState = LEDControlState.SOLID;
    private RobotLEDStatus lastStatus = RobotLEDStatus.OFF;
    private StatusCode lastStatusCode;

    private final IPhoenix6 candle = factory.getDevice(NAME, "drgb");
    private final IPhoenix6 canifier = factory.getDevice(NAME, "argb");

    public LedManager() {
        var ledStatus = pubsub.GetEvent(RobotLEDStatusEvent.class);
        ledStatus.Subscribe(this::setStatus);
        var ledState = pubsub.GetEvent(RobotLEDStateEvent.class);
        ledState.Subscribe(this::StateChanged);
    }

    private void StateChanged(LEDControlState ledControlState) {
        if(ledControlState != controlState) {
            controlState = ledControlState;
            outputsChanged = true;
            GreenLogger.log("LED Changed: " + controlState.name());
        }
    }

    private void setStatus(RobotLEDStatus robotLEDStatus) {
        if(robotLEDStatus != lastStatus) {
            lastStatus = robotLEDStatus;
            setLedColor(robotLEDStatus.red, robotLEDStatus.green, robotLEDStatus.blue);
        }
    }

    /**`
     * Base enum for LED states
     */
    public enum LEDControlState {
        FAST_BLINK,
        BLINK,
        SOLID,
    }

    private void setLedColor(int r, int g, int b) {
        if (ledR != r || ledG != g || ledB != b) {
            ledR = (int) (r / 255.0 * MAX);
            ledG = (int) (g / 255.0 * MAX);
            ledB = (int) (b / 255.0 * MAX);
            GreenLogger.log("LED Changed: " + lastStatus.name() + " r:" + ledR + " g:" + ledG + " b:" + ledB);
            outputsChanged = true;
        }
    }

    private void writeToLed(int r, int g, int b) {
        SOLID_COLOR.Color = new RGBWColor(r,g,b);
        lastStatusCode = candle.setControl(SOLID_COLOR);
        // special handling for Phoenix6 5 and ghosting to prevent cluttering up IPhoenix6 interface
        if (canifier instanceof CANifierImpl) {
            ((CANifierImpl)canifier).setLEDOutput(r / 255.0, CANifier.LEDChannel.LEDChannelB);
            ((CANifierImpl)canifier).setLEDOutput(g / 255.0, CANifier.LEDChannel.LEDChannelA);
            ((CANifierImpl)canifier).setLEDOutput(b / 255.0, CANifier.LEDChannel.LEDChannelC);
        }
    }

    @Override
    public void periodic() {
        if (controlState == LEDControlState.BLINK && System.currentTimeMillis() >= lastWriteTime + (period / 2)) {
            outputsChanged = true;
        }
        if (controlState == LEDControlState.FAST_BLINK && System.currentTimeMillis() >= lastWriteTime + (period / 8)) {
            outputsChanged = true;
        }
        if (outputsChanged) {
            outputsChanged = false;
            switch (controlState) {
                case FAST_BLINK, BLINK:
                    if (blinkLedOn) {
                        writeToLed(0, 0, 0);
                        blinkLedOn = false;
                    } else {
                        writeToLed(ledR, ledG, ledB);
                        blinkLedOn = true;
                    }
                    lastWriteTime = System.currentTimeMillis();
                    break;
                case SOLID:
                    writeToLed(ledR, ledG, ledB);
                    break;
            }
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
            controlState = LEDControlState.SOLID;
            setLedColor(MAX, 0, 0); // set red
            ref.passed = ref.passed && lastStatusCode == StatusCode.OK;
        }));
        group.addCommands(new WaitCommand(1.5));
        group.addCommands(runOnce(()->{
            setLedColor(0, MAX, 0); // set green
            ref.passed = ref.passed && lastStatusCode == StatusCode.OK;
        }));
        group.addCommands(new WaitCommand(1.5));
        group.addCommands(runOnce(()->{
            setLedColor(0, 0, MAX); // set blue
            ref.passed = ref.passed && lastStatusCode == StatusCode.OK;
        }));
        group.addCommands(new WaitCommand(1.5));
        group.addCommands(runOnce(()->{
            pubsub.GetEvent(SubsystemTestCommand.TestResult.class).Publish(ref.passed);
            GreenLogger.log("Testing " + NAME + " passed: " + ref.passed);
        }));
        return group;
    }

    /**
     * Base enum for RobotLEDStatus
     */
    public enum RobotLEDStatus {
        ENABLED(0, MAX, 0), // green
        DISABLED(MAX, MAX / 6, 0), // orange
        ERROR(MAX, 0, 0), // red
        AUTONOMOUS(0, MAX, MAX), // cyan
        ENDGAME(0, 0, MAX), // blue
        ON_TARGET(MAX, MAX, MAX), // white
        ZEROING(MAX, 0, MAX), // deep magenta, - robot needs zero
        OFF(0, 0, 0); // off

        final int red;
        final int green;
        final int blue;

        RobotLEDStatus(int r, int g, int b) {
            this.red = r;
            this.green = g;
            this.blue = b;
        }

    }
}
