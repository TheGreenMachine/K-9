package com.team1816.lib.controlboard;

public class XboxController extends Controller {

    public XboxController(int port) {
        super(port);
        mJoystickButtonMap.put(Controller.Button.A, 1);
        mJoystickButtonMap.put(Controller.Button.B, 2);
        mJoystickButtonMap.put(Controller.Button.X, 3);
        mJoystickButtonMap.put(Controller.Button.Y, 4);
        mJoystickButtonMap.put(Controller.Button.LEFT_BUMPER, 5);
        mJoystickButtonMap.put(Controller.Button.RIGHT_BUMPER, 6);
        mJoystickButtonMap.put(Controller.Button.BACK, 7);
        mJoystickButtonMap.put(Controller.Button.START, 8);
        mJoystickButtonMap.put(Controller.Button.L_JOYSTICK, 9);
        mJoystickButtonMap.put(Controller.Button.R_JOYSTICK, 10);
        mJoystickAxisMap.put(Axis.LEFT_X, 0);
        mJoystickAxisMap.put(Axis.LEFT_Y, 1);
        mJoystickAxisMap.put(Axis.LEFT_TRIGGER, 2);
        mJoystickAxisMap.put(Axis.RIGHT_TRIGGER, 3);
        mJoystickAxisMap.put(Axis.RIGHT_X, 4);
        mJoystickAxisMap.put(Axis.RIGHT_Y, 5);
    }
}
