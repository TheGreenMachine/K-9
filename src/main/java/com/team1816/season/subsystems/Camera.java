package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

@Singleton
public class Camera {

    private static Camera INSTANCE;

    // Components
    private final NetworkTable networkTable;

    @Inject
    private static LedManager led;

    // State
    private double deltaXAngle;
    private double distance;
    private final NetworkTableEntry usingVision;

    private double rawCenterX;

    // Constants
    private static final double CAMERA_FOV = 87.0; // deg
    private static final double CAMERA_FOCAL_LENGTH = 350; // px
    private static final double VIDEO_WIDTH = 672.0; // px
    public static final double ALLOWABLE_AIM_ERROR = 1; // deg

    public Camera() {
        networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        usingVision = networkTable.getSubTable("Calibration").getEntry("VISION");
    }

    public double getDeltaXAngle() {
        return deltaXAngle;
    }

    public double getDistance() {
        return distance;
    }

    public double getRawCenterX() {
        return rawCenterX;
    }

    public void setEnabled(boolean enabled) {
        led.setCameraLed(enabled);
        usingVision.setBoolean(enabled);
    }
}
