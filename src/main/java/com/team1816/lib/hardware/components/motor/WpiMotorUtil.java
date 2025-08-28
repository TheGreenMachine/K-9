package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.team1816.lib.hardware.DeviceConfiguration;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.system.plant.DCMotor;

public class WpiMotorUtil {
    public static DCMotor getMotorConstants(DeviceConfiguration config) {
        switch (config.deviceType) {
            case TalonFX:
                return DCMotor.getKrakenX60(1);
            case TalonFXS:
                if (config.motorType == MotorArrangementValue.Brushed_DC) {
                    return DCMotor.getBag(1);
                }
                // assume minion
                return new DCMotor(12, 3.1, 200.46, 1.43, 753.98, 1);

        }
        GreenLogger.log(config.getClass().getName() + " is unknown using default motor constants");
        return DCMotor.getBag(1);
    }

    public static String getModuleName(int index) {
        switch (index) {
            case 0:
                return "fl";
            case 1:
                return "fr";
            case 2:
                return "bl";
            case 3:
                return "br";
        }
        return String.valueOf(index);
    }
}
