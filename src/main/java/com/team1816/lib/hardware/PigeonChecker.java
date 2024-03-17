package com.team1816.lib.hardware;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.team1816.lib.util.logUtil.GreenLogger;

public class PigeonChecker {

    public static boolean checkPigeon(PigeonIMU pigeon) {
        boolean failure;
        GreenLogger.log("Checking: pigeon " + pigeon.getDeviceID());
        PigeonIMU.GeneralStatus status = new PigeonIMU.GeneralStatus();
        pigeon.getGeneralStatus(status);
        failure = status.state != PigeonIMU.PigeonState.Ready;
        if (failure) {
            GreenLogger.log("Failed! Status: " + status);
        }
        return !failure;
    }
}
