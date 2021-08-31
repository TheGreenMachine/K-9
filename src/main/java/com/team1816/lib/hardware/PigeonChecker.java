package com.team1816.lib.hardware;

import com.ctre.phoenix.sensors.PigeonIMU;

public class PigeonChecker {

    public static boolean checkPigeon(PigeonIMU pigeon) {
        boolean failure;
        System.out.println("Checking: pigeon " + pigeon.getDeviceID());
        PigeonIMU.GeneralStatus status = new PigeonIMU.GeneralStatus();
        pigeon.getGeneralStatus(status);
        failure = status.state != PigeonIMU.PigeonState.Ready;
        if (failure) {
            System.out.println("Failed! Status: " + status.toString());
        }
        return !failure;
    }
}
