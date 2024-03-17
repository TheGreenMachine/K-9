package com.team1816.lib.subsystems;

public class DrivetrainLogger {

    public static void init(TrackableDrivetrain drivetrain) {
        var subsystem = (Subsystem) drivetrain;
        subsystem.CreateSubSystemLog(
            "LeftActVel",
            drivetrain::getLeftVelocityNativeUnits
        );
        subsystem.CreateSubSystemLog(
            "RightActVel",
            drivetrain::getRightVelocityNativeUnits
        );
        subsystem.CreateSubSystemLog(
            "LeftVel",
            drivetrain::getLeftVelocityDemand
        );
        subsystem.CreateSubSystemLog(
            "RightVel",
            drivetrain::getRightVelocityDemand
        );
        subsystem.CreateSubSystemLog(
            "LeftError",
            drivetrain::getLeftVelocityError
        );
        subsystem.CreateSubSystemLog(
            "RightError",
            drivetrain::getRightVelocityError
        );
        subsystem.CreateSubSystemLog(
            "ActualHeading",
            drivetrain::getHeadingDegrees
        );
        subsystem.CreateSubSystemLog(
            "Heading",
            drivetrain::getDesiredHeading
        );
    }
}
