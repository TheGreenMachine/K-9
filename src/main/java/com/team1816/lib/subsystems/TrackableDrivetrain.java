package com.team1816.lib.subsystems;

public interface TrackableDrivetrain {
    double getLeftVelocityNativeUnits();
    double getRightVelocityNativeUnits();

    double getLeftVelocityDemand();
    double getRightVelocityDemand();

    double getLeftVelocityError();
    double getRightVelocityError();

    double getHeadingDegrees();
    double getDesiredHeading();
}
