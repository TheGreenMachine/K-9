package com.team1816.season;

import com.google.inject.Singleton;
import com.team1816.season.subsystems.Drive;
import com.team1816.season.subsystems.TankDrive;

@Singleton
public class SeasonFactory implements Drive.Factory {

    private static Drive mDrive;

    @Override
    public Drive getInstance() {
        if (mDrive == null) {
            mDrive = new TankDrive();
        }
        return mDrive;
    }
}
