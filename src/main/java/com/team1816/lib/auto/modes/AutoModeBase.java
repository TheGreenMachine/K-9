package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.auto.actions.NoopAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase {

    protected final double mUpdateRate = 1.0 / 50.0;
    protected boolean mActive = false;
    protected boolean mIsInterrupted = false;
    protected TrajectoryAction trajectory;

    protected abstract void routine() throws AutoModeEndedException;

    public Trajectory getTrajectory() {
        return trajectory.getTrajectory();
    }

    public void run() {
        mActive = true;

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }

        done();
    }

    public void done() {
        GreenLogger.log("Auto mode done");
    }

    public void stop() {
        mActive = false;
    }

    public boolean isActive() {
        return mActive;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void waitForDriverConfirm() throws AutoModeEndedException {
        if (!mIsInterrupted) {
            interrupt();
        }
        runAction(new NoopAction());
    }

    public void interrupt() {
        GreenLogger.log("** Auto mode interrrupted!");
        mIsInterrupted = true;
    }

    public void resume() {
        GreenLogger.log("** Auto mode resumed!");
        mIsInterrupted = false;
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        long waitTime = (long) (mUpdateRate * 1000.0);

        // Wait for interrupt state to clear
        while (isActiveWithThrow() && mIsInterrupted) {
            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.start();

        // Run action, stop action on interrupt, non active mode, or done
        while (isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {
            action.update();

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }

    public boolean getIsInterrupted() {
        return mIsInterrupted;
    }
}
