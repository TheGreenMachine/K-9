package com.team1816.lib.util;

import com.team254.lib.util.DriveSignal;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class GreenDriveHelperTest {

    private GreenDriveHelper greenDriveHelper;

    @Before
    public void testInit() {
        greenDriveHelper = new GreenDriveHelper();
    }

    @Test
    public void cheesyDriveTestQuickTurnPositive() {
        var result = greenDriveHelper.cheesyDrive(0, 1.0, true, false);
        assertInRange(result);
    }

    @Test
    public void cheesyDriveTestQuickTurnNegative() {
        var result = greenDriveHelper.cheesyDrive(0, -1.0, true, false);
        assertInRange(result);
    }

    @Test
    public void cheesyDriveTestQuickTurn50Positive() {
        var result = greenDriveHelper.cheesyDrive(0, .5, true, false);
        assertInRange(result);
    }

    @Test
    public void cheesyDriveTestQuickTurn50Negative() {
        var result = greenDriveHelper.cheesyDrive(0, -.5, true, false);
        assertInRange(result);
    }

    @Test
    public void cheesyDriveTestTurnNegative() {
        var result = greenDriveHelper.cheesyDrive(1.0, -1.0, false, false);
        assertInRange(result);
    }

    @Test
    public void cheesyDriveTestTurnPositive() {
        var result = greenDriveHelper.cheesyDrive(1.0, 1.0, false, false);
        assertInRange(result);
    }

    private void assertInRange(DriveSignal signal) {
        var value = signal.getLeft();
        Assert.assertTrue("Left exceeded 1.0 was " + value, value <= 1.0);
        Assert.assertTrue("Left exceeded -1.0 was " + value, value >= -1.0);
        value = signal.getRight();
        Assert.assertTrue("Right exceeded 1.0 was " + value, value <= 1.0);
        Assert.assertTrue("Right exceeded -1.0 was " + value, value >= -1.0);
    }
}
