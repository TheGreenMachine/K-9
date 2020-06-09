package com.team1816.lib.hardware;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class RobotFactoryTest {

    private RobotFactory factory;

    @Before
    public void testInit() {
        factory = RobotFactory.getInstance();
    }

    @Test
    public void testGetInstance() {
        Assert.assertNotNull(RobotFactory.getInstance());
    }

    @Test
    public void testGetMotor() {
        Assert.assertTrue(factory.getMotor("fooDrive","left") instanceof GhostTalonSRX);
    }

    public void testGetSolenoid() {
    }

    public void testGetDoubleSolenoid() {
    }

    @Test
    public void testGetCanifier() {
        Assert.assertNull(factory.getCanifier("foo"));
    }

    @Test
    public void testGetConstant() {
        Assert.assertEquals(0,factory.getConstant("foo"),0);
        Assert.assertEquals(2,factory.getConstant("foo",2),0);
        Assert.assertEquals(0,factory.getConstant( "subFoo","foo"),0);
        Assert.assertEquals(3,factory.getConstant("subFoo","foo",3),0);
    }

    @Test
    public void testGetPcmId() {
        Assert.assertEquals(-1, factory.getPcmId());
    }

    @Test
    public void testGetSubsystem() {
        Assert.assertFalse(factory.getSubsystem("foo").implemented);
    }

    @Test
    public void testIsVerbose() {
        Assert.assertTrue(RobotFactory.isVerbose());
    }
}
