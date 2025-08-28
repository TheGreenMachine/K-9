package com.team1816.lib.hardware;

import com.team1816.lib.Singleton;
import com.team1816.lib.hardware.components.gyro.Pigeon2Impl;
import com.team1816.lib.hardware.components.led.CANdleImpl;
import com.team1816.lib.hardware.components.led.CANifierImpl;
import com.team1816.lib.hardware.components.motor.TalonFXImpl;
import com.team1816.lib.hardware.components.motor.TalonFXSImpl;
import com.team1816.lib.hardware.components.sensor.CanRangeImpl;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.LedManager;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class RobotFactoryTest {

    private RobotFactory factory;

    @BeforeEach
    public void testInit() {
        factory = Singleton.get(RobotFactory.class);
        factory.RobotIsReal = false;
    }

    @Test
    public void testFactoryLoad() {
        assertNotNull(factory);
    }

    @Test
    public void testGetConstant() {
        assertEquals(3, factory.getConstant("foo", 2), 0);
        assertEquals(3, factory.getConstant("subFoo", "foo", 3), 0);
    }

    @Test
    public void testGetSubsystem() {
        //non-existent subsystems should always be un implemented
        var subsystem = factory.getSubsystemConfig("foo");
        assertFalse(subsystem.implemented);
        // name should be set when getting subsystem
        assertEquals("foo", subsystem.name);
    }

    @Test
    public void testGetTalonFXlDevice() {
        factory.RobotIsReal = true;
        var device = factory.getDevice("drivetrain", "leftMain");
        assertNotNull(device);
        assertEquals(TalonFXImpl.class, device.getClass());
    }

    @Test
    public void testGetTalonFXSlDevice() {
        factory.RobotIsReal = true;
        var device = factory.getDevice("turret", "turret");
        assertNotNull(device);
        assertEquals(TalonFXSImpl.class, device.getClass());
    }

    @Test
    public void testGetCANifierDevice() {
        factory.RobotIsReal = true;
        var device = factory.getDevice("ledManager", "argb");
        assertNotNull(device);
        assertEquals(CANifierImpl.class, device.getClass());
    }

    @Test
    public void testGetCANdleDevice() {
        factory.RobotIsReal = true;
        var device = factory.getDevice("ledManager", "drgb");
        assertNotNull(device);
        assertEquals(CANdleImpl.class, device.getClass());
    }

    @Test
    public void testCreateSubsystem() {
        Singleton.CreateSubSystem (LedManager.class);
        // verify that subsystems can only be created once
        assertThrows(RuntimeException.class, () -> Singleton.CreateSubSystem(LedManager.class));
    }

    @Test
    public void testGetCANRangeDevice() {
        factory.RobotIsReal = true;
        var device = factory.getDevice("drivetrain", "range");
        assertNotNull(device);
        assertEquals(CanRangeImpl.class, device.getClass());
    }

    @Test
    public void testGetPigeonDevice() {
        factory.RobotIsReal = true;
        var device = factory.getDevice("drivetrain", "imu");
        assertNotNull(device);
        assertEquals(Pigeon2Impl.class, device.getClass());
    }

    @Test
    public void testDefaults() throws IllegalAccessException {
        ValidateFieldDefaults(new DeviceConfiguration());
        ValidateFieldDefaults(new SubsystemConfig());
        ValidateFieldDefaults(new PIDSlotConfiguration());
        ValidateFieldDefaults(new RobotConfiguration());
    }

    private void ValidateFieldDefaults(Object inst) throws IllegalAccessException {
        var clazz = inst.getClass();
        var fields = clazz.getFields();
        for (var field : fields) {
            switch (field.getName()) {
                case "canBusName" -> assertEquals("highspeed", field.get(inst));
                case "implemented" -> assertEquals(true, field.get(inst));
                case "id" -> assertEquals(-1, field.get(inst));
                case "invertSensor"-> assertEquals(false, field.get(inst));
                case "defaultAuto" -> assertEquals("", field.get(inst));
                default -> assertNull(field.get(inst), clazz.getSimpleName() + ":" + field.getName());
            }
        }
    }

    @Test
    public void testGetBadDevice() {
        assertThrows(IllegalArgumentException.class, () -> factory.getDevice("drivetrain", "foo"));
    }

    @Test
    public void testGetBadSubsystem() {
        var subsys = factory.getSubsystemConfig("foo");
        assertFalse(subsys.implemented);
    }

}
