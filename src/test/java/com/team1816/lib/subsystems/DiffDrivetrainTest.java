package com.team1816.lib.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.team1816.lib.Singleton;
import com.team1816.lib.hardware.DeviceConfiguration;
import com.team1816.lib.hardware.KinematicsConfig;
import com.team1816.lib.hardware.SubsystemConfig;
import com.team1816.lib.hardware.components.ICTREDevice;
import com.team1816.lib.hardware.components.gyro.IGyro;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.hardware.factory.RobotFactory;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Map;

import static com.team1816.lib.subsystems.IDrivetrain.NAME;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

class DiffDrivetrainTest {

    private DiffDrivetrain target;

    @BeforeEach
    void setUp() {
        // reset Singletons so other tests do not interfere
        Singleton.Reset();

        // create mocks for testing drivetrain
        KinematicsConfig mockKinematics = mock();
        mockKinematics.robotMass = 50.0;
        mockKinematics.wheelRadius = 0.0508;
        mockKinematics.maxDriveSpeed = 1.0;
        mockKinematics.driveGearing = 48.0;
        mockKinematics.wheelCOF = 1.0;
        mockKinematics.wheelbaseWidth = 1.0;
        mockKinematics.wheelbaseLength = 1.0;

        DeviceConfiguration mockDevice = mock();
        mockDevice.deviceType = ICTREDevice.DeviceType.TalonFX;

        Map<String, DeviceConfiguration> mockDevices = mock();
        when(mockDevices.get("flDr")).thenReturn(mockDevice);

        SubsystemConfig mockConfig = mock();
        mockConfig.kinematics = mockKinematics;
        mockConfig.devices = mockDevices;

        CurrentLimitsConfigs mockCurrent = mock();
        mockCurrent.StatorCurrentLimit = 100.0;

        IGyro mockGyro = mock();
        IMotor mockMotor = mock();

        RobotFactory mockFactory = mock();
        when(mockFactory.getDevice(NAME, "gyro")).thenReturn(mockGyro);
        when(mockFactory.getDevice(contains(NAME), contains("Dr"))).thenReturn(mockMotor);
        when(mockFactory.getSubsystemConfig(NAME)).thenReturn(mockConfig);
        when(mockFactory.GetCurrentConfigs(any(DeviceConfiguration.class))).thenReturn(mockCurrent);
        Singleton.registerMock(mockFactory);

        // now that mocks are created, create test instance
        target = Singleton.get(DiffDrivetrain.class);
    }

    @Test
    void metersToRotations() {
        assertEquals(82.7, target.metersToRotations(.55, true), 0.1);
        assertEquals(1.7, target.metersToRotations(.55, false), 0.1);
    }

    @Test
    void rotationsToMeters() {
        assertEquals(.55, target.rotationsToMeters(82.7, true), .01);
        assertEquals(.55, target.rotationsToMeters(1.7, false), .01);
    }

    @Test
    void clampVelocity() {
        assertEquals(1.0, target.clampVelocity(1.0));
        assertEquals(-1.0, target.clampVelocity(-1.0));
        assertEquals(1.0, target.clampVelocity(2.0));
        assertEquals(-1.0, target.clampVelocity(-2.0));
        assertEquals(-0.34, target.clampVelocity(-0.34));
    }
}
