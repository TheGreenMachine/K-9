package com.team1816.season.subsystems;

import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.LibModule;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.RobotState;
import com.team1816.season.SeasonModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;
import org.mockito.Spy;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.when;

// @RunWith(JUnit4.class)
public class TurretTest {

    private final RobotState state;
    private Turret mTurret;

    @Spy
    private Constants constants;

    public TurretTest() {
        RobotFactory mockFactory = Mockito.spy(RobotFactory.class);
        when(mockFactory.getConstant(Turret.NAME, "absPosTicksSouth")).thenReturn(1980.0);
        Subsystem.factory = mockFactory;
        Injector injector = Guice.createInjector(new LibModule(), new SeasonModule());
        state = injector.getInstance(RobotState.class);
    }

    @Before
    public void setUp() {
        mTurret = new Turret();
        mTurret.zeroSensors();
        state.reset();
    }

    @Test
    public void fieldFollowingTest() {
        mTurret.setTurretAngle(0);
        mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
        mTurret.writePeriodicOutputs();
        mTurret.readPeriodicInputs();
        assertEquals(0, state.getLatestFieldToTurret(), 0.1);
        assertEquals(0, state.vehicle_to_turret.getDegrees(), .01);
        assertEquals(1980, mTurret.getActualTurretPositionTicks(), .01);
    }

    @Test
    public void fieldFollowing45Test() {
        mTurret.setTurretAngle(0);
        mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
        state.field_to_vehicle = new Pose2d(0, 0, Rotation2d.fromDegrees(45));
        mTurret.writePeriodicOutputs();
        mTurret.readPeriodicInputs();
        assertEquals(45, state.vehicle_to_turret.getDegrees(), .01);
        assertEquals(0, state.getLatestFieldToTurret(), 0.1);
        // Turret should move CW
        assertEquals(1980 + 512, mTurret.getActualTurretPositionTicks(), .01);
    }

    @Test
    public void fieldFollowing315Test() {
        mTurret.setTurretAngle(0);
        mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
        state.field_to_vehicle = new Pose2d(0, 0, Rotation2d.fromDegrees(-45));
        mTurret.writePeriodicOutputs();
        mTurret.readPeriodicInputs();
        assertEquals(315, state.vehicle_to_turret.getDegrees(), .01);
        assertEquals(0, state.getLatestFieldToTurret(), 0.1);
        // Turret should move CCW
        assertEquals(1980 - 512, mTurret.getActualTurretPositionTicks(), .01);
    }

    @Test
    public void convertTurretDegreesToTicksTest() {
        assertEquals(1980, Turret.convertTurretDegreesToTicks(0));
        assertEquals(1980, Turret.convertTurretDegreesToTicks(360));
        assertEquals(1980, Turret.convertTurretDegreesToTicks(720));
        assertEquals(1980 + 512, Turret.convertTurretDegreesToTicks(45));
        assertEquals(1980 - 512, Turret.convertTurretDegreesToTicks(-45));
    }

    @Test
    public void convertTurretTicksToDegrees() {
        assertEquals(0, Turret.convertTurretTicksToDegrees(1980), .01);
        assertEquals(45, Turret.convertTurretTicksToDegrees(1980 + 512), .01);
        assertEquals(315, Turret.convertTurretTicksToDegrees(1980 - 512), .01);
    }
}
