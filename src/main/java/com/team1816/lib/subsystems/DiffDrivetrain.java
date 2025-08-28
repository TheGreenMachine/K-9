package com.team1816.lib.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.team1816.lib.hardware.DeviceConfiguration;
import com.team1816.lib.hardware.components.gyro.IGyro;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.hardware.components.motor.WpiMotorUtil;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import static com.team1816.lib.Singleton.factory;

public class DiffDrivetrain extends SubsystemBase implements IDrivetrain {

    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private double lastSimTime;
    private static DifferentialDriveOdometry odometry;
    private static DifferentialDriveKinematics kinematics;

    private IGyro gyro = (IGyro) factory.getDevice(NAME, "gyro");
    private IMotor flDr = (IMotor) factory.getDevice(NAME, "flDr");
    private IMotor frDr = (IMotor) factory.getDevice(NAME, "frDr");
    VelocityVoltage leftReq = new VelocityVoltage(0);
    VelocityVoltage rightReq = new VelocityVoltage(0);
    DifferentialDrivetrainSim driveSim;
    DeviceConfiguration drConf = config.devices.get("flDr");
    DCMotor driveMotor = WpiMotorUtil.getMotorConstants(drConf).withReduction(gearing);
    private SwerveDrivetrain.SwerveDriveState state = new SwerveDrivetrain.SwerveDriveState();

    public DiffDrivetrain() {
        GreenLogger.log("FieldCentric: " + false);
        configureAutoBuilder();
        odometry = new DifferentialDriveOdometry(new Rotation2d(gyro.getGyroYaw()), flDr.getMotorPosition(), frDr.getMotorPosition());
        var width = getKinematicsConfig().wheelbaseWidth;
        kinematics = new DifferentialDriveKinematics(width);
        if (Utils.isSimulation()) {
            // using 1 for gearing since driveMotor already has gearing
            driveSim = new DifferentialDrivetrainSim(
                driveMotor, 1, MOI, massKG, getKinematicsConfig().wheelRadius, width, null
            );
            startSimThread();
        }
    }

    private void configureAutoBuilder() {

        var moduleConfig = new ModuleConfig(whlRad, maxSpd, cof, driveMotor, factory.GetCurrentConfigs(drConf).StatorCurrentLimit, 1);

        RobotConfig robotConfig;
        PathFollowingController pathFollowingController;

        robotConfig = new RobotConfig(massKG, MOI, moduleConfig, config.kinematics.wheelbaseWidth);
        pathFollowingController = new PPLTVController(TimedRobot.kDefaultPeriod, maxSpd);

        GreenLogger.log(robotConfig);
        AutoBuilder.configure(
            () -> state.Pose,   // Supplier of current robot pose
            (pose) -> { // Consumer for seeding pose against auto
                GreenLogger.log("Setting " + pose);
                resetPose(pose);
            },
            () -> state.Speeds, // Supplier of current robot speeds
            this::setChassisSpeeds,
            pathFollowingController,
            robotConfig,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Subsystem for requirements
        );
    }

    // sets the wheel speeds based on desired chassis speed
    private void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        var wheel = kinematics.toWheelSpeeds(chassisSpeeds);
        var leftRotorRps = metersToRotations(clampVelocity(wheel.leftMetersPerSecond), true);
        var rightRotorRps = metersToRotations(clampVelocity(wheel.rightMetersPerSecond), true);
        flDr.setControl(leftReq.withVelocity(leftRotorRps));
        frDr.setControl(rightReq.withVelocity(rightRotorRps));
    }

    // updates the simulation values
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            var batVoltage = RobotController.getBatteryVoltage();
            flDr.setSimSupplyVoltage(batVoltage);
            frDr.setSimSupplyVoltage(batVoltage);
            gyro.setSimSupplyVoltage(batVoltage);

            // update the drivetrain sim
            driveSim.setInputs(flDr.getSimMotorVoltage(), frDr.getSimMotorVoltage());
            driveSim.update(deltaTime);

            // Get wheel speeds (m/s) and positions (meters)
            var leftVel = driveSim.getLeftVelocityMetersPerSecond();
            var rightVel = driveSim.getRightVelocityMetersPerSecond();
            var leftPos = driveSim.getLeftPositionMeters();
            var rightPos = driveSim.getRightPositionMeters();

            // Convert to rotor values
            double leftRotorVel = metersToRotations(leftVel, true); // Rotor RPS
            double rightRotorVel = metersToRotations(rightVel, true);
            double leftRotorPos = metersToRotations(leftPos, true); // Rotor rotations
            double rightRotorPos = metersToRotations(rightPos, true);

            // Update rotor sensor simulation
            flDr.setSimRotorPosition(leftRotorPos);
            flDr.setSimRotorVelocity(leftRotorVel);
            frDr.setSimRotorPosition(rightRotorPos);
            frDr.setSimRotorVelocity(rightRotorVel);

            // update gyro
            gyro.updateYaw(driveSim.getHeading().getDegrees());
        });
        m_simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    @Override
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {

        return run(() -> {
            var supplier = requestSupplier.get();
            if (supplier instanceof SwerveRequest.RobotCentric) {
                var req = (SwerveRequest.RobotCentric) supplier;
                setChassisSpeeds(kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
                    req.VelocityX + req.RotationalRate,
                    req.VelocityX - req.RotationalRate
                )));
            } else {
                setChassisSpeeds(kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(0, 0)));
            }
        });
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (Utils.isSimulation()) {
            driveSim.setPose(Pose2d.kZero);
        }
        // zero out the sensors and set initial pose
        flDr.zeroMotorPosition();
        frDr.zeroMotorPosition();
        gyro.updateYaw(0);
        odometry.resetPosition(new Rotation2d(0), new DifferentialDriveWheelPositions(0, 0), pose);
    }

    @Override
    public SwerveDrivetrain.SwerveDriveState getState() {
        // we are using SwerveDriveState as a common data paylod between the 2 drivetrain classes
        var leftDistance = rotationsToMeters(flDr.getMotorPosition(), true);
        var rightDistance = rotationsToMeters(frDr.getMotorPosition(), true);
        odometry.update(
            gyro.getRotation2d(),
            leftDistance,
            rightDistance
        );
        state.Pose = odometry.getPoseMeters();
        // speeds in m/s
        state.Speeds = kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(
                rotationsToMeters(flDr.getMotorVelocity(), true),
                rotationsToMeters(frDr.getMotorVelocity(), true)
            ));
        var tStamp = Utils.getCurrentTimeSeconds();
        state.OdometryPeriod = tStamp - state.Timestamp;
        state.Timestamp = tStamp;
        return state;
    }

    @Override
    public boolean IsFieldCentric() {
        return false;
    }

}
