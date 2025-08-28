package com.team1816.lib.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.team1816.lib.hardware.KinematicsConfig;
import com.team1816.lib.hardware.SubsystemConfig;
import com.team1816.lib.hardware.components.motor.WpiMotorUtil;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import static com.team1816.lib.Singleton.factory;
import static com.team1816.lib.util.FormatUtils.GetDisplay;

public class HoloDrivetrain extends SwerveDrivetrain<CommonTalon, CommonTalon, ParentDevice> implements IDrivetrain {

    SubsystemConfig config = factory.getSubsystemConfig(NAME);
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private double lastSimTime;
    private boolean fieldCentric;

    /**
     * Swerve request to apply during robot-centric path following
     */
    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    // Creates the CTRE swerve drivetrain.  The getDeviceById calls are made by the CTRE class and are based
    // on the defined values in the getSwerveModuleConstants
    public HoloDrivetrain() {
        super((id, bus) -> (CommonTalon) factory.getDeviceById(NAME, id),
            (id, bus) -> (CommonTalon) factory.getDeviceById(NAME, id),
            (id, bus) -> (ParentDevice) factory.getDeviceById(NAME, id),
            factory.getSwerveDrivetrainConstant(NAME),
            factory.getSwerveModuleConstants(NAME)
        );
        //default to filed centric
        fieldCentric = factory.getConstant(NAME, "fieldCentric", 1) == 1;
        GreenLogger.log("FieldCentric: " + fieldCentric);
        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> {
            this.setControl(requestSupplier.get());
        });
    }

    @Override
    public boolean IsFieldCentric() {
        return fieldCentric;
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    private void configureAutoBuilder() {

        var drConf = config.devices.get("flDr");
        var driveMotor = WpiMotorUtil.getMotorConstants(drConf).withReduction(gearing);
        var moduleConfig = new ModuleConfig(whlRad, maxSpd, cof, driveMotor, factory.GetCurrentConfigs(drConf).StatorCurrentLimit, 1);
        // In order of front left, front right, back left, back right
        var kinematics = new SwerveDriveKinematics(this.getModuleLocations());
        var modules = kinematics.getModules();
        RobotConfig robotConfig;
        PathFollowingController pathFollowingController;

        robotConfig = new RobotConfig(massKG, MOI, moduleConfig, modules);
        var tranKp = factory.getConstant(NAME, "translationKp", 5);
        var rotKp = factory.getConstant(NAME, "rotationKp", 5);
        GreenLogger.log(
            "translationKp:" + GetDisplay(tranKp) +
                " rotationKp:" + GetDisplay(rotKp)
        );
        pathFollowingController = new PPHolonomicDriveController(
            // PID constants for translation
            new PIDConstants(tranKp, 0, 0),
            // PID constants for rotation
            new PIDConstants(rotKp, 0, 0)
        );

        GreenLogger.log(robotConfig);
        AutoBuilder.configure(
            () -> getState().Pose,   // Supplier of current robot pose
            (pose) -> { // Consumer for seeding pose against auto
                GreenLogger.log("Setting " + pose);
                resetPose(pose);
            },
            () -> getState().Speeds, // Supplier of current robot speeds
            // Consumer of ChassisSpeeds and feedforwards to drive the robot
            (speeds, feedforwards) -> setControl(
                pathApplyRobotSpeeds.withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ),
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
}
