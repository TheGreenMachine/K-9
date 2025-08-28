package com.team1816.lib;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.team1816.lib.subsystems.DiffDrivetrain;
import com.team1816.lib.subsystems.HoloDrivetrain;
import com.team1816.lib.subsystems.IDrivetrain;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class BaseRobotContainer {

    public static IDrivetrain drivetrain;
    private final CommandXboxController controller = new CommandXboxController(0);
    public SendableChooser<Command> autoChooser;
    private boolean poseInitialized;
    public static boolean IsHolonomic;

    public void InitializeLibSubSystems() {
        Singleton.CreateSubSystem(LedManager.class);

        var drvConf = Singleton.factory.getSubsystemConfig(IDrivetrain.NAME);
        IsHolonomic = drvConf.modules != null && drvConf.modules.size() > 2;
        if(IsHolonomic){
            drivetrain = Singleton.CreateSubSystem(HoloDrivetrain.class);
        } else {
            drivetrain = Singleton.CreateSubSystem(DiffDrivetrain.class);
        }

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        var kinematics = drivetrain.getKinematicsConfig();
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // setup teleop drivetrain command
        double maxAngularRate = RotationsPerSecond.of(kinematics.maxAngularRate).in(RadiansPerSecond);
        if (drivetrain.IsFieldCentric()) {
            SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-controller.getLeftY() * kinematics.maxDriveSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-controller.getLeftX() * kinematics.maxDriveSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-controller.getRightX() * maxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );
        } else {
            SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-controller.getLeftY() * kinematics.maxDriveSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-controller.getLeftX() * kinematics.maxDriveSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-controller.getRightX() * maxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );
        }
        autoChooser = AutoBuilder.buildAutoChooser(Singleton.factory.getDefaultAuto());
        SmartDashboard.putData("Auto Mode", autoChooser);
        autoChooser.onChange(this::updatePoseOnSelection);
    }

    public void updateInitialPose(){
        if(poseInitialized || DriverStation.getAlliance().isEmpty()) return;
        updatePoseOnSelection(autoChooser.getSelected());
    }

    private void updatePoseOnSelection(Command selectedAuto) {
        if (selectedAuto != null) {
            try {
                // Load the PathPlanner auto
                PathPlannerAuto auto = (PathPlannerAuto) selectedAuto;
                // Get the starting pose of the first path in the auto
                Pose2d startingPose = auto.getStartingPose();
                if (startingPose != null) {
                    var alliance = DriverStation.getAlliance();
                    if (!alliance.isEmpty() && alliance.get() == DriverStation.Alliance.Red) {
                        startingPose = FlippingUtil.flipFieldPose(startingPose);
                    }
                    // Reset odometry and update Field2d this is to give drivers clue that the
                    // proper auto is set prior to auto start
                    GreenLogger.log("Init " + startingPose);
                    drivetrain.resetPose(startingPose);
                    PPLibTelemetry.setTargetPose(startingPose);
                    poseInitialized = true;
                }
            } catch (Exception e) {
                GreenLogger.log("Error loading auto pose: " + e.getMessage());
            }
        }
    }
}
