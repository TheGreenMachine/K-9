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
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);    // forward/back
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);    // strafe
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);  // rotation
    private static double maxAngularRate = 0;

    private SwerveRequest GetSwerverCommand(SwerveRequest.FieldCentric drive) {

        // 1. Get raw joystick values (-1.0 to +1.0)
        double rawX    = -controller.getLeftY();    // forward/back  (negative because forward is usually negative Y)
        double rawY    = -controller.getLeftX();    // strafe left/right
        double rawRot  = -controller.getRightX();   // rotation

        // 2. Deadband (remove drift)
        rawX   = Math.abs(rawX)   < 0.08 ? 0 : rawX;
        rawY   = Math.abs(rawY)   < 0.08 ? 0 : rawY;
        rawRot = Math.abs(rawRot) < 0.08 ? 0 : rawRot;

        // 3. Cube the inputs → insane precision at low speed, full power at full stick
        double x    = rawX   * rawX   * rawX;
        double y    = rawY   * rawY   * rawY;
        double rot  = rawRot * rawRot * rawRot;

        // 4. Slew rate limit → buttery smooth acceleration
        x   = xLimiter.calculate(x);
        y   = yLimiter.calculate(y);
        rot = rotLimiter.calculate(rot);

        // 5. Slow mode (right bumper = precision mode)
        if (controller.rightBumper().getAsBoolean()) {
            x   *= 0.35;
            y   *= 0.35;
            rot *= 0.45;
        }
        return drive.withVelocityX(x * drivetrain.maxSpd) // Drive forward with negative Y (forward)
            .withVelocityY(y * drivetrain.maxSpd) // Drive left with negative X (left)
            .withRotationalRate(rot * maxAngularRate); // Drive counterclockwise with negative X (left)
    }

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
        maxAngularRate = RotationsPerSecond.of(kinematics.maxAngularRate).in(RadiansPerSecond);
        if (drivetrain.IsFieldCentric()) {
            SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> GetSwerverCommand(drive))
            );
        } else {
            SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-controller.getLeftY() * drivetrain.maxSpd) // Drive forward with negative Y (forward)
                        .withVelocityY(-controller.getLeftX() * drivetrain.maxSpd) // Drive left with negative X (left)
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
