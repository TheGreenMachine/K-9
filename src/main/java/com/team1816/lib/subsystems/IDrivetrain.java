package com.team1816.lib.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1816.lib.hardware.KinematicsConfig;
import com.team1816.lib.hardware.SubsystemConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import static com.team1816.lib.Singleton.factory;

public interface IDrivetrain extends ITestableSubsystem {
    // setup common variables for use by drivetrain implementations
    String NAME = "drivetrain";

    SubsystemConfig config = factory.getSubsystemConfig(NAME);
    double massKG = config.kinematics.robotMass;
    double whlRad = config.kinematics.wheelRadius;
    double maxSpd = (config.kinematics.maxDriveRPS / config.kinematics.driveGearing) * 2 * Math.PI * config.kinematics.wheelRadius;;
    double cof = config.kinematics.wheelCOF;
    double gearing = config.kinematics.driveGearing;
    double wheelCircumference = 2 * Math.PI * whlRad;

    // this is an approximation assumes mass is evenly spread over robot
    double MOI = (massKG * config.kinematics.wheelbaseWidth * config.kinematics.wheelbaseWidth + config.kinematics.wheelbaseLength * config.kinematics.wheelbaseLength) / 12;

    // Gets configuration for kinematics
    default KinematicsConfig getKinematicsConfig() {
        return config.kinematics;
    }

    boolean IsFieldCentric();

    Command applyRequest(Supplier<SwerveRequest> requestSupplier);

    void resetPose(Pose2d pose);

    SwerveDrivetrain.SwerveDriveState getState();

    // Converts meters to rotations using defined wheel circumference
    // set useGearing to true to account for gearing
    default double metersToRotations(double meters, boolean useGearing) {
        if (useGearing) {
            return meters / wheelCircumference * gearing;
        }
        return meters / wheelCircumference;
    }

    // Converts rotations to meters using defined wheel circumference
    // set useGearing to true to account for gearing
    default double rotationsToMeters(double rotations, boolean useGearing) {
        if (useGearing) {
            return rotations / gearing * wheelCircumference;
        }
        return rotations * wheelCircumference;
    }

    // returns a velocity clamped to the configured maximum
    default double clampVelocity(double velocity) {
        return Math.min(Math.max(velocity, -maxSpd), maxSpd);
    }
}
