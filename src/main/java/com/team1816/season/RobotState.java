package com.team1816.season;

import com.google.inject.Singleton;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Singleton
public class RobotState {

    public final Field2d field = new Field2d();
    public Pose2d field_to_vehicle = Constants.EmptyPose;
    public Rotation2d vehicle_to_turret = Constants.EmptyRotation;

    public RobotState() {
        SmartDashboard.putData("Field", field);
        reset();
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(
        Pose2d initial_field_to_vehicle,
        Rotation2d initial_vehicle_to_turret
    ) {
        reset(initial_field_to_vehicle);
        vehicle_to_turret = initial_vehicle_to_turret;
    }

    public synchronized void reset(Pose2d initial_field_to_vehicle) {
        field_to_vehicle = initial_field_to_vehicle;
    }

    public synchronized void reset() {
        reset(Constants.StartingPose, Constants.EmptyRotation);
    }

    public synchronized Pose2d getLatestFieldToVehicle() {
        // CCW rotation increases degrees
        return field_to_vehicle;
    }

    public Double getLatestFieldToTurret() {
        return field_to_vehicle.getRotation().minus(vehicle_to_turret).getDegrees();
    }

    public synchronized void outputToSmartDashboard() {
        //shuffleboard periodic updates should be here
        field.setRobotPose(field_to_vehicle);
    }
}
