package com.team1816.lib;

import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.Drivetrain;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

public class BaseRobotState {

    private final SwerveDriveKinematics kinematics;
    private final double WHEELBASE_LENGTH_METERS;
    private final double WHEELBASE_WIDTH_METERS;
    private SwerveDrivePoseEstimator poseEstimator;
    private Pose2d odometryPose = new Pose2d();
    private SwerveDriveOdometry odometry;
    public static RobotFactory factory = Singleton.get(RobotFactory.class);

    protected BaseRobotState() {
        WHEELBASE_WIDTH_METERS = factory.getConstant(Drivetrain.NAME,"wheelbaseWidth");
        WHEELBASE_LENGTH_METERS = factory.getConstant(Drivetrain.NAME,"wheelbaseLength");

        // In order of front left, front right, back left, back right
        kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEELBASE_LENGTH_METERS / 2, WHEELBASE_WIDTH_METERS / 2),
            new Translation2d(WHEELBASE_LENGTH_METERS / 2, -WHEELBASE_WIDTH_METERS / 2),
            new Translation2d(-WHEELBASE_LENGTH_METERS / 2, WHEELBASE_WIDTH_METERS / 2),
            new Translation2d(-WHEELBASE_LENGTH_METERS / 2, -WHEELBASE_WIDTH_METERS / 2));

        odometry = new SwerveDriveOdometry(
            kinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
            new Pose2d());

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
            new Pose2d(),
            VecBuilder.fill(Units.inchesToMeters(2.0), Units.inchesToMeters(2.0), Units.degreesToRadians(2.0)),
            VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(1.0), Units.degreesToRadians(2.0)));
    }

    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
