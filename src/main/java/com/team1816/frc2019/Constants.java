package com.team1816.frc2019;

import com.team1816.frc2019.subsystems.Drive;

public class Constants {
  public static double kLooperDt = Robot.getFactory().getConstant("kLooperDt");
  public static final double kDriveWheelTrackWidthInches = Robot.getFactory().getConstant("trackWidth");
  public static final double kDriveWheelDiameterInches = Robot.getFactory().getConstant("wheelDiameter");
  public static double kTrackScrubFactor = Robot.getFactory().getConstant("kTrackScrubFactor");
  public static int kPCMId =  Robot.getFactory().getConstant("kPCMId").intValue();
  public static final int kCANTimeoutMs = 10; // use for important on the fly updates
  public static final int kLongCANTimeoutMs = 100; // use for constructors

  // reset button
  public static final int kResetButtonChannel = 4;

  // Control Board
  public static final boolean kUseDriveGamepad = false;
  public static final int kDriveGamepadPort = 0;
  public static final int kButtonGamepadPort = 2;
  public static final int kMainThrottleJoystickPort = 0;
  public static final int kMainTurnJoystickPort = 1;
  public static final double kJoystickThreshold = 0.2;

  // Canifier
  public static int kCanifierWristId = Robot.getFactory().getConstant("canifier","canId").intValue();

  public static final double kMaxTrackerDistance = 9.0;
  public static final double kMaxGoalTrackAge = 2.5;
  public static final double kMaxGoalTrackAgeNotTracking = 0.1;
  public static final double kMaxGoalTrackSmoothingTime = 0.5;
  public static final double kTrackStabilityWeight = 0.0;
  public static final double kTrackAgeWeight = 10.0;
  public static final double kTrackSwitchingWeight = 100.0;

  public static double kDriveVoltageRampRate = 0.0;

  public static final double kMinLookAhead = 12.0; // inches
  public static final double kMinLookAheadSpeed = 12.0; // inches per second
  public static final double kMaxLookAhead = 48.0; // inches
  public static final double kMaxLookAheadSpeed = 120.0; // inches per second
  public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
  public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;
  public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain * our speed in inches per sec
  public static final double kPathFollowingProfileKp = 0.3 / 12.0;  // % throttle per inch of error
  public static final double kPathFollowingProfileKi = 0.0;
  public static final double kPathFollowingGoalPosTolerance = 3.0;
  public static final double kPathFollowingGoalVelTolerance = 12.0;
  public static final double kPathStopSteeringDistance = 12.0;
  public static double kCameraFrameRate = 30;
  public static final double kPathFollowingMaxAccel = Robot.getFactory().getConstant("maxVel");
  public static final double kPathFollowingMaxVel = Robot.getFactory().getConstant("maxAccel");;
}
