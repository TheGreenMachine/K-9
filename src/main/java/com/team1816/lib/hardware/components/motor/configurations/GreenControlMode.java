package com.team1816.lib.hardware.components.motor.configurations;

/**
 * A general enum containing motor control modes
 *
 * @see com.ctre.phoenix.motorcontrol.ControlMode
 * @see com.revrobotics.CANSparkMax.ControlType
 */
public enum GreenControlMode {
    // Universal
    PERCENT_OUTPUT,
    VELOCITY_CONTROL,
    POSITION_CONTROL,
    MOTION_PROFILE,

    // All except TalonFX
    CURRENT,

    // TalonFX exclusive
    BRAKE,
    NEUTRAL,

    // CTRE Exclusive
    FOLLOWER,
    MOTION_MAGIC,
    MOTION_MAGIC_EXPO,
    MOTION_PROFILE_ARC,
    MUSIC_TONE,
    DISABLED,

    // TalonFX and REV
    VOLTAGE_CONTROL,

    // REV Exclusive
    SMART_VELOCITY,

}
