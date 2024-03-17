package com.team1816.lib.hardware.components.motor.configurations;

/**
 * Enum containing all motor feedback device types
 *
 * @see com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice
 * @see com.revrobotics.SparkRelativeEncoder.Type
 */
public enum FeedbackDeviceType { // No enum inheritance :(
    // Near Universal
    NO_SENSOR,

    // Phoenix 5 Universal
    SENSOR_SUM,
    SENSOR_DIFFERENCE,
    REMOTE_SENSOR_0,
    REMOTE_SENSOR_1,
    SOFTWARE_EMULATED_SENSOR,

    // TalonFX
    INTEGRATED_SENSOR,
    REMOTE_CANCODER,

    // TalonSRX
    ANALOG,
    TACHOMETER,
    PULSE_WIDTH,
    ABSOLUTE_MAG_ENCODER,
    RELATIVE_MAG_ENCODER,

    // TalonSRX & SparkMax
    QUADRATURE,

    // SparkMax
    HALL_SENSOR
}
