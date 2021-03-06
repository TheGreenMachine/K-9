package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends Subsystem implements PidProvider {

    public static final double TURRET_JOG_SPEED = 0.15;
    public static final double CARDINAL_SOUTH = 0; // deg
    public static final double CARDINAL_WEST = 90; // deg
    public static final double CARDINAL_NORTH = 180; // deg
    public static final double CARDINAL_EAST = 270; // deg
    private static final String NAME = "turret";
    public static final int TURRET_LIMIT_REVERSE =
        ((int) factory.getConstant(NAME, "minPos"));
    public static final int TURRET_LIMIT_FORWARD =
        ((int) factory.getConstant(NAME, "maxPos"));
    public static final int ABS_TICKS_SOUTH =
        ((int) factory.getConstant(NAME, "absPosTicksSouth"));
    // Constants
    private static final int kPrimaryCloseLoop = 0;
    private static final int kPIDGyroIDx = 0;
    private static final int kPIDVisionIDx = 1;
    private static final int TURRET_ENCODER_PPR = (int) factory.getConstant(
        NAME,
        "encPPR"
    );
    private static final int TURRET_ENCODER_MASK = TURRET_ENCODER_PPR - 1;
    private static final int ALLOWABLE_ERROR_TICKS = 5;
    private static Turret INSTANCE;
    // Components
    private final IMotorControllerEnhanced turret;
    private final Camera camera = Camera.getInstance();
    private final RobotState robotState = RobotState.getInstance();
    private final LedManager led = LedManager.getInstance();
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    // State
    private int desiredTurretPos = 0;
    private int followingTurretPos = 0;
    private double turretSpeed;
    private boolean outputsChanged;
    private double turretAngleRelativeToField;
    private ControlMode controlMode = ControlMode.MANUAL;

    public Turret() {
        super(NAME);
        this.turret = factory.getMotor(NAME, "turret");

        turret.setNeutralMode(NeutralMode.Brake);

        SmartDashboard.putNumber("TURRET_POSITION_MIN", TURRET_LIMIT_REVERSE);
        SmartDashboard.putNumber("TURRET_POSITION_MAX", TURRET_LIMIT_FORWARD);

        this.kP = factory.getConstant(NAME, "kP");
        this.kI = factory.getConstant(NAME, "kI");
        this.kD = factory.getConstant(NAME, "kD");
        this.kF = factory.getConstant(NAME, "kF");

        synchronized (this) {
            this.zeroSensors();

            // Position Control
            double peakOutput = 0.75;

            turret.configPeakOutputForward(peakOutput, Constants.kCANTimeoutMs);
            turret.configNominalOutputForward(0, Constants.kCANTimeoutMs);
            turret.configNominalOutputReverse(0, Constants.kCANTimeoutMs);
            turret.configPeakOutputReverse(-peakOutput, Constants.kCANTimeoutMs);
            turret.configAllowableClosedloopError(
                kPIDGyroIDx,
                ALLOWABLE_ERROR_TICKS,
                Constants.kCANTimeoutMs
            );
            turret.configAllowableClosedloopError(
                kPIDVisionIDx,
                ALLOWABLE_ERROR_TICKS,
                Constants.kCANTimeoutMs
            );

            // Soft Limits
            turret.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
            turret.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
            turret.configForwardSoftLimitThreshold(
                TURRET_LIMIT_FORWARD,
                Constants.kCANTimeoutMs
            ); // Forward = MAX
            turret.configReverseSoftLimitThreshold(
                TURRET_LIMIT_REVERSE,
                Constants.kCANTimeoutMs
            ); // Reverse = MIN
            turret.overrideLimitSwitchesEnable(true);
            turret.overrideSoftLimitsEnable(true);

            turretAngleRelativeToField =
                robotState.getHeadingRelativeToInitial().getDegrees();
        }
    }

    public static Turret getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Turret();
        }
        return INSTANCE;
    }

    public static int convertTurretDegreesToTicks(double degrees) {
        return (int) (((degrees) / 360.0) * TURRET_ENCODER_PPR) + ABS_TICKS_SOUTH;
    }

    public static double convertTurretTicksToDegrees(double ticks) {
        var adjTicks = (ticks - ABS_TICKS_SOUTH);
        return adjTicks / TURRET_ENCODER_PPR * 360;
    }

    @Override
    public synchronized void zeroSensors() {
        if (turret instanceof TalonSRX) {
            var sensors = ((TalonSRX) turret).getSensorCollection();
            sensors.setQuadraturePosition(
                sensors.getPulseWidthPosition() & TURRET_ENCODER_MASK,
                Constants.kLongCANTimeoutMs
            );
        }
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(ControlMode controlMode) {
        if (this.controlMode != controlMode) {
            if (controlMode == ControlMode.CAMERA_FOLLOWING) {
                if (Constants.kUseAutoAim) {
                    turret.selectProfileSlot(kPIDVisionIDx, 0);
                    this.controlMode = controlMode;
                    camera.setEnabled(true);
                    led.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
                }
            } else {
                turret.selectProfileSlot(kPIDGyroIDx, 0);
                this.controlMode = controlMode;
                camera.setEnabled(false);
                if (controlMode == ControlMode.MANUAL) {
                    led.indicateStatus(LedManager.RobotStatus.MANUAL_TURRET);
                } else {
                    led.indicateDefaultStatus();
                }
            }
        }
    }

    @Override
    public double getKP() {
        return kP;
    }

    @Override
    public double getKI() {
        return kI;
    }

    @Override
    public double getKD() {
        return kD;
    }

    @Override
    public double getKF() {
        return kF;
    }

    public void setTurretSpeed(double speed) {
        setControlMode(ControlMode.MANUAL);
        if (turretSpeed != speed) {
            turretSpeed = speed;
            outputsChanged = true;
        }
    }

    public synchronized void setTurretPosition(double position) {
        //Since we are using position we need ensure value stays in one rotation
        int adjPos = (int) position & TURRET_ENCODER_MASK;
        if (desiredTurretPos != adjPos) {
            desiredTurretPos = adjPos;
            outputsChanged = true;
        }
    }

    public synchronized void setTurretAngle(double angle) {
        setControlMode(ControlMode.POSITION);
        setTurretAngleInternal(angle);
    }

    private synchronized void setTurretAngleInternal(double angle) {
        setTurretPosition(convertTurretDegreesToTicks(angle));
    }

    public synchronized void lockTurret() {
        setTurretAngle(getActualTurretPositionDegrees());
    }

    public double getActualTurretPositionDegrees() {
        return convertTurretTicksToDegrees(getActualTurretPositionTicks());
    }

    public int getActualTurretPositionTicks() {
        return (int) turret.getSelectedSensorPosition(kPrimaryCloseLoop);
    }

    public double getTargetPosition() {
        return followingTurretPos;
    }

    public double getPositionError() {
        return turret.getClosedLoopError(kPrimaryCloseLoop);
    }

    @Override
    public void readPeriodicInputs() {
        turretAngleRelativeToField =
            robotState.getHeadingRelativeToInitial().getDegrees();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (controlMode) {
            case CAMERA_FOLLOWING:
                autoHome();
                positionControl();
                break;
            case FIELD_FOLLOWING:
                trackGyro();
                positionControl();
                break;
            case POSITION:
                followingTurretPos = 0;
                positionControl();
                break;
            case MANUAL:
                manualControl();
                break;
        }
    }

    private void autoHome() {
        var angle = camera.getDeltaXAngle();
        int adj =
            convertTurretDegreesToTicks(angle * .14) +
            followingTurretPos -
            ABS_TICKS_SOUTH;
        System.out.println(angle + " " + adj + " " + followingTurretPos);
        if (adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    private void trackGyro() {
        int fieldTickOffset =
            convertTurretDegreesToTicks(turretAngleRelativeToField) - ABS_TICKS_SOUTH;
        int adj = desiredTurretPos + fieldTickOffset;
        // Valid positions are 0 to encoder max ticks if we go negative adjust
        if (adj < 0) adj += TURRET_ENCODER_PPR;
        if (adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    private void positionControl() {
        if (outputsChanged) {
            turret.set(
                com.ctre.phoenix.motorcontrol.ControlMode.Position,
                followingTurretPos
            );
            outputsChanged = false;
        }
    }

    private void manualControl() {
        if (outputsChanged) {
            if (turretSpeed == 0) {
                turret.set(
                    com.ctre.phoenix.motorcontrol.ControlMode.Position,
                    getActualTurretPositionTicks() + 200 * turret.getMotorOutputPercent()
                );
            } else {
                turret.set(
                    com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,
                    turretSpeed
                );
            }
            outputsChanged = false;
        }
    }

    @Override
    public void stop() {
        camera.setEnabled(false);
    }

    @Override
    public boolean checkSystem() {
        boolean passed;
        turret.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, .2);
        Timer.delay(2);
        var ticks = getActualTurretPositionTicks();
        var diff = Math.abs(ticks - TURRET_LIMIT_FORWARD);
        System.out.println(" + TICKS: " + ticks + "  ERROR: " + diff);
        passed = diff <= 50;
        turret.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, -.2);
        Timer.delay(2);
        ticks = getActualTurretPositionTicks();
        diff = Math.abs(ticks - TURRET_LIMIT_REVERSE);
        System.out.println(" - TICKS: " + ticks + "  ERROR: " + diff);
        passed = passed & diff <= 50;
        turret.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
        return passed;
    }

    public enum ControlMode {
        FIELD_FOLLOWING,
        CAMERA_FOLLOWING,
        POSITION,
        MANUAL,
    }
}
