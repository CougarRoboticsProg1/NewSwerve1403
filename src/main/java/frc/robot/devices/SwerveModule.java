package frc.robot.devices;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
    private static final int CAN_TIMEOUT_MS = 250;

    private double absoluteEncoderResetIterations = 0;

    private final CANSparkMax driveMotor;
    private final TalonFX steerMotor;

    private final CANCoder absoluteEncoder;
    private final double absoluteEncoderOffset;
    private final RelativeEncoder driveRelativeEncoder;

    public SwerveModule(int driveMotorPort, int steerMotorNumber, int CANCoderNumber, double offset) {
        this.driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        this.steerMotor = new TalonFX(steerMotorNumber);
        this.absoluteEncoder = new CANCoder(CANCoderNumber);
        this.driveRelativeEncoder = driveMotor.getEncoder();
        this.absoluteEncoderOffset = offset;
        configEncoders();
        configSteerMotor();
        configDriveMotor();
    }

    public void configEncoders() {
        // Config absolute encoder
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = Math.toDegrees(this.absoluteEncoderOffset);
        config.sensorDirection = false;
        absoluteEncoder.configAllSettings(config, 250);
        absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 250);

        // Config drive relative encoder
        double drivePositionConversionFactor = Math.PI * ModuleConstants.kWheelDiameterMeters * ModuleConstants.driveReduction;
        driveRelativeEncoder.setPositionConversionFactor(drivePositionConversionFactor);
        driveRelativeEncoder.setVelocityConversionFactor(drivePositionConversionFactor / 60.0);

    }

    public void configSteerMotor() {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.slot0.kP = 0.5;
        motorConfiguration.slot0.kI = 0;
        motorConfiguration.slot0.kD = 5;
        motorConfiguration.voltageCompSaturation = 12;
        motorConfiguration.supplyCurrLimit.currentLimit = 20;
        motorConfiguration.supplyCurrLimit.enable = true;

        steerMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS);
        steerMotor.enableVoltageCompensation(true);
        steerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS);
        steerMotor.setSensorPhase(true);
        steerMotor.setInverted(TalonFXInvertType.CounterClockwise);
        steerMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setSelectedSensorPosition(
                getAbsoluteEncoderAbsoluteAngle() / ModuleConstants.steerRelativeEncoderPositionConversionFactor,
                0, CAN_TIMEOUT_MS);
        steerMotor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                STATUS_FRAME_GENERAL_PERIOD_MS,
                CAN_TIMEOUT_MS);
    }

    public void configDriveMotor() {
        driveMotor.setInverted(true);
        driveMotor.enableVoltageCompensation(12);
        driveMotor.setSmartCurrentLimit((int) 20);
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    }

    public double getSteerAngle() {
        double motorAngleRadians = steerMotor.getSelectedSensorPosition()
                * ModuleConstants.steerRelativeEncoderPositionConversionFactor;
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }
        return motorAngleRadians;
    }

    public void setControllerMode(IdleMode mode) {
        this.driveMotor.setIdleMode(mode);
    }

    public void setRampRate(double rate) {
        driveMotor.setOpenLoopRampRate(rate);
    }

    public double angleError(double targetAngle) {
        double steerAngle = getSteerAngle();
        double difference = steerAngle - getSteerAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead
        // of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngle += 2.0 * Math.PI;
        }
        return steerAngle - getSteerAngle();
    }

    public void set(double driveVoltage, double steerAngle) {
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        double difference = steerAngle - getSteerAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead
        // of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngle += 2.0 * Math.PI;
        }
        difference = steerAngle - getSteerAngle(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can
        // be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into
            // the range [0, 2pi)
            steerAngle += Math.PI;
            driveVoltage *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        this.driveMotor.setVoltage(driveVoltage);
        setReferenceAngle(steerAngle);
    }

    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = steerMotor.getSelectedSensorPosition()
                * ModuleConstants.steerRelativeEncoderPositionConversionFactor;

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
        // fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter
        // anymore.
        if (steerMotor.getSelectedSensorVelocity()
                * ModuleConstants.steerRelativeEncoderVelocityConversionFactor < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++absoluteEncoderResetIterations >= ENCODER_RESET_ITERATIONS) {
                absoluteEncoderResetIterations = 0;
                double absoluteAngle = getAbsoluteEncoderAbsoluteAngle();
                steerMotor.setSelectedSensorPosition(
                        absoluteAngle / ModuleConstants.steerRelativeEncoderPositionConversionFactor);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            absoluteEncoderResetIterations = 0;
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go
        // above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        steerMotor.set(TalonFXControlMode.Position,
                adjustedReferenceAngleRadians / ModuleConstants.steerRelativeEncoderPositionConversionFactor);
    }

    /**
     * Gets the current angle reading of the encoder in radians.
     *
     * @return The current angle in radians. Range: [0, 2pi)
     */
    public double getAbsoluteEncoderAbsoluteAngle() {
        double angle = Math.toRadians(absoluteEncoder.getAbsolutePosition());
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    public CANSparkMax getDriveMotor() {
        return this.driveMotor;
    }

    public TalonFX getSteerMotorNumber() {
        return this.steerMotor;
    }

    public CANCoder getCANCoder() {
        return this.absoluteEncoder;
    }

    public RelativeEncoder getRelativeEncoder() {
        return driveRelativeEncoder;
    }
}