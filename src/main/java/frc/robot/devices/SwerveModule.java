package frc.robot.devices;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule {
    public final CANSparkMax driveMotor;
    public final TalonFX steerMotor;
    public CANCoder absoluteEncoder;
    public RelativeEncoder relativeEncoder;

    public SwerveModule(int driveMotorPort, int steerMotorPort, int absoluteEncoderPort) {
        this.driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        this.steerMotor = new TalonFX(steerMotorPort);
        this.absoluteEncoder = new CANCoder(absoluteEncoderPort);
        this.relativeEncoder = driveMotor.getEncoder();
    }


    public void setMode(IdleMode mode) {
        driveMotor.setIdleMode(mode);
    }

    public double getVoltage() {
        return driveMotor.getBusVoltage();
    }




}
