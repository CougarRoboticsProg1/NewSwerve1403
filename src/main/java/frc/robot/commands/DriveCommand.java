package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

public class DriveCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_fieldRelativeSupplier;
    private final DoubleSupplier m_brakeSupplier;
    private final BooleanSupplier m_increaseSpeedSupplier;
    private final BooleanSupplier m_decreaseSpeedSupplier;

    private boolean isFieldRelative;

    public DriveCommand(Drivetrain drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier fieldRelativeSupplier, 
                               DoubleSupplier m_brakeSupplier, 
                               BooleanSupplier m_increaesSpeedSupplier,
                               BooleanSupplier m_decreaseSpeedSuppliuer) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_fieldRelativeSupplier = fieldRelativeSupplier;
        this.m_brakeSupplier = m_brakeSupplier;
        isFieldRelative = false;
        this.m_increaseSpeedSupplier = m_increaesSpeedSupplier;
        this.m_decreaseSpeedSupplier = m_decreaseSpeedSuppliuer;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if(m_fieldRelativeSupplier.getAsBoolean()) {
            isFieldRelative = !isFieldRelative;
        }
        if(m_increaseSpeedSupplier.getAsBoolean()) {
            Drivetrain.increaseVoltage();
        }
        if(m_decreaseSpeedSupplier.getAsBoolean()) {
            Drivetrain.decreaseVoltage();
        }
        SmartDashboard.putBoolean("isFieldRelative", isFieldRelative);
        if(m_brakeSupplier.getAsDouble() == 0) {
            m_drivetrainSubsystem.setRobotIdleMode(IdleMode.kCoast);
            if(isFieldRelative) {
                m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            m_rotationSupplier.getAsDouble(),
                            m_drivetrainSubsystem.getGyroscopeRotation()
                    )
                );
            } else {
                m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                    m_translationXSupplier.getAsDouble(),
                    m_translationYSupplier.getAsDouble(), 
                    m_rotationSupplier.getAsDouble()));
            }
        } else {
           m_drivetrainSubsystem.setRobotIdleMode(IdleMode.kBrake);
           m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        }
        
        
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
