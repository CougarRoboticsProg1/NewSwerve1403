// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.CSE.CougarScriptObject;
import frc.robot.CSE.CougarScriptReader;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrainSubsystem = Drivetrain.getInstance();
  private final XboxController m_controller = new XboxController(0);
  private CougarScriptReader reader;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * ModuleConstants.kMaxSpeed,
            () -> -modifyAxis(m_controller.getLeftX()) * ModuleConstants.kMaxSpeed,
            () -> -modifyAxis(m_controller.getRightX()) * ModuleConstants.kMaxAngularSpeed,
            () -> m_controller.getAButton(),
            () -> m_controller.getRightTriggerAxis(),
            () -> m_controller.getRightBumper(),
            () -> m_controller.getLeftBumper()
    ));

    // new CougarScriptReader((Pose2d startPose) -> {
    //   double feetToMeters = 0.30478512648;

    //   Translation2d flippedXandY = new Translation2d(
    //     startPose.getY() * feetToMeters, startPose.getX() * feetToMeters);

    //   Rotation2d theta = new Rotation2d(
    //     startPose.getRotation().getDegrees());
      
    //   Pose2d transformedStartPose;

    //   transformedStartPose = new Pose2d(flippedXandY, theta);
    //   m_drivetrainSubsystem.setPose(transformedStartPose);
    // });

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getYButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope, m_drivetrainSubsystem);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // // Square the axis
    // value = Math.copySign(value * value, value);

    return value;
  }

  public CougarScriptReader getCougarScriptReader() {
    return reader;
  }
}
