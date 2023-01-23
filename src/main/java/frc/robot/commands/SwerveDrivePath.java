package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrivePath extends CommandBase {

    private Drivetrain drivetrain;

    private List<Translation2d> wayPoints;
    private final Pose2d startPose;
    private final Pose2d endPose;

    private TrajectoryConfig trajectoryConfig;
    private Trajectory trajectory;
    PIDController xController; 
    PIDController yController;
    ProfiledPIDController angleController;
    SwerveControllerCommand swerveControllerCommand;

    public SwerveDrivePath(Drivetrain drivetrain, double startAngle, double endAngle, List<Translation2d> wayPoints) {
        this.drivetrain = drivetrain;
        this.wayPoints = wayPoints;

        for (int i = 0; i < wayPoints.size(); i++) {
            double x = wayPoints.get(i).getX();
            double y = wayPoints.get(i).getY();
            wayPoints.set(i, wayPoints.get(i).plus(
            new Translation2d(-x + y, -y + x)));
            wayPoints.set(i, wayPoints.get(i).times(0.30478512648));
        }

        startPose = new Pose2d(wayPoints.get(0).getX(), wayPoints.get(0).getY(), Rotation2d.fromDegrees(startAngle));
        endPose = new Pose2d(wayPoints.get(wayPoints.size()-1).getX(), wayPoints.get(wayPoints.size()-1).getY() , Rotation2d.fromDegrees(endAngle));

        wayPoints.remove(0);
        wayPoints.remove(wayPoints.size()-1);

        trajectoryConfig = new TrajectoryConfig(
        ModuleConstants.kMaxSpeed,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(ModuleConstants.kDriveKinematics);

        xController = new PIDController(AutoConstants.kPTranslationController, 0, AutoConstants.kDTranslationController);
        yController = new PIDController(AutoConstants.kPTranslationController, 0, AutoConstants.kDTranslationController);

        angleController = new ProfiledPIDController(
            22, 0, 0, AutoConstants.kThetaControllerConstraints);
    }

    @Override
    public void initialize() {
        drivetrain.resetOdometry();
        drivetrain.zeroGyroscope();

        trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,
            wayPoints,
            endPose,
            trajectoryConfig);

        swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            drivetrain::getPose,
            ModuleConstants.kDriveKinematics,
            xController,
            yController,
            angleController,
            drivetrain::setModuleStates,
            drivetrain);

        swerveControllerCommand.schedule();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return swerveControllerCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
