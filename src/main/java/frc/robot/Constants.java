package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double steerReduction = (15.0 / 32.0) * (10.0 / 60.0);

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
                // Front right
                new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
                // Back left
                new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
                // Back right
                new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0));

        public static final double kMaxSpeed = 5880.0 / 60.0 / driveReduction * kWheelDiameterMeters * Math.PI;
        public static final double kMaxAngularSpeed = kMaxSpeed / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);

        public static final double steerRelativeEncoderPositionConversionFactor = 2.0 * Math.PI / 2048.0 * steerReduction;
        public static final double steerRelativeEncoderVelocityConversionFactor = steerRelativeEncoderPositionConversionFactor * 10.0;

        public static final int FL_Drive_Id = 1;
        public static final int FL_Steer_Id = 2;
        public static final int FL_Encoder_Id = 1;
        public static final double FL_Encoder_Offset = -Math.toRadians(180.263671875);

        public static final int FR_Drive_Id = 8;
        public static final int FR_Steer_Id = 3;
        public static final int FR_Encoder_Id = 3;
        public static final double FR_Encoder_Offset = -Math.toRadians(267.1875);

        public static final int BL_Drive_Id = 14;
        public static final int BL_Steer_Id = 4;
        public static final int BL_Encoder_Id = 2;
        public static final double BL_Encoder_Offset = -Math.toRadians(268.2421875);

        public static final int BR_Drive_Id = 2;
        public static final int BR_Steer_Id = 1;
        public static final int BR_Encoder_Id = 4;
        public static final double BR_Encoder_Offset = -Math.toRadians(153.544921875);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = ModuleConstants.kMaxSpeed / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = ModuleConstants.kMaxAngularSpeed / 5;// 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;// orig 3
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;// 2 //Orig 4
        public static final double kPXController = 5;
        public static final double kPYController = 5;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
        public static final double kGoToPointLinearP = 0;
        public static final double kGoToPointLinearF = 0.5;
        public static final double kGoToPointAngularP = 0;
        public static final double kGoToPointAngularF = 0;

        public static final double kPTranslationController = 320;
        public static final double kDTranslationController = 30;
        public static final double kPThetaController = 3;

        public static final double maxTrajectoryOverrunSeconds = 3;
        public static final double kMaxDistanceMetersError = 0.1;
        public static final double kMaxAngleDegreesError = 5;
    }

}