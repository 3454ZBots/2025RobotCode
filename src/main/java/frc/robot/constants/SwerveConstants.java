package frc.robot.constants;



import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants 
{   
    //Ref: https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java
    public static final class SwerveModuleConstants
    {
        // The Swerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in theSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                                                             / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                                                                   / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                                                                   / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.00089;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = (0.00003 / kDriveWheelFreeSpeedRps);
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 40; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }
    
    //Swerve Drive Constants
    public static final class SwerveDriveConstants
    {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.2;
        public static final double kMaxAngularSpeed = 1.5 * Math.PI;

        //We should look into this
        //Slew code, makes robot move more smoothly
        //public static final double kDirectionSlewRate = 1.2; // radians per second
        //public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        //public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.6);
        // Distance between centers of right and left wheels on robot (I'm assuming this goes with ln 74)
        public static final double kWheelBase = Units.inchesToMeters(26.6);

        // Distance between front and back wheels on robot (I'm assuming this goes with that ln 76)
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), //Front Left
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //Front Right
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //Back Left
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //Back Right


        //The angular offsets make no sense but zeroeing absolute encoders will be necessary to fix this I think    
        public static final int kFrontLeftDrivingCanId = 6;
        public static final int kFrontLeftTurningCanId = 7;
        public static final double kFrontLeftChassisAngularOffset = Math.PI / 2;
        
        public static final int kFrontRightDrivingCanId = 4;
        public static final int kFrontRightTurningCanId = 5;
        public static final double kFrontRightChassisAngularOffset = Math.PI;
    
        public static final int kRearLeftDrivingCanId = 8;
        public static final int kRearLeftTurningCanId = 9;
        public static final double kBackLeftChassisAngularOffset = 0;
    
        public static final int kRearRightDrivingCanId = 2;
        public static final int kRearRightTurningCanId = 3;
        public static final double kBackRightChassisAngularOffset = -Math.PI / 2;
    
        public static final boolean kGyroReversed = false;
        public static final int PIGEON_ID = 10;
    }

    /* Constants relevant to the NEO motors specifically */
    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}