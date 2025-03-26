package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;




public final class AutoConstants {
    
    /*
     * Basic info about how driving should work with autonomous mode
     * Includes values for all math/physics constants. 
     */
    
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared);

    public static final Pose2d m_centered = new Pose2d(2.97+0.37, 4.04, new Rotation2d());

    public static final Transform2d RIGHT_POST_TRANSFORM = new Transform2d(new Translation2d(-Units.inchesToMeters(17.625), -Units.inchesToMeters(6.5)), new Rotation2d());
    public static final Transform2d LEFT_POST_TRANSFORM = new Transform2d(new Translation2d(-Units.inchesToMeters(17.625), Units.inchesToMeters(6.5)), new Rotation2d());

    public static final Pose2d[] EVERY_APRILTAG_POSE2D = {
        null,
        new Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.8), Rotation2d.fromDegrees(126)),
        new Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.2), Rotation2d.fromDegrees(234)),
        new Pose2d(Units.inchesToMeters(455.15), Units.inchesToMeters(317.15), Rotation2d.fromDegrees(270)),
        new Pose2d(Units.inchesToMeters(365.2), Units.inchesToMeters(241.64), Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(365.2), Units.inchesToMeters(75.39), Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300)),
        new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60)),
        new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120)),
        new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(180)),
        new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240)),
        new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.8), Rotation2d.fromDegrees(54)),
        new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.2), Rotation2d.fromDegrees(306)),
        new Pose2d(Units.inchesToMeters(325.68), Units.inchesToMeters(241.64), Rotation2d.fromDegrees(189)),
        new Pose2d(Units.inchesToMeters(325.68), Units.inchesToMeters(75.39), Rotation2d.fromDegrees(180)),
        new Pose2d(Units.inchesToMeters(235.73), Units.inchesToMeters(-0.15), Rotation2d.fromDegrees(90)),
        new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240)),
        new Pose2d(Units.inchesToMeters(144), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(180)),
        new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120)),
        new Pose2d(Units.inchesToMeters(193.1), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60)),
        new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(193.1), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300))
    };
    

   
}
