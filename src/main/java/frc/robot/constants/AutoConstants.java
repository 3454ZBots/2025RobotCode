package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;




public final class AutoConstants {
    
    /*
     * Basic info about how driving should work with autonomous mode
     * Includes values for all math/physics constants. 
     */
    public static final class AutoDriveConstants
    {
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
    }

   
}
