package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/*
 * Needs serious reworking
 */
public class VisionSubsystem extends SubsystemBase {

    NetworkTable table; 
    NetworkTableEntry tx, ty, ta, targetpose, botpose;
    double tX, tY, limeArea, limeX, limeY, limeZ, limeRoll, limePitch, limeYaw, botroll, botpitch, botyaw;
    boolean isorienting, hasPose;
    DriveSubsystem robotdrive;

    public VisionSubsystem(DriveSubsystem drivesystem) {
	    table = NetworkTableInstance.getDefault().getTable("limelight-unique");
	    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
	    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        robotdrive = drivesystem;
    }


    public void visionPeriodic() {
        //reference documentation: https://docs.limelightvision.io/en/latest/getting_started.html#basic-programming

		tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        //X Y Z in meters, Roll Pitch Yaw in degrees
        targetpose = table.getEntry("targetpose_robotspace");
        botpose = table.getEntry("botpose_targetspace");

        //read values periodically
        tX = tx.getDouble(0.0);
        tY = ty.getDouble(0.0);

        //tag area as percentage of image
        limeArea = ta.getDouble(0.0);

        limeZ = targetpose.getDoubleArray(new double[6])[2];
        limeX = targetpose.getDoubleArray(new double[6])[0];
        limeY = targetpose.getDoubleArray(new double[6])[1];
        limeRoll = targetpose.getDoubleArray(new double[6])[3];
        limePitch = targetpose.getDoubleArray(new double[6])[4];
        limeYaw = targetpose.getDoubleArray(new double[6])[5];
        botpitch = botpose.getDoubleArray(new double[6])[4];
        botroll = botpose.getDoubleArray(new double[6])[3];
        botyaw = botpose.getDoubleArray(new double[6])[5];

        //post to smart dashboard periodically
        /*
        SmartDashboard.putNumber("tx", tX);
        SmartDashboard.putNumber("ty", tY);
        SmartDashboard.putNumber("LimelightArea (ta)", limeArea);
        SmartDashboard.putNumber("LimelightZ", limeZ);
        SmartDashboard.putNumber("LimelightX", limeX);
        SmartDashboard.putNumber("LimelightY", limeY);
        SmartDashboard.putNumber("LimelightRoll", limeRoll);
        SmartDashboard.putNumber("LimelightPitch", limePitch);
        SmartDashboard.putNumber("LimelightYaw", limeYaw);
        SmartDashboard.putNumber("distance", estimateDistance());
        SmartDashboard.putNumber("botpitch", botpitch);
        SmartDashboard.putNumber("botroll", botroll);
        SmartDashboard.putNumber("botyaw", botyaw);
        */

        if(isorienting)
        {
            //orient();
        }
        


        //Old Correction Code
        /*
        // limelight distance variables  
        float kpDistance = -0.1f; // proportional control constant for distance
        float current_distance = (float) estimateDistance();
        float desired_distance = 1.1f; // how far you want to be from the april tag
        float kpAim = -0.1f;
        float min_aim_command = 0.05f;
        float distance_error= desired_distance - current_distance;
        float distance_adjust = kpDistance * distance_error;
        SmartDashboard.putNumber("DistanceAdjust", distance_adjust);
        SmartDashboard.putNumber("DistanceError", distance_error);
        double moveInputNum = 0.0;
        float heading_error = (float) (-1.0 * tX);
        float steering_adjust = 0.0f; 
		SmartDashboard.putNumber("Heading Error", heading_error);
			 
        if (tX > 1.0) {
            steering_adjust = kpAim * heading_error - min_aim_command;
        } else if (tX < -1.0) {
            steering_adjust = kpAim * heading_error + min_aim_command;
        }
        */
    }


    // public void orient()
    // {
    //     double goalPose = 0;
    //     double goalDist = 2;
    //     double offset = 0;
    //     if(Math.abs(limeYaw) - goalPose > 0.5 && !hasPose) 
    //     {
    //         if(limeYaw > goalPose)
    //         {
    //             robotdrive.drive(0, 0, -0.2, false);
    //         }
    //         else if(limeYaw < goalPose)
    //         {
    //             robotdrive.drive(0, 0, 0.2, false);
    //         }
    //     }
    //     else 
    //     {
    //         hasPose = true;
    //     }
    //     // x - side, z - dist, y - turn
    //     // currently works moving left towards target if the angle is correct
    //     //else
    //     if((Math.abs(limeX) > 0.1 || Math.abs(limeZ) - goalDist > 0.1) && hasPose)
    //     {
    //         if(limeZ > offset)
    //         {
    //             robotdrive.drive(-0.1, 0, 0, false);
    //         }
    //         else if(limeZ < offset)//goalPose)
    //         {
    //             robotdrive.drive(0.1, 0, 0, false);
    //         }

    //         if(limeX > offset)//goalPose)
    //         {
    //             robotdrive.drive(0, 0.1, 0, false);
    //         }
    //         else if(limeX < offset)//goalPose)
    //         {
    //             robotdrive.drive(0, -0.1, 0, false);
    //         }
    //     }

    //     else 
    //     {
    //         isorienting = false;
    //         robotdrive.drive(0, 0, 0, false);
    //     }
        
    // }





    public void beginOrienting()
    {
        isorienting = true;
        hasPose = false;
    }

    public void stopOrienting()
    {
        isorienting = false;
    }


   










































    public double estimateDistance()
	{
		NetworkTableEntry ty = table.getEntry("ty");
		double targetOffsetAngle_Vertical = ty.getDouble(0.0);
	
		// how many degrees back is your limelight rotated from perfectly vertical?
		double limelightMountAngleDegrees = 25.0; 
	
		// distance from the center of the Limelight lens to the floor
		double limelightLensHeightMeters = 0.485; 
	
		// distance from the target to the floor
		double goalHeightMeters = 1; 
	
		double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
		double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
	
		//calculate distance
		double distanceFromLimelightToGoalMeters = (goalHeightMeters - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);
		//SmartDashboard.putNumber("real distance from math", distanceFromLimelightToGoalMeters);
		return distanceFromLimelightToGoalMeters;
	}

}