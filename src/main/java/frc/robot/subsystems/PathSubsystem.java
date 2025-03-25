package frc.robot.subsystems;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutoConstants;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.robot.LimelightHelpers;
import com.pathplanner.lib.path.Waypoint;

public class PathSubsystem extends SubsystemBase{

    DriveSubsystem driveSubsystem;
    Field2d pathField;
    Field2d targetField;

    
    public PathSubsystem(DriveSubsystem driveSubsystem){

        this.driveSubsystem = driveSubsystem;
        pathField = new Field2d();
        targetField = new Field2d();
        SmartDashboard.putData("target field", targetField);
        SmartDashboard.putData("path field", pathField);

    }

    public void followpath(boolean isLeft){
        double id = LimelightHelpers.getFiducialID("limelight");

        //Tag filtering only accepts reef tags. ID 0 means no target found
        if(!((id > 5 && id < 12) || (id > 16 && id < 23))){
            return;
        }


        Pose2d startPose2d = driveSubsystem.getVisionPose();
        driveSubsystem.m_odometry.resetPose(startPose2d); //THIS IS MISSION CRITICAL
        Pose2d endPose2d;

        double[] tprs = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        Pose2d globalTargetPose = startPose2d.transformBy(new Transform2d(tprs[0], tprs[1], new Rotation2d()));
        if(isLeft) {
            endPose2d = globalTargetPose.transformBy(AutoConstants.LEFT_POST_TRANSFORM);
    
        }
        else {
            endPose2d = globalTargetPose.transformBy(AutoConstants.RIGHT_POST_TRANSFORM);
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose2d, endPose2d);

        PathConstraints constraints = new PathConstraints(3.0, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, new Rotation2d()));

        path.preventFlipping = true;
        
        //driveSubsystem.fakefield.setRobotPose(path.getPathPoses().get(path.getPathPoses().size()-1));
        
        //AutoBuilder.followPath(path).schedule();
        pathField.setRobotPose(endPose2d);
        targetField.setRobotPose(globalTargetPose);
        
        SmartDashboard.putNumber("tprs X", tprs[0]);
        SmartDashboard.putNumber("tprs Y", tprs[1]);
        SmartDashboard.putNumberArray("TPRS", tprs);


    }

    //Get Pose
    //Get current target
    //left or right
    //determine destination (i.e. final pose)
    //calculate difference (trajectory) between current pose and destination pose
    //pass that path into AutoBuilder to follow

    //how will this command get scheduled in conjunction with other commands?


    //https://pathplanner.dev/pplib-follow-a-single-path.html#manually-create-path-following-commands
}
