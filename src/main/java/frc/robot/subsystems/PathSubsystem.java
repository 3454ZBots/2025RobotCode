package frc.robot.subsystems;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.Waypoint;

public class PathSubsystem extends SubsystemBase{

    DriveSubsystem driveSubsystem;
    
    public PathSubsystem(DriveSubsystem driveSubsystem){

        this.driveSubsystem = driveSubsystem;


    }

    public void followpath(){

        Pose2d startPose2d = driveSubsystem.getPose();
        Pose2d endPose2d = new Pose2d(startPose2d.getX() + 1,startPose2d.getY(),new Rotation2d());

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose2d, endPose2d);

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, new Rotation2d()));

        path.preventFlipping = true;

        AutoBuilder.followPath(path).schedule();

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
