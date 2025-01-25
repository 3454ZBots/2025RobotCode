package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathSubsystem extends SubsystemBase{

    DriveSubsystem driveSubsystem;
    
    public PathSubsystem(DriveSubsystem driveSubsystem){

        this.driveSubsystem = driveSubsystem;


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
