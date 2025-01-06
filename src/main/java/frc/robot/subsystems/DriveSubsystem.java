package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants.SwerveDriveConstants;

public class DriveSubsystem extends SubsystemBase {
    // Create Swerve Modules
    private final SwerveModule m_frontLeft = new SwerveModule(
        SwerveDriveConstants.kFrontLeftDrivingCanId,
        SwerveDriveConstants.kFrontLeftTurningCanId,
        SwerveDriveConstants.kFrontLeftChassisAngularOffset);

    private final SwerveModule m_frontRight = new SwerveModule(
        SwerveDriveConstants.kFrontRightDrivingCanId,
        SwerveDriveConstants.kFrontRightTurningCanId,
        SwerveDriveConstants.kFrontRightChassisAngularOffset);

    private final SwerveModule m_rearLeft = new SwerveModule(
        SwerveDriveConstants.kRearLeftDrivingCanId,
        SwerveDriveConstants.kRearLeftTurningCanId,
        SwerveDriveConstants.kBackLeftChassisAngularOffset);

    private final SwerveModule m_rearRight = new SwerveModule(
        SwerveDriveConstants.kRearRightDrivingCanId,
        SwerveDriveConstants.kRearRightTurningCanId,
        SwerveDriveConstants.kBackRightChassisAngularOffset);

    // The gyro sensor
    private Pigeon2 m_gyro = new Pigeon2(SwerveDriveConstants.PIGEON_ID);
    
    //These are used for displaying the robot pose
    private Field2d m_field = new Field2d(); 
    private Field2d fakefield = new Field2d(); 

    private boolean isFieldOriented = true;
    private boolean released = true; //is what released?

    //We still don't know how these work
    private Matrix<N3,N1> poseDeviations = VecBuilder.fill(0.1, 0.1, 0.1);
    private Matrix<N3,N1> visionDeviations = VecBuilder.fill(0.9, 0.9, 0.9);

    //This allows our swerve module states to be viewable in advantage scope
    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    private SwerveDriveOdometry m_odometry;
    private SwerveDrivePoseEstimator m_PoseEstimator;

    private RobotConfig pathConfig;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        m_gyro.setYaw(0); //Robot assumes it is facing perfectly forward initially, this is actually called when it first connects to driverstation
        resetEncoders();
        m_odometry = new SwerveDriveOdometry(SwerveDriveConstants.kDriveKinematics, getHeading(), getModulePositions());

        m_PoseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.kDriveKinematics, getHeading(), getModulePositions(), getPose(), poseDeviations, visionDeviations);
        
        SmartDashboard.putData("field pose", m_field);
        SmartDashboard.putData("Rotation 2D Pose", fakefield);


        
        try {
            pathConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        //PathPlanner setup
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> pathPlannerDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.0, 0.0, 0.0)  // Rotation PID constants
            ),
            pathConfig,
            this::isOnBlue,
            this // Reference to this subsystem to set requirements
        );
    }

    private boolean isOnBlue() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    //Called repeatedly
    @Override
    public void periodic() 
    {
        /*
         * All of this updates the the robot pose (i.e. where the robot thinks it is relative to the field)
         * and publishes that pose to advantageScope
         */
        m_odometry.update(
        getHeading(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
        m_PoseEstimator.update(getHeading(), getModulePositions());
        m_field.setRobotPose(m_PoseEstimator.getEstimatedPosition());


        //Publishing the state of each swerve module to advantage scope
        SwerveModuleState[] states = new SwerveModuleState[]
        {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        };
        publisher.set(states);
        
        //Info for debuging/PID tuning
        SmartDashboard.putNumber("FL Distance", m_frontLeft.getPosition().distanceMeters);
        SmartDashboard.putNumber("FR Distance", m_frontRight.getPosition().distanceMeters);
        SmartDashboard.putNumber("RL Distance", m_rearLeft.getPosition().distanceMeters);
        SmartDashboard.putNumber("RR Distance", m_rearRight.getPosition().distanceMeters);
            
        SmartDashboard.putNumber("FL Speed", m_frontLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("FR Speed", m_frontRight.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("RL Speed", m_rearLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("RR Speed", m_rearRight.getState().speedMetersPerSecond);

        //for troubleshooting
        fakefield.setRobotPose(new Pose2d(0, 0, getHeading()));

        
    }

    //Runs when simulating the robot
    @Override
    public void simulationPeriodic ()
    {
        m_PoseEstimator.update(Rotation2d.fromDegrees(m_gyro.getAngle() * -1), getModulePositions());
        m_field.setRobotPose(getPose());
    }

    /**
     * Returns the currently-estimated pose of the robot, i.e. where it thinks it is relative to the field
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }


 
    /**
     * Resets the odometry to the specified pose, used by path planner
     *
     * @param pose The pose to which to set the odometry.
    */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            getHeading(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose);
    }

    public void toggleFieldOriented(boolean pressed)
    {
        SmartDashboard.putBoolean("Field-Oriented", isFieldOriented);
        //Should be a better way to do this
        if(pressed && released)
        {
            isFieldOriented = !isFieldOriented;
            released = false;
        }
        if(!pressed)
        {
            released = true;
        }
    }

    /**
     * Drive method used by path planner, accepts a robot relative ChassisSpeeds object
     * @param speeds
     */
    private void pathPlannerDrive(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 1);
        SwerveModuleState[] swerveModuleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
        setModuleStates(swerveModuleStates);
    }




    /**
     * Intended default command of the DriveSubsystem for driving with controller input
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        //Dead Zones
        if (Math.abs(xSpeed) < 0.1) 
        {
            xSpeed = 0;
        }

        if (Math.abs(ySpeed) < 0.1) 
        {
            ySpeed = 0;
        }

        if (Math.abs(rot) < 0.1) 
        {
            rot = 0;
        }

        // Adjust input based on max speed
        xSpeed *= SwerveDriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= SwerveDriveConstants.kMaxSpeedMetersPerSecond;
        rot *= SwerveDriveConstants.kMaxAngularSpeed;

        SmartDashboard.putNumber("Rot", rot);
        SmartDashboard.putNumber("IMU Heading:", getHeading().getDegrees());
        SmartDashboard.putNumber("IMU Turn Rate", getTurnRate());

        /*
         * The speeds passed into this method represent the intended movement of the entire robot, either from its perspective or
         * relative to the field. This determines what each swerve module needs to do to achieve that
         */
        SwerveModuleState[] swerveModuleStates;
        if(isFieldOriented) {
            swerveModuleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading()));
        }
        else {
            swerveModuleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        }
        
        //Ensures no wheels are tryinh to move above max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }


    /**
     * 
     * @return ChassisSpeeds object with the current robot relative speeds
     */
    private ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }


    /**
     * 
     * @return array of swerve module states
     */
    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
        };
    }


    /**
     * Sets the swerve module states directly instead of using drive().
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }


    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }


    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading() {
        return m_gyro.getRotation2d();
    }


    public double getTurnRate() {
        return m_gyro.getRate() * (SwerveDriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    
    private SwerveModulePosition[] getModulePositions() 
    {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
    }

    /**
     * Sets the angle of the IMU, intended for when the robot starts agled relative to the field
     * @param angle
     */
    public void startAngle(double angle)
    {
        m_gyro.setYaw(angle);
    }
}
