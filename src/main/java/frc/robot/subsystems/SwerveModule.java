package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.SwerveConstants.SwerveModuleConstants;

public class SwerveModule {
    private final SparkFlex m_drivingSparkFlex;
    private final SparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingPIDController;
    private final SparkClosedLoopController m_turningPIDController;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake; //As opposed to coast
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    private SparkFlexConfig m_drivingConfig;
    private SparkMaxConfig m_turningConfig;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());


    /**
     * Constructs a SwerveModule and reconfigures SparkMaxes accordingly
     */
    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_drivingSparkFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

        
        


        m_drivingEncoder = m_drivingSparkFlex.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
        m_drivingPIDController = m_drivingSparkFlex.getClosedLoopController();
        m_turningPIDController = m_turningSparkMax.getClosedLoopController();

        m_drivingConfig = new SparkFlexConfig();
        m_turningConfig = new SparkMaxConfig();
        

        /*
         * This should change the units for driving
         * Rotations -> Meters
         * RPM -> Meters per second
         * 
         * and for turning
         * ? -> Radians
         * ? -> Radians per second
         * 
         * Its not clear this is taking effect and the conversions are done manually when
         * needed.
         */
        m_drivingConfig.encoder
            .positionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor)
            .velocityConversionFactor(SwerveModuleConstants.kDrivingEncoderVelocityFactor);

        m_turningConfig.absoluteEncoder.positionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor);
        m_turningConfig.absoluteEncoder.velocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor);


        // Setting the PID constants
        m_drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(SwerveModuleConstants.kDrivingP, SwerveModuleConstants.kDrivingI, SwerveModuleConstants.kDrivingD, SwerveModuleConstants.kDrivingFF)
            .outputRange(SwerveModuleConstants.kDrivingMinOutput, SwerveModuleConstants.kDrivingMaxOutput);
            
        
        m_turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(SwerveModuleConstants.kTurningP, SwerveModuleConstants.kTurningI, SwerveModuleConstants.kTurningD, SwerveModuleConstants.kTurningFF)
            .outputRange(SwerveModuleConstants.kTurningMinOutput, SwerveModuleConstants.kTurningMaxOutput);

        /*
         * This is necessary because the gearing in the swerve module means the motor
         * spins the opposite direction it turn the wheel in
         */
        m_turningConfig.absoluteEncoder.inverted(SwerveModuleConstants.kTurningEncoderInverted);

        

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_turningConfig.closedLoop
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput)
            .positionWrappingMaxInput(SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

        
        

        m_drivingConfig
            .idleMode(kDrivingMotorIdleMode)
            .smartCurrentLimit(SwerveModuleConstants.kDrivingMotorCurrentLimit);
        
        m_turningConfig
            .idleMode(kTurningMotorIdleMode)
            .smartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);
       

        m_drivingSparkFlex.configure(m_drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turningSparkMax.configure(m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
        //add description
        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            m_drivingEncoder.getPosition(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module, i.e. when we actually tell it what to do
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
        

        /*
         * Manually converting units, shouldn't be necessary
         */
        double speedRPMs = correctedDesiredState.speedMetersPerSecond;
        double optimizedangle = correctedDesiredState.angle.getRadians();

        // SmartDashboard.putNumber("optimized angle", optimizedangle);
        // SmartDashboard.putNumber("drivespeed", speedRPMs);
        // SmartDashboard.putNumber("angle from encoder", m_turningEncoder.getPosition());
        /*
         * Command driving and turning SPARKS MAX towards their respective setpoints.
         * This actually tells the motors to move
         */
        //m_drivingPIDController.setReference(speedRPMs, SparkMax.ControlType.kVelocity);
        //m_turningPIDController.setReference(optimizedangle, SparkMax.ControlType.kPosition);

        //These should be usefull for PID tuning
        // SmartDashboard.putNumber("driving encoder - Can ID" + m_drivingSparkFlex.getDeviceId(), m_drivingEncoder.getVelocity());
        SmartDashboard.putNumber("desired speed (RPMs) for Spark " + m_drivingSparkFlex.getDeviceId(), correctedDesiredState.speedMetersPerSecond);


        //Updates the modules internal state
        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }

    public void turnOneMotor() {
        m_turningSparkMax.set(0.2);
    }


}
