package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.SwerveConstants.SwerveModuleConstants;

public class SwerveModule {
    private final SparkMax m_drivingSparkMax;
    private final SparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingPIDController;
    private final SparkClosedLoopController m_turningPIDController;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake; //As opposed to coast
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());


    /**
     * Constructs a SwerveModule and reconfigures SparkMaxes accordingly
     */
    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

        /*
         * The following code reconfigures the SparkMaxes the way you would in
         * Rev Client, but none of it takes effect if burnFlash() is not called at the end
         */
        m_drivingSparkMax.restoreFactoryDefaults();
        m_turningSparkMax.restoreFactoryDefaults();

        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_drivingPIDController = m_drivingSparkMax.getPIDController();
        m_turningPIDController = m_turningSparkMax.getPIDController();
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);

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
         * needed. Is this becuase we are applying the conversions to the encoders and not
         * the actual motor controllers?
         */
        m_drivingEncoder.setPositionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor);
        m_drivingEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDrivingEncoderVelocityFactor);

        m_turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor);
        m_turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor);

        /*
         * This is necessary because the gearing in the swerve module means the motor
         * spins the opposite direction it turn the wheel in
         */
        m_turningEncoder.setInverted(SwerveModuleConstants.kTurningEncoderInverted);


        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput);
        m_turningPIDController.setPositionPIDWrappingMaxInput(SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Setting the PID constants, this doesn't matter if we aren't doing burnFlash()
        m_drivingPIDController.setP(SwerveModuleConstants.kDrivingP);
        m_drivingPIDController.setI(SwerveModuleConstants.kDrivingI);
        m_drivingPIDController.setD(SwerveModuleConstants.kDrivingD);
        m_drivingPIDController.setFF(SwerveModuleConstants.kDrivingFF);
        m_drivingPIDController.setOutputRange(SwerveModuleConstants.kDrivingMinOutput,
        SwerveModuleConstants.kDrivingMaxOutput);
        
       // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_turningPIDController.setP(SwerveModuleConstants.kTurningP);
        m_turningPIDController.setI(SwerveModuleConstants.kTurningI);
        m_turningPIDController.setD(SwerveModuleConstants.kTurningD);
        m_turningPIDController.setFF(SwerveModuleConstants.kTurningFF);
        m_turningPIDController.setOutputRange(SwerveModuleConstants.kTurningMinOutput,
        SwerveModuleConstants.kTurningMaxOutput);

        m_drivingSparkMax.setIdleMode(kDrivingMotorIdleMode);
        m_turningSparkMax.setIdleMode(kTurningMotorIdleMode);
        m_drivingSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kDrivingMotorCurrentLimit);
        m_turningSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);

        m_drivingSparkMax.burnFlash();
        m_turningSparkMax.burnFlash();

        
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
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            new Rotation2d(m_turningEncoder.getPosition()));

        /*
         * Manually converting units, shouldn't be necessary
         */
        double speedRPMs = (optimizedDesiredState.speedMetersPerSecond / 
            SwerveModuleConstants.kWheelCircumferenceMeters) * 60;
        double optimizedangle = optimizedDesiredState.angle.getRadians();

        SmartDashboard.putNumber("optimized angle", optimizedangle);
        /*
         * Command driving and turning SPARKS MAX towards their respective setpoints.
         * This actually tells the motors to move
         */
        m_drivingPIDController.setReference(speedRPMs, CANSparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(optimizedangle, CANSparkMax.ControlType.kPosition);

        //These should be usefull for PID tuning
        //SmartDashboard.putNumber("driving encoder - Can ID" + m_drivingSparkMax.getDeviceId(), m_drivingEncoder.getVelocity());
        //SmartDashboard.putNumber("desired speed (RPMs) for Spark " + m_drivingSparkMax.getDeviceId(), optimizedDesiredState.speedMetersPerSecond);


        //Updates the modules internal state
        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }


}
