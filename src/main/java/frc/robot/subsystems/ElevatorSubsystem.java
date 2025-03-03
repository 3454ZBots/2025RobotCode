package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.MechanismConstants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.VoltageUnit;


public class ElevatorSubsystem extends SubsystemBase {


    private SparkMax motor = new SparkMax(MechanismConstants.ELEVATOR_RIGHT_ID, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private DigitalInput bottomSwitch = new DigitalInput(0);

    private static double kDt = 0.02;
    private static double kMaxVelocity = 0.3;
    private static double kMaxAcceleration = 0.05;
    private static double kP = 0.0;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kS = 0.0;
    private static double kG = 0.0;
    private static double kV = 0.0;


    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);
    
    //10.5 dynamic test time
    private Config sysConfig = new SysIdRoutine.Config(Volts.of(0.9).per(Second), Voltage.ofBaseUnits(1.4, Volts), Time.ofBaseUnits(10.5, Second));

    // Creates a SysIdRoutine
    SysIdRoutine routine = new SysIdRoutine(
        sysConfig,
        new SysIdRoutine.Mechanism(voltage -> {safeVoltage(voltage);}, log -> {logMotors(log);}, this)
    );

    public void logMotors(SysIdRoutineLog log) {
        log.motor("shooter-wheel")
                    .voltage(Voltage.ofBaseUnits(motor.getBusVoltage()*motor.getAppliedOutput()*-1, Volts))
                    .angularPosition(Rotations.of(encoder.getPosition()*-1))
                    .angularVelocity(
                        RotationsPerSecond.of(encoder.getVelocity()*-1));
    }

    public void safeVoltage(Voltage voltage) {
        motor.set(voltage.baseUnitMagnitude()/motor.getBusVoltage());
    }

    public void Go() {
        
        motor.setVoltage(
        m_controller.calculate(encoder.getPosition())
            + m_feedforward.calculate(m_controller.getSetpoint().velocity));
    }

    public void Stop() {
        motor.set(0);
    }

    //Probably down
    public Command dynamicForwardTest() {
        return routine.dynamic(Direction.kForward);
    }

    //Probably up
    public Command dynamicBackwardTest() {
        return routine.dynamic(Direction.kReverse);
    }

    //Also probably down
    public Command staticForwardTest() {
        return routine.quasistatic(Direction.kForward);
    }

    //Also probably up
    public Command staticBackwardTest() {
        return routine.quasistatic(Direction.kReverse);
    }

    @Override
    public void periodic() {
        if(motor.getAppliedOutput() > 0 && bottomSwitch.get()) {
            motor.set(0);
        }
    }

}