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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static double kMaxVelocity = 0.7;
    private static double kMaxAcceleration = 0.06;
    private static double kP = 0.0059971;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kS = 0.46363;
    private static double kG = 0.59113;
    private static double kV = 1.08166;


    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);
    
    //10.5 dynamic test time
    private Config staticConfig = new SysIdRoutine.Config(Volts.of(0.9).per(Second), Voltage.ofBaseUnits(1.4, Volts), Time.ofBaseUnits(3.8, Second));
    private Config dynamicConfig = new SysIdRoutine.Config(Volts.of(0.9).per(Second), Voltage.ofBaseUnits(1.4, Volts), Time.ofBaseUnits(10.5, Second));

    // Creates a SysIdRoutine
    SysIdRoutine dynamicRoutine = new SysIdRoutine(
        dynamicConfig,
        new SysIdRoutine.Mechanism(voltage -> {safeVoltage(voltage);}, log -> {logMotors(log);}, this)
    );

    SysIdRoutine staticRoutine = new SysIdRoutine(
        staticConfig,
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
        m_controller.setGoal(2);
        
    }

    public void Stop() {
        
        m_controller.setGoal(0);
    }

    //Probably down
    public Command dynamicForwardTest() {
        return dynamicRoutine.dynamic(Direction.kForward);
    }

    //Probably up
    public Command dynamicBackwardTest() {
        return dynamicRoutine.dynamic(Direction.kReverse);
    }

    //Also probably down
    public Command staticForwardTest() {
        return staticRoutine.quasistatic(Direction.kForward);
    }

    //Also probably up
    public Command staticBackwardTest() {
        return staticRoutine.quasistatic(Direction.kReverse);
    }



    public void runElevator() {
        motor.setVoltage(
            m_controller.calculate(encoder.getPosition()*-1)
                + m_feedforward.calculate(m_controller.getSetpoint().velocity)*-1);

        SmartDashboard.putNumber("Elevator Height", encoder.getPosition()*-1);
        SmartDashboard.putNumber("Elevator Output", motor.getAppliedOutput()*-1);
        SmartDashboard.putNumber("Elevator FF output", m_feedforward.calculate(m_controller.getSetpoint().velocity)*-1);
        SmartDashboard.putNumber("Elevator Feedback", m_controller.calculate(encoder.getPosition()*-1));
        SmartDashboard.putNumber("Elevator Goal", m_controller.getGoal().position);
    }

}