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


    private SparkMax motor = new SparkMax(MechanismConstants.ELEVATOR_LEFT_ID, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private DigitalInput bottomSwitch = new DigitalInput(0);
    private double controllerGoal = 0;

    private static double kDt = 0.02;
    private static double kMaxVelocity = 1;
    private static double kMaxAcceleration = 0.1;
    private static double kP = 10;
    private static double kI = 0;
    private static double kD = 0.0;
    private static double kS = 0.1;
    private static double kG = 0.80;
    private static double kV = 5.0;


    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

    //10.5 dynamic test time
    private Config staticConfig = new SysIdRoutine.Config(Volts.of(0.9).per(Second), Voltage.ofBaseUnits(1.4, Volts), Time.ofBaseUnits(3.8, Second));
    private Config dynamicConfig = new SysIdRoutine.Config(Volts.of(0.9).per(Second), Voltage.ofBaseUnits(1.4, Volts), Time.ofBaseUnits(10.5, Second));

    

    public void low() {
        m_controller.setGoal(0.5);

    }

    public void trough() {

        m_controller.setGoal(0);
    }

    public void middle() {

        m_controller.setGoal(1);
    }

    public void high() {

        m_controller.setGoal(2.3);
    }



    public void runElevator() {

        SmartDashboard.putBoolean("lower limit switch", bottomSwitch.get());
        if(bottomSwitch.get() == false) {
            encoder.setPosition(0);
            
        }


        motor.setVoltage(
             m_controller.calculate(encoder.getPosition()*MechanismConstants.ELEVATOR_CONVERSION)*-1
                 + m_feedforward.calculate(m_controller.getSetpoint().velocity)*-1);

        SmartDashboard.putNumber("Elevator Height", encoder.getPosition()*MechanismConstants.ELEVATOR_CONVERSION);
        SmartDashboard.putNumber("Elevator Output", motor.getAppliedOutput()*-1);
        SmartDashboard.putNumber("Elevator FF output", m_feedforward.calculate(m_controller.getSetpoint().velocity)*-1);
        SmartDashboard.putNumber("Elevator Feedback", m_controller.calculate(encoder.getPosition()*MechanismConstants.ELEVATOR_CONVERSION)*-1);
        SmartDashboard.putNumber("Elevator Goal", m_controller.getGoal().position);
    }

}