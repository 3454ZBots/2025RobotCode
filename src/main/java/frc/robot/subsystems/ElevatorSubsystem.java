package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class ElevatorSubsystem extends SubsystemBase {


    SparkMax motor = new SparkMax(9999, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();

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

    // Creates a SysIdRoutine
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(motor::setVoltage, log -> {logMotors(log);}, this)
    );

    public void logMotors(SysIdRoutineLog log) {
        log.motor("shooter-wheel")
                    .voltage(Voltage.ofBaseUnits(motor.getBusVoltage()*motor.getAppliedOutput(), Volts))
                    .angularPosition(Rotations.of(encoder.getPosition()))
                    .angularVelocity(
                        RotationsPerSecond.of(encoder.getVelocity()));
    }

    public void Go() {
        motor.setVoltage(
        m_controller.calculate(encoder.getPosition())
            + m_feedforward.calculate(m_controller.getSetpoint().velocity));
    }

    public void Stop() {
        motor.set(0);
    }


}