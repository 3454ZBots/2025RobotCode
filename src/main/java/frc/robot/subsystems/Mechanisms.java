package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MechanismConstants;

public class Mechanisms extends SubsystemBase{
    
    private SparkMax elevatorRight;
    private SparkMax elevatorLeft;
    private SparkMax coral;
    private SparkClosedLoopController elevatorPIDright;
    private SparkClosedLoopController elevatorPIDleft;
    private DigitalInput opticalSensor;

    private SparkMax algaeRoller;
    private SparkMax algaeWrist;

    boolean algaeActivated = false;

    public Mechanisms(){

            elevatorRight = new SparkMax(MechanismConstants.ELEVATOR_RIGHT_ID, MotorType.kBrushless);
            elevatorLeft = new SparkMax(MechanismConstants.ELEVATOR_LEFT_ID, MotorType.kBrushless);
            coral = new SparkMax(MechanismConstants.CORAL_ID, MotorType.kBrushless);

            elevatorPIDright = elevatorRight.getClosedLoopController();
            elevatorPIDleft = elevatorLeft.getClosedLoopController();

           //Three motors (Coral Intake) (2 elevator motors, L and R),
           //all neos, one optical sensor (Stops intake of coral), two encoders

           //Algae: 1 motor for roller, 1 motor for "wrist"
           //SparkMAX's ?? Or old Spark's 
           //Any optical/ultrasonic sensors for the algae?
           algaeRoller = new SparkMax(-9999, MotorType.kBrushless);
           algaeWrist = new SparkMax(-9999, MotorType.kBrushless);

            
    }

    public void coralTrough(){

        elevatorPIDright.setReference(MechanismConstants.CORAL_TROUGH_HEIGHT, SparkMax.ControlType.kPosition);
        elevatorPIDleft.setReference(MechanismConstants.CORAL_TROUGH_HEIGHT, SparkMax.ControlType.kPosition);
        

    }

    public void coralLow(){

        elevatorPIDright.setReference(MechanismConstants.CORAL_LOW_HEIGHT, SparkMax.ControlType.kPosition);
        elevatorPIDleft.setReference(MechanismConstants.CORAL_LOW_HEIGHT, SparkMax.ControlType.kPosition);

    }

    public void coralMedium(){

        elevatorPIDright.setReference(MechanismConstants.CORAL_MEDIUM_HEIGHT, SparkMax.ControlType.kPosition);
        elevatorPIDleft.setReference(MechanismConstants.CORAL_MEDIUM_HEIGHT, SparkMax.ControlType.kPosition);
    }

    public void coralHigh(){

        elevatorPIDright.setReference(MechanismConstants.CORAL_HIGH_HEIGHT, SparkMax.ControlType.kPosition);
        elevatorPIDleft.setReference(MechanismConstants.CORAL_HIGH_HEIGHT, SparkMax.ControlType.kPosition);

    }

    public void intakeCoral(){
        coral.set(0.5);
        //Turn on intake for coral, needs to be turned off seperately
    }

    public void stopintake(){
        coral.set(0);
    }

    public void activateAlgae(){
        
        if (algaeActivated == false) {
            algaeRoller.set(0.5);
            
            algaeActivated = true;

        } else if(algaeActivated) {

            algaeRoller.set(-0.5);
        
    
            algaeActivated = false;
            new SequentialCommandGroup(Commands.waitSeconds(1), Commands.runOnce(()-> stopAlgae())).schedule();
        }

    }

    public void stopAlgae(){
            algaeRoller.set(0);

    } 

    public void wrist(double wristvalue){

        algaeWrist.set(wristvalue * -0.25);

    }


}
