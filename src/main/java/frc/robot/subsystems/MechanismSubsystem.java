package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MechanismConstants;

public class MechanismSubsystem extends SubsystemBase{
    
    private SparkMax elevatorRight;
    private SparkMax elevatorLeftWhichIsAFollowerSoDoNotUseIt;
    private SparkMax coral;
    private SparkClosedLoopController elevatorPIDright;
    private SparkClosedLoopController elevatorPIDleft;
    private DigitalInput opticalSensor;

    private SparkFlex algaeRoller;
    private SparkMax algaeWrist;

    int algaeActivated = 0;
    //private DigitalInput bottomSwitch = new DigitalInput(0);
    //private DigitalInput topSwitch = new DigitalInput(1);

    public MechanismSubsystem(){

            //elevatorRight = new SparkMax(MechanismConstants.ELEVATOR_RIGHT_ID, MotorType.kBrushless);

            //May not need to define this one
            //elevatorLeftWhichIsAFollowerSoDoNotUseIt = new SparkMax(MechanismConstants.ELEVATOR_LEFT_ID, MotorType.kBrushless);
            
            
            coral = new SparkMax(MechanismConstants.CORAL_ID, MotorType.kBrushless);

            //elevatorPIDright = elevatorRight.getClosedLoopController();
           // elevatorPIDleft = elevatorLeft.getClosedLoopController();

           //Three motors (Coral Intake) (2 elevator motors, L and R),
           //all neos, one optical sensor (Stops intake of coral), two encoders

           //Algae: 1 motor for roller, 1 motor for "wrist"
           //SparkMAX's ?? Or old Spark's 
           //Any optical/ultrasonic sensors for the algae?
           algaeRoller = new SparkFlex(MechanismConstants.ALGAE_ROLLER_ID, MotorType.kBrushless);
           algaeWrist = new SparkMax(MechanismConstants.ALGAE_WRIST_ID, MotorType.kBrushless);
            
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Down Elevator", bottomSwitch.get());
        // SmartDashboard.putBoolean("Up Elevator", topSwitch.get());
        // SmartDashboard.putNumber("Elevator Motor Output", elevatorRight.getAppliedOutput());
        
        // if(elevatorRight.getAppliedOutput() > 0 && !bottomSwitch.get()) {
        //     elevatorRight.set(0);
        // } else if(!topSwitch.get() && elevatorRight.getAppliedOutput() < 0 ){


        // }

        
    }


    // public void coralTrough(){

    //     elevatorPIDright.setReference(MechanismConstants.CORAL_TROUGH_HEIGHT, SparkMax.ControlType.kPosition);
    //     elevatorPIDleft.setReference(MechanismConstants.CORAL_TROUGH_HEIGHT, SparkMax.ControlType.kPosition);
        

    // }

    // public void coralLow(){

    //     elevatorPIDright.setReference(MechanismConstants.CORAL_LOW_HEIGHT, SparkMax.ControlType.kPosition);
    //     elevatorPIDleft.setReference(MechanismConstants.CORAL_LOW_HEIGHT, SparkMax.ControlType.kPosition);

    // }

    // public void coralMedium(){

    //     elevatorPIDright.setReference(MechanismConstants.CORAL_MEDIUM_HEIGHT, SparkMax.ControlType.kPosition);
    //     elevatorPIDleft.setReference(MechanismConstants.CORAL_MEDIUM_HEIGHT, SparkMax.ControlType.kPosition);
    // }

    // public void coralHigh(){

    //     elevatorPIDright.setReference(MechanismConstants.CORAL_HIGH_HEIGHT, SparkMax.ControlType.kPosition);
    //     elevatorPIDleft.setReference(MechanismConstants.CORAL_HIGH_HEIGHT, SparkMax.ControlType.kPosition);

    // }

    public void intakeCoral(){
        coral.set(0.5);
        //Turn on intake for coral, needs to be turned off seperately
    }

    public void stopIntake(){
        coral.set(0);
    }


    public void oneAlgae() {
        if(algaeRoller.getAppliedOutput() != 0) {
            algaeRoller.set(0);
        }
        else if(algaeRoller.getAppliedOutput() == 0) {
            algaeRoller.set(-0.5);
        }
    }

    public void twoAlgae() {
        if(algaeRoller.getAppliedOutput() != 0) {
            algaeRoller.set(0);
        }
        else if(algaeRoller.getAppliedOutput() == 0) {
            algaeRoller.set(0.5);
        }
    }

    public void stopAlgae(){
            algaeRoller.set(0);
    } 

    public void wrist(double wristvalue, double elevatorvalue){
        if(elevatorvalue < 0.05 && elevatorvalue > -0.05) {
            elevatorvalue = 0;
        }


        algaeWrist.set(wristvalue * 0.15);
        // elevatorRight.set(elevatorvalue * 0.3);

        // if(elevatorvalue > 0 && !bottomSwitch.get()) {
        //     elevatorRight.set(0);
        // } else if(!topSwitch.get() && elevatorvalue < 0 ) {


        // }
    }


}
