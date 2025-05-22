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
    
    
    private SparkMax coral;

    private SparkFlex algaeRoller;
    private SparkMax algaeWrist;

    private boolean coralFast = false;
    private boolean coralSlow = false
    ;

    private boolean algaeFront = false;
    private boolean algaeBack = false;

    int algaeActivated = 0;

    public MechanismSubsystem() {

            //elevatorRight = new SparkMax(MechanismConstants.ELEVATOR_RIGHT_ID, MotorType.kBrushless);

            //May not need to define this one
            //elevatorLeftWhichIsAFollowerSoDoNotUseIt = new SparkMax(MechanismConstants.ELEVATOR_LEFT_ID, MotorType.kBrushless);
            
            
            coral = new SparkMax(MechanismConstants.CORAL_ID, MotorType.kBrushless);

            //elevatorPIDright = elevatorRight.getClosedLoopController();
            //elevatorPIDleft = elevatorLeft.getClosedLoopController();

           //Three motors (Coral Intake) (2 elevator motors, L and R),
           //all neos, one optical sensor (Stops intake of coral), two encoders

           //Algae: 1 motor for roller, 1 motor for "wrist"
           //SparkMAX's ?? Or old Spark's 
           //Any optical/ultrasonic sensors for the algae?
           algaeRoller = new SparkFlex(MechanismConstants.ALGAE_ROLLER_ID, MotorType.kBrushless);
           algaeWrist = new SparkMax(MechanismConstants.ALGAE_WRIST_ID, MotorType.kBrushless);
           SmartDashboard.putBoolean("Coral Slow", coralSlow);
           SmartDashboard.putBoolean("Coral Fast", coralFast);
           SmartDashboard.putBoolean("Algae Front", algaeFront);
           SmartDashboard.putBoolean("Algae Back", algaeBack);
            
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

    public void slowCoral(){
        if(coral.get() != 0) {
            coral.set(0);
            coralFast = false;
            coralSlow = false;
        }
        else {
            coral.set(0.3);
            coralFast = false;
            coralSlow = true;
        }
        SmartDashboard.putBoolean("Coral Slow", coralSlow);
        SmartDashboard.putBoolean("Coral Fast", coralFast);
        
    }

    public void stopCoral(){
        coral.set(0);
        coralFast = false;
        coralSlow = false;
        SmartDashboard.putBoolean("Coral Slow", coralSlow);
        SmartDashboard.putBoolean("Coral Fast", coralFast);
    }

    public void fastCoral() {
        if(coral.get() != 0) {
            coral.set(0);
            coralFast = false;
            coralSlow = false;
        }
        else {
            coral.set(0.7);
            coralFast = true;
            coralSlow = false;
        }
        SmartDashboard.putBoolean("Coral Slow", coralSlow);
        SmartDashboard.putBoolean("Coral Fast", coralFast);

        
    }


    public void backAlgae() {
        if(algaeRoller.getAppliedOutput() != 0) {
            algaeRoller.set(0);
            algaeBack = false;
            algaeFront = false;
            
        }
        else if(algaeRoller.getAppliedOutput() == 0) {
            algaeRoller.set(-0.5);
            algaeBack = true;
            algaeFront = false;
        }
        SmartDashboard.putBoolean("Algae Front", algaeFront);
        SmartDashboard.putBoolean("Algae Back", algaeBack);
    }

    public void frontAlgae() {
        if(algaeRoller.getAppliedOutput() != 0) {
            algaeRoller.set(0);
            algaeBack = false;
            algaeFront = false;
        }
        else if(algaeRoller.getAppliedOutput() == 0) {
            algaeRoller.set(0.5);
            algaeFront = true;
            algaeBack = false;
        }
        SmartDashboard.putBoolean("Algae Front", algaeFront);
        SmartDashboard.putBoolean("Algae Back", algaeBack);
    }

    public void stopAlgae(){
        algaeRoller.set(0);
        algaeBack = false;
        algaeFront = false;
        SmartDashboard.putBoolean("Algae Front", algaeFront);
        SmartDashboard.putBoolean("Algae Back", algaeBack);
    } 

    public void wrist(double wristvalue) {
        


        algaeWrist.set(wristvalue * 0.2);
        // elevatorRight.set(elevatorvalue * 0.3);

        // if(elevatorvalue > 0 && !bottomSwitch.get()) {
        //     elevatorRight.set(0);
        // } else if(!topSwitch.get() && elevatorvalue < 0 ) {


        // }
    }


}
