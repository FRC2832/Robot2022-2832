package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;

public class Ingestor {
    private WPI_TalonSRX ingestorWheels;
    //private WPI_TalonSRX ingestorGate;
    private WPI_TalonSRX stage1Conveyor;
    private WPI_TalonSRX stage2Conveyor;
    private CANSparkMax ingestorLift;
    private CANSparkMax.MotorType brushless = CANSparkMax.MotorType.valueOf("kBrushless");
    // private XboxController driverController;
    private XboxController operatorController;
    private final double TRIGGER_SENSITIVITY = 0.5;

    public Ingestor(){
        ingestorWheels = new WPI_TalonSRX(27);
        //ingestorGate = new WPI_TalonSRX(2);
        stage1Conveyor = new WPI_TalonSRX(25);
        stage2Conveyor = new WPI_TalonSRX(23);
        ingestorLift = new CANSparkMax(31, brushless);
        // driverController = new XboxController(0);
        operatorController = new XboxController(2);
    }

    public void runIngestor(){
        if (operatorController.getLeftBumperPressed()){ //ingest ball
            ingestorWheels.set(ControlMode.Velocity, 1000);
        }
        if (operatorController.getRightBumperPressed()){ //expell ball
            ingestorWheels.set(ControlMode.Velocity, -1000);
        }
        /*if (operatorController.getXButtonPressed()){
            ingestorGate.set(ControlMode.Velocity, 1000);
        }*/ //needs to be in shooter.java class
        // TODO: try ifPressed and ifReleased instead of the else
        if(operatorController.getAButton()){ //push ball to stage 2
            stage1Conveyor.set(1000);
        } else {
            stage1Conveyor.set(0);
        }
        if(operatorController.getBButton()){ //push ball to ingestor
            stage1Conveyor.set(-1000);
        } else {
            stage1Conveyor.set(0);
        }
        if(operatorController.getXButton()){ //push ball to shooter
            stage2Conveyor.set(1000);
        } else {
            stage2Conveyor.set(0);
        }
        if(operatorController.getYButton()){ //push ball to stage 1
            stage2Conveyor.set(-1000);
        } else {
            stage2Conveyor.set(0);
        }
        if(operatorController.getLeftTriggerAxis() >= TRIGGER_SENSITIVITY){ //lift ingestor
            ingestorLift.set(1000);
        }
        if(operatorController.getRightTriggerAxis() >= TRIGGER_SENSITIVITY){ //lower ingestor
            ingestorLift.set(-1000);
        }
    }

    /*public void ingest(){
        ingestorWheels.set(ControlMode.Velocity, 1000);
    }

    public void expel(){
        ingestorWheels.set(ControlMode.Velocity, -1000);
    }
    
    public void stage1(){
        
    }*/
}