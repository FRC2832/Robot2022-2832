package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;

public class Ingestor {
    private WPI_TalonSRX ingestorWheels;
    private ColorSensorV3 stage2ColorSensor;
    //private WPI_TalonSRX ingestorGate;
    private WPI_TalonSRX stage1Conveyor;
    private WPI_TalonSRX stage2Conveyor;
    private CANSparkMax ingestorLift;
    private CANSparkMax.MotorType brushless;
    private XboxController driverController;
    private XboxController operatorController;
    private final double TRIGGER_SENSITIVITY = 0.5;
    
    public Ingestor(){
        ingestorWheels = new WPI_TalonSRX(27);
        //ingestorGate = new WPI_TalonSRX(2);
        stage1Conveyor = new WPI_TalonSRX(25);
        stage2Conveyor = new WPI_TalonSRX(23);
        ingestorLift = new CANSparkMax(31, brushless);
        driverController = new XboxController(0);
        operatorController = new XboxController(1);
        Port port = Port.kOnboard; // TODO: Need to verify this.
        stage2ColorSensor = new ColorSensorV3(port);
    }

    public void runIngestor(){
        // driver controller conditions
        if (driverController.getLeftBumperPressed()){ //ingest ball
            ingestorWheels.set(ControlMode.Velocity, 1000);
        }
        if (driverController.getRightBumperPressed()){ //expell ball
            ingestorWheels.set(ControlMode.Velocity, -1000);
        }
        
        // color sensor conditions
        if(stage2ColorSensor.getProximity() > 1000){
            System.out.println("getProximity() > 1000");
        }
        
        Color sensorColor = stage2ColorSensor.getColor();
        if (stage2ColorSensor.getBlue() > 128) {
            System.out.println("getBlue() returned more than 128");
        }
        if (stage2ColorSensor.getRed() > 128) {
            System.out.println("getRed() returned more than 128");
        }
        if (sensorColor.blue > 128.0) {
            System.out.println("sensorColor.blue is more than 128.0");
        }
        if (sensorColor.red > 128.0) {
            System.out.println("sensorColor.red is more than 128.0");
        }
        /*if (operatorController.getXButtonPressed()){
            ingestorGate.set(ControlMode.Velocity, 1000);
        }*/ //needs to be in shooter.java class
        
        // operator controller conditions
        if(operatorController.getAButtonPressed()){ //push ball to stage 2
            stage1Conveyor.set(1000);
        }
        if(operatorController.getBButtonPressed()){ //push ball to ingestor
            stage1Conveyor.set(-1000);
        }
        if(operatorController.getXButtonPressed()){ //push ball to shooter
            stage2Conveyor.set(1000);
        }
        if(operatorController.getYButtonPressed()){ //push ball to stage 1
            stage2Conveyor.set(-1000);
        }
        if(driverController.getLeftTriggerAxis() >= TRIGGER_SENSITIVITY){ //lift ingestor
            ingestorLift.set(1000);
        }
        if(driverController.getRightTriggerAxis() >= TRIGGER_SENSITIVITY){ //lower ingestor
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