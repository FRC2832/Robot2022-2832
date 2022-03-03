package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;

public class Ingestor {
    private WPI_TalonSRX ingestorWheels;
    //private WPI_TalonSRX ingestorGate;
    private CANSparkMax ingestorConveyor;
    private XboxController driverController;
    private XboxController operatorController;
    private CANSparkMaxLowLevel kBrushless;

    public Ingestor(){
        ingestorWheels = new WPI_TalonSRX(1);
        //ingestorGate = new WPI_TalonSRX(2);
        ingestorConveyor = new CANSparkMax(3, MotorType.kBrushless);
        driverController = new XboxController(0);
        operatorController = new XboxController(1);
    }

    public void runIngestor(){
        if (driverController.getLeftBumperPressed()){
            ingestorWheels.set(ControlMode.Velocity, 1000);
        }
        if (driverController.getRightBumperPressed()){
            ingestorWheels.set(ControlMode.Velocity, -1000);
        }
        /*if (operatorController.getXButtonPressed()){
            ingestorGate.set(ControlMode.Velocity, 1000);
        }*/ //needs to be in shooter.java class
        if(operatorController.getAButtonPressed()){
            ingestorConveyor.set(1000);
        }
        if(operatorController.getBButtonPressed()){
            ingestorConveyor.set(-1000);
        }
    }
}