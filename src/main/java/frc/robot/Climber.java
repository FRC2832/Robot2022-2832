package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;

public class Climber {
    private WPI_TalonFX rung1;
    private WPI_TalonFX rung23;
    private XboxController operatorController;
    private final double TRIGGER_SENSITIVITY = 0.5;

    public Climber() {
        rung1 = new WPI_TalonFX(33);
        rung23 = new WPI_TalonFX(22);
        operatorController = new XboxController(1);
    }

    public void runClimber() {
        if(operatorController.getLeftBumperPressed()){ //raise center arm
            rung1.set(ControlMode.Velocity, 400);
        }

        if(operatorController.getLeftTriggerAxis() >= TRIGGER_SENSITIVITY){ //lower center arm
            rung1.set(ControlMode.Velocity, -400);
        }
        if(operatorController.getRightBumperPressed()){ //raise double arm
            rung23.set(ControlMode.Velocity, 400);
        }

        if(operatorController.getRightTriggerAxis() >= TRIGGER_SENSITIVITY){ //lower double arm
            rung23.set(ControlMode.Velocity, -400);
        }
    }    
    
    /*public void raiseRung1Arm(){
        rung1.set(ControlMode.Velocity, 1000);
    }
    public void lowerRung1Arm(){
        rung1.set(ControlMode.Velocity, -1000);
    }
    public void raiseRung2Arm(){
        rung23.set(ControlMode.Velocity, 1000);
    }
    public void lowerRung2Arm(){
        rung23.set(ControlMode.Velocity, -1000);
    }*/
}
