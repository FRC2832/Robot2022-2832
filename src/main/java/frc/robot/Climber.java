package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;

public class Climber {
    private WPI_TalonSRX liftGate;
    private WPI_TalonFX shootMotor;
    private XboxController operatorController;

    public Climber(){
        liftGate = new WPI_TalonSRX(3);
        shootMotor = new WPI_TalonFX(4);
        operatorController = new XboxController(1);
    }

    public void runClimber(){
        
    }
}
