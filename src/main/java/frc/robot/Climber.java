package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ingestor;
import frc.robot.ColorSensor;

public class Climber extends SubsystemBase {
    private WPI_TalonFX rung1;
    private WPI_TalonFX rung23;
    private final double CLIMBER_UP_SPEED = -0.6;
    private final double CLIMBER_DOWN_SPEED = 0.4;
    private Ingestor ingestor;
    private ColorSensor colorSensor;

    public Climber() {
        rung1 = new WPI_TalonFX(CanIDConstants.rung12Winch);
        rung23 = new WPI_TalonFX(CanIDConstants.rung34Winch);
        rung1.setNeutralMode(NeutralMode.Brake);
        rung23.setNeutralMode(NeutralMode.Brake);
        colorSensor = new ColorSensor();
        ingestor = new Ingestor(colorSensor);
    }

    public void arm1Up() {
        rung1.set(CLIMBER_UP_SPEED);
    }

    public void arm1Down() {
        rung1.set(CLIMBER_DOWN_SPEED);
    }

    public void arm1Hold() {
        rung1.set(0);
    }
    
    public void arm2Up() {
        ingestor.lowerIngestor(0.5);
        rung23.set(CLIMBER_UP_SPEED);
    }

    public void arm2Down() {
        ingestor.liftIngestor();
        rung23.set(CLIMBER_DOWN_SPEED);
    }

    public void arm2Hold() {
        rung23.set(0);
    }
}
