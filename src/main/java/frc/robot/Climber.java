package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private WPI_TalonFX rung1;
    private WPI_TalonFX rung23;
    private final double CLIMBER_UP_SPEED = -0.6;
    private final double CLIMBER_DOWN_SPEED = 0.4;

    public Climber() {
        rung1 = new WPI_TalonFX(CanIDConstants.RUNG_1_2_WINCH);
        rung23 = new WPI_TalonFX(CanIDConstants.RUNG_3_4_WINCH);
        rung1.setNeutralMode(NeutralMode.Brake);
        rung23.setNeutralMode(NeutralMode.Brake);
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
        rung23.set(CLIMBER_UP_SPEED);
    }

    public void arm2Down() {
        rung23.set(CLIMBER_DOWN_SPEED);
    }

    public void arm2Hold() {
        rung23.set(0);
    }
}
